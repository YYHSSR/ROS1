#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 graduation_detect.py 发布的黄线像素数据控制阿克曼小车巡线。

核心巡线控制逻辑：
恒定巡航速度 + 目标列像素偏差的 PD 控制；
每次回调都是"先算出 (lin_vel, ang_vel)，再统一限幅、最后发布一次"，
与 follow_lane_contest.py / detect_lane_contest.py 的黄色车道线巡线
部分保持同样的结构，便于对照调参。
本节点订阅检测结果、右侧黄线点 /lane_detect_right_points 与 /odom（后两者
仅用于盲区防压线保护），向 /cmd_vel 发布 Twist。

针对"黄线局部缺失时要能直行通过""弯曲/起伏路段要自动纠偏且不压线"
这两个要求，在原有 P 控制基础上做了四处专门处理：
  1. 缺口直行（gap hold）：前方可见黄线变短（即将到达缺口）或看不到
     参考列时直行，让车按缺口前的方向直接开过缺口。缺口前黄线末端会
     向外侧弯，如果继续跟着它转，车会斜着冲过缺口并压线。
  2. 丢线搜索：连续丢线超过容忍帧数后，边前进边按 turn_ang_vel 转向
     找线，而不是停车——停车后阿克曼底盘无法原地转向，会永久卡死。
  3. PD 控制：在原来的比例项之外增加微分项，抑制弯曲/起伏路段因误差
     快速变化而产生的转向超调（画龙/压线），让转向能提前跟上误差变化
     的趋势，而不是等误差已经很大才纠正。
  4. 盲区防压线保护：相机看不到车身旁边约 0.95 m 内的地面，弯道处 PD
     会提前转向切弯。记住看到过的左侧黄线和右侧黄线地面点，预测转向后
     车身两侧是否会贴近黄线，会贴近就先少转/不转，等车身越过拐角再转。

输入 /lane_detect_pose 的约定：
  position.x  目标黄线在参考行附近的像素列，仅用于观察，缺口处可为 -1
  position.y  下方中央 ROI 黄色像素纵向中心 (px)，用于丢线时判断搜索方向
  position.z  本帧跟踪到的黄线像素数
  orientation.x  跟踪到的黄线最远端所在图像行号，无像素时为 -1
  orientation.y / orientation.z  position.x 对应黄线点在地面上距 base_link
                 的前方/左侧距离 (m)，position.x 为 -1 时无效

修改字段含义时，必须同时修改检测节点 graduation_detect.py 与本控制节点。
"""

import math
import threading
import time
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Polygon, Pose, Twist
from nav_msgs.msg import Odometry


class LaneFollowController:
    """恒速 + 目标列像素 PD 控制，短暂丢线直行，超时看门狗停车。

    控制流程（对应 pose_callback）：
      1. 即将到缺口或短暂丢线（未超过 gap_hold_frames） -> 直行开过黄线缺口
      2. 持续丢线（超过容忍帧数）                   -> 前进 + 按 y 判断转向搜索 / 直行
      3. 正常检测到黄线（x >= 0）                   -> 目标列像素误差 PD 控制转向
      以上结果最后都经过盲区防压线保护，换掉会让车身两侧贴近黄线的角速度。
    """

    def __init__(self):
        """读取 ROS 私有参数并建立检测输入、速度输出与看门狗。"""
        # 【调试】巡航速度 (m/s)：正常巡线与丢线搜索都使用该速度，与
        # follow_lane_contest.py 的 lin_vel=0.19 一致。想让车跑快/慢，改这个。
        self.cruise_speed = float(rospy.get_param("~cruise_speed", 0.19))
        # 【调试】目标列像素坐标：黄线应停留在画面中的哪一列，车身即保持在
        # 其右侧车道内。车贴着线跑偏左/偏右，就调大/调小这个值
        # （对应 follow_lane_contest.py 里的 target_x）。第 320 行上每像素
        # 约 2.2 mm：140 时黄线在车身中心左侧约 0.40 m（左轮外沿净距约
        # 0.20 m），115 时约 0.45 m（净距约 0.25 m，接近车道中间）。
        # 必须与下面的 side_margin 匹配：side_margin 大于这里对应的净距时，
        # 防压线保护会一直拦住 PD 的左转，弯道处黄线容易移出画面。
        self.target_x = float(rospy.get_param("~target_x", 115.0))
        # 【调试】比例增益：像素误差 -> 角速度 (rad/s per px)。转向太"肉"就
        # 调大，转向抖动/画龙就调小（对应 follow_lane_contest.py 的 0.007）。
        self.kp = float(rospy.get_param("~kp", 0.007))
        # 【调试】微分增益：像素误差变化率 -> 角速度 (rad/s per (px/frame))。
        # 用于抑制弯曲/起伏路段的转向超调（画龙/压线）：误差变化快时提前
        # 施加一个反向修正，让转向提前跟上趋势而不是等误差变大才纠正。
        # 默认 0.0（等效原来的纯 P 控制）；如果过弯时能看到明显"先冲出去
        # 再往回纠正"的画龙现象，可以从 0.001~0.003 开始尝试调大；调太大
        # 会引入高频抖动（因为像素检测本身有噪声，微分项会放大噪声）。
        self.kd = float(rospy.get_param("~kd", 0.0))
        # 【调试】持续丢线后边前进边搜索的转向角速度 (rad/s)，负值=右转。
        # follow_lane_contest.py 用的是 -1.2（转弯更急）；这里调得更保守。
        # 缺口/弯道后重新找线太慢、冲出赛道就调大绝对值；找线时转过头
        # 压到另一侧线就调小绝对值。
        self.turn_ang_vel = float(rospy.get_param("~turn_ang_vel", -0.38))
        # 【调试】角速度限幅 (rad/s)，与 follow_lane_contest.py 的 ±0.8 一致。
        self.max_ang_vel = float(rospy.get_param("~max_ang_vel", 0.8))
        self.min_ang_vel = -self.max_ang_vel
        # 【调试】判断"完全丢线"的最少像素数阈值；低于该值按丢线处理
        # （对应 follow_lane_contest.py 的 z <= 5）。
        self.min_line_pixels = float(rospy.get_param("~min_line_pixels", 5))
        # 【调试】丢线时的搜索方向判断阈值：position.y 不大于该值时转向
        # 搜索，否则直行（对应 follow_lane_contest.py 的 y <= 340）。
        self.side_hint_threshold = float(rospy.get_param("~side_hint_threshold", 340.0))
        # 【调试】检测消息中断超过该时长（秒）即视为失效，看门狗强制停车。
        # follow_lane_contest.py 没有这层保护，这里额外加上防止检测节点
        # 卡死/崩溃时小车保留旧速度指令继续冲。
        self.message_timeout = float(rospy.get_param("~message_timeout", 0.4))
        # 【调试】缺口直行帧数：从"前方黄线变短/看不到参考列"开始，最多直行
        # 多少帧再转为向右搜索。实测两处缺口从黄线末端开始外弯到重新看到
        # 缺口后的黄线约 0.8 m，0.19 m/s 下约 43 帧（检测约 10 Hz），这里留
        # 余量取 60 帧（约 6 s）。车速改了要按比例改这个值；调小会在缺口
        # 中途就向右转、冲进内场；调大只影响真正跑出赛道时多久开始搜索。
        self.gap_hold_frames = int(rospy.get_param("~gap_hold_frames", 60))
        # 【调试】"前方黄线即将结束"的判断行号：跟踪到的黄线最远端行号
        # (orientation.x) 不小于该值、且列坐标不在目标列右侧时，就认为前方
        # 快到缺口，立即停止跟随列坐标、直行。正常巡线时最远端约为 241
        # （跟踪窗口顶部），左上角右转弯中约 280~301；两处缺口前在最远端到
        # 300 左右时黄线开始向外弯。调小会在弯道中误判为缺口而直行；调大
        # 则会先跟着末端外弯转一段再直行，车头已偏向外侧，容易压线。
        self.line_end_row = float(rospy.get_param("~line_end_row", 298.0))
        # 【调试】盲区防压线保护：相机只能看到车头前方约 0.95 m 以外的地面，
        # 车身旁边的黄线看不到。按目标列转向相当于瞄着前方约 1 m 处的点，
        # 弯道处会提前转向切弯：左拐时左侧车身压到左侧黄线拐角，右拐时
        # 右侧车身扫到弯道内侧的右侧黄线线端。这里把看到过的左、右黄线点
        # 按里程计记下来，对 PD 给出的角速度做预测检查：沿该角速度往前推
        # side_horizon 秒，途中任一侧离记忆黄线小于 side_margin/right_margin
        # 就换一个最接近的角速度，车会先直行让车身越过拐角再转；车头背离
        # 黄线时不受限制，可以正常回正。
        # body_half_width：车身中心线到左侧车轮外沿的距离 (m)，由 URDF
        #   车轮位置 ±0.154 m、轮宽 0.099 m 得到，换车才需要改。
        self.body_half_width = float(rospy.get_param("~body_half_width", 0.20))
        # side_margin：左轮外沿与左侧黄线中心之间至少保留的距离 (m)，含黄线
        #   自身半宽约 0.03 m。实测 target_x=140 配 0.20 时顶部凸起处最近
        #   约 14 cm，target_x=115 配 0.25 时约 19 cm。不要大于 target_x
        #   对应的直道净距（见 target_x 注释）。调大离线更远，但车会更晚
        #   左转，也会把车往右侧线推；调小则凸起拐角处更贴线。
        self.side_margin = float(rospy.get_param("~side_margin", 0.25))
        # side_horizon：沿候选角速度往前预测多少秒来检查净距。调大更早
        #   发现前方拐角、更早收住左转，但也更保守、更容易让黄线移出画面；
        #   调小则反应更晚，拐角处容易切进去。
        self.side_horizon = float(rospy.get_param("~side_horizon", 1.5))
        # right_margin：右轮外沿与右侧黄线（右侧道路线/内侧街区边线）之间
        #   至少保留的距离 (m)。右转弯时 PD 同样会提前转向切弯，右侧车身
        #   扫到弯道内侧的线端；调大右转更晚、更靠外侧，太大会被逼向左侧
        #   黄线；调小则更容易切弯压到右侧线。
        self.right_margin = float(rospy.get_param("~right_margin", 0.15))
        # turn_rate_limit：底盘实际能达到的最大转向角速度 (rad/s)，仅用于
        #   预测。实测 0.19 m/s 下右转约 0.4、左转约 0.25，指令再大也转不动；
        #   换车速或底盘时要重新估计。
        self.turn_rate_limit = float(rospy.get_param("~turn_rate_limit", 0.4))
        self.side_dt = 0.1
        self.side_steps = max(1, int(round(self.side_horizon / self.side_dt)))
        # side_check_front：检查车头前方多远范围内的记忆黄线 (m)。车身前沿
        #   在 0.22 m 处，略微向前多看一点能提前避开拐角；调得太大会把前方
        #   正常弯道也当成贴线，平白限制左转。
        self.side_check_front = float(rospy.get_param("~side_check_front", 0.45))
        self.side_check_rear = -0.30
        # 后轴中心在 base_link 后方的距离 (m)，取自 URDF 后轮位置。
        self.rear_axle_offset = 0.22
        # 启动时保持静止，直到第一次检测到黄线，避免开局无画面时误转向。
        self.found_line = False
        self.last_message_time = None
        # 丢线容忍与 PD 控制所需的状态量（内部状态，不建议手动改）。
        self.lost_frame_count = 0
        self.prev_error = 0.0
        self.prev_line_missing = False
        # 里程计位姿 (x, y, yaw) 与记忆的黄线地面点（里程计坐标系）。
        self.odom_pose = None
        self.line_memory = deque(maxlen=120)
        self.right_memory = deque(maxlen=300)
        self.right_lock = threading.Lock()

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=5)
        self.right_sub = rospy.Subscriber(
            "/lane_detect_right_points", Polygon, self.right_callback, queue_size=1
        )
        self.pose_sub = rospy.Subscriber(
            "/lane_detect_pose", Pose, self.pose_callback, queue_size=1
        )
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.1), self.watchdog)
        rospy.on_shutdown(self.clean_shutdown)
        rospy.loginfo("[graduation_control] 纯视觉黄线巡线控制已启动")

    def _publish(self, speed, yaw_rate):
        """限幅后发布一次 Twist；左转为正、右转为负。"""
        yaw_rate = max(self.min_ang_vel, min(self.max_ang_vel, yaw_rate))
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(yaw_rate)
        self.vel_pub.publish(cmd)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def right_callback(self, msg):
        """记下检测节点给出的右侧黄线地面点。"""
        if self.odom_pose is None:
            return
        with self.right_lock:
            for p in msg.points:
                self._remember(self.right_memory, p.x, p.y)

    def _remember(self, memory, fwd, left):
        """把地面点（base_link 下）换算到里程计坐标系后存入 memory。"""
        px, py, yaw = self.odom_pose
        c, s = math.cos(yaw), math.sin(yaw)
        memory.append((px + fwd * c - left * s, py + fwd * s + left * c))

    def _clearance(self, points, px, py, yaw, side):
        """位姿 (px, py, yaw) 下车身一侧外沿到记忆黄线的最小距离 (m)。

        side=+1 看左侧，-1 看右侧；车身旁边没有记忆点时返回 None。
        """
        if points is None:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        dx = points[:, 0] - px
        dy = points[:, 1] - py
        fwd = dx * c + dy * s
        lateral = (-dx * s + dy * c) * side
        beside = (fwd >= self.side_check_rear) & (fwd <= self.side_check_front) & (lateral > 0.0)
        if not np.any(beside):
            return None
        return float(np.min(lateral[beside])) - self.body_half_width

    def _predicted_clearance(self, left_pts, right_pts, lin_vel, ang_vel):
        """按 (lin_vel, ang_vel) 行驶 side_horizon 秒，途中左/右两侧的最小净距。

        阿克曼底盘实际转向角速度有上限，超过 turn_rate_limit 的指令按上限预测；
        车身绕后轴转动（后轴不会横向甩出），所以按后轴中心积分位置。
        """
        yaw_rate = max(-self.turn_rate_limit, min(self.turn_rate_limit, ang_vel))
        px, py, yaw = self.odom_pose
        rx = px - self.rear_axle_offset * math.cos(yaw)
        ry = py - self.rear_axle_offset * math.sin(yaw)
        worst = [None, None]
        for _ in range(self.side_steps):
            yaw += yaw_rate * self.side_dt
            rx += lin_vel * self.side_dt * math.cos(yaw)
            ry += lin_vel * self.side_dt * math.sin(yaw)
            px = rx + self.rear_axle_offset * math.cos(yaw)
            py = ry + self.rear_axle_offset * math.sin(yaw)
            for i, (pts, side) in enumerate(((left_pts, 1), (right_pts, -1))):
                gap = self._clearance(pts, px, py, yaw, side)
                if gap is not None and (worst[i] is None or gap < worst[i]):
                    worst[i] = gap
        return worst

    @staticmethod
    def _shortfall(gap, required):
        """预测净距比要求少多少；满足要求（或旁边没有黄线）时为 0。"""
        if gap is None or required is None:
            return 0.0
        return max(0.0, required - gap)

    def _limit_turn(self, lin_vel, ang_vel):
        """盲区防压线保护：在期望角速度附近找一个沿预测轨迹左右两侧都不会
        比 side_margin / right_margin 更贴近记忆黄线的角速度，离期望值越近越好。

        某侧已经小于要求时，只要求不再变得更近；所有候选都不满足时选总缺口
        最小的。返回 (角速度, 当前左侧净距, 当前右侧净距)。
        """
        if self.odom_pose is None:
            return ang_vel, None, None
        left_pts = np.array(self.line_memory) if self.line_memory else None
        with self.right_lock:
            right_pts = np.array(self.right_memory) if self.right_memory else None
        now_left = self._clearance(left_pts, *self.odom_pose, side=1)
        now_right = self._clearance(right_pts, *self.odom_pose, side=-1)
        if now_left is None and now_right is None:
            return ang_vel, None, None
        req_left = req_right = None
        if now_left is not None:
            req_left = min(self.side_margin, now_left)
        if now_right is not None:
            req_right = min(self.right_margin, now_right)

        ang_vel = max(self.min_ang_vel, min(self.max_ang_vel, ang_vel))
        best_ang, best_short = ang_vel, None
        for k in range(int(4 * self.max_ang_vel / 0.05) + 1):
            offset = 0.05 * ((k + 1) // 2) * (1 if k % 2 else -1)
            candidate = ang_vel + offset
            if candidate > self.max_ang_vel + 1e-6 or candidate < self.min_ang_vel - 1e-6:
                continue
            gap_left, gap_right = self._predicted_clearance(left_pts, right_pts, lin_vel, candidate)
            short = self._shortfall(gap_left, req_left) + self._shortfall(gap_right, req_right)
            if short == 0.0:
                return candidate, now_left, now_right
            if best_short is None or short < best_short:
                best_ang, best_short = candidate, short
        return best_ang, now_left, now_right

    def pose_callback(self, msg):
        """PD 比例-微分控制，短暂丢线直行，持续丢线前进搜索。

        先在各分支里算出 (lin_vel, ang_vel)，再统一限幅、发布一次。
        """
        self.last_message_time = rospy.get_time()

        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        far_row = msg.orientation.x

        # 启动阶段：尚未首次检测到黄线时保持静止，避免瞬间误转向。
        if not self.found_line:
            if x >= 0:
                self.found_line = True
                rospy.loginfo("[graduation_control] 首次检测到黄色车道线，开始巡线。")
            else:
                self._publish(0.0, 0.0)
                return

        # 缺口前黄线末端向左外弯，列坐标会落到目标列左侧；右转弯时黄线也会
        # 从画面右侧出去、最远端行号同样变大，但列坐标在目标列右侧，不能
        # 误判为缺口直行。
        line_ending = far_row >= self.line_end_row and 0 <= x <= self.target_x
        line_missing = (z <= self.min_line_pixels) or (x < 0) or line_ending

        # ------------------------------------------------------------
        # 巡线核心逻辑：
        #   1) 即将到缺口或短暂丢线（未超过 gap_hold_frames）-> 直行开过缺口
        #   2) 持续丢线超过容忍帧数 -> 前进 + 按 y 判断转向搜索 / 直行
        #   3) 正常检测到黄线 -> 目标列像素误差 PD 控制转向
        # ------------------------------------------------------------
        just_recovered = line_missing is False and self.prev_line_missing
        if line_missing:
            self.lost_frame_count += 1
        else:
            self.lost_frame_count = 0
        self.prev_line_missing = line_missing

        if line_missing and self.lost_frame_count <= self.gap_hold_frames:
            # 缺口容忍期内：直行开过缺口。黄线末端会向外侧弯进缺口，此时
            # 不再跟随列坐标，也不沿用之前的转向，保持缺口前的车头方向。
            lin_vel = self.cruise_speed
            ang_vel = 0.0
        elif line_missing:
            # 超过容忍帧数仍丢线：边前进边按 turn_ang_vel 转向找线。阿克曼
            # 底盘线速度为 0 时无法转向，所以这里必须保持前进速度；小车沿
            # 左侧黄线顺时针跑，赛道在缺口后向右转，因此默认向右搜索。
            lin_vel = self.cruise_speed
            if y <= self.side_hint_threshold:
                ang_vel = self.turn_ang_vel
            else:
                ang_vel = 0.0
        else:
            # 正常检测到黄线：目标列像素误差做 PD 控制（比例项跟踪目标列，
            # 微分项抑制弯曲/起伏路段的转向超调，减少压线风险）。
            lin_vel = self.cruise_speed
            error = self.target_x - x
            if just_recovered:
                # 刚从缺口/丢线状态恢复：prev_error 是缺口之前的旧值，
                # 直接做差会把缺口期间的位置跳变当成"变化率"放大，造成
                # 突然的大幅转向。这一帧先只用比例项，从下一帧开始再正常
                # 使用微分项。
                derivative = 0.0
            else:
                derivative = error - self.prev_error
            ang_vel = self.kp * error + self.kd * derivative
            self.prev_error = error

        if self.odom_pose is not None and x >= 0:
            self._remember(self.line_memory, msg.orientation.y, msg.orientation.z)
        ang_vel, gap_left, gap_right = self._limit_turn(lin_vel, ang_vel)

        self._publish(lin_vel, ang_vel)

        rospy.loginfo_throttle(
            1.0, "[巡线] x=%.1f y=%.1f z=%.0f far=%.0f lin=%.2f ang=%.2f lost=%d L=%s R=%s"
            % (x, y, z, far_row, lin_vel, ang_vel, self.lost_frame_count,
               "-" if gap_left is None else "%.2f" % gap_left,
               "-" if gap_right is None else "%.2f" % gap_right)
        )

    def watchdog(self, _event):
        """检测消息中断超时后持续发送零速度，防止底盘保留旧指令。"""
        now = rospy.get_time()
        if (self.last_message_time is None
                or now - self.last_message_time > self.message_timeout):
            self._publish(0.0, 0.0)

    def clean_shutdown(self):
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(0.02)


if __name__ == "__main__":
    try:
        rospy.init_node("graduation_control", anonymous=False)
        LaneFollowController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
