#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据 graduation_detect.py 的检测结果控制阿克曼小车沿左侧黄线巡线。

每收到一帧检测结果（pose_callback），先按黄线状态选出基础指令：
  1. 正常看到黄线      -> 恒速 + 目标列像素误差 PD 控制转向。
  2. 快到缺口/短暂丢线 -> 保持车头方向直行开过缺口（最多 gap_hold_frames 帧）。
     缺口前黄线末端会向外弯，继续跟着转会斜着冲过缺口并压线。
  3. 持续丢线          -> 边前进边向右搜索（阿克曼底盘停车后无法原地转向）。
再经过盲区防压线保护修正角速度，限幅后发布到 /cmd_vel。

盲区防压线保护：相机看不到车头前方约 0.95 m 以内的地面，按目标列转向
相当于瞄着前方约 1 m 处，弯道处会提前转向切弯。节点按里程计记住看到过
的左侧黄线点和右侧黄线点，沿候选角速度预测车身两侧与这些点的净距，
选一个两侧都不贴线、又最接近 PD 输出的角速度。

订阅：
  /lane_detect_pose          检测结果（字段约定见 graduation_detect.py）
  /lane_detect_right_points  右侧黄线地面点，仅用于防压线保护
  /odom                      里程计位姿，仅用于防压线保护
发布：/cmd_vel（左转角速度为正）。检测消息中断超过 message_timeout 时看门狗发零速。

标【调试】的参数可通过 ROS 私有参数覆盖（如 _target_x:=120），注释里说明了改动的效果。
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
    """恒速 + 目标列 PD 巡线，缺口直行，持续丢线搜索，盲区防压线保护。"""

    def __init__(self):
        self._init_tracking_params()
        self._init_gap_params()
        self._init_guard_params()

        # 内部状态。启动后先保持静止，直到第一次检测到黄线。
        self.found_line = False
        self.last_message_time = None
        self.lost_frame_count = 0
        self.prev_error = 0.0
        self.prev_line_missing = False
        # 里程计位姿 (x, y, yaw) 与记忆的左/右黄线地面点（里程计坐标系）。
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

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _init_tracking_params(self):
        """巡航速度与 PD 转向。"""
        # 【调试】巡航速度 (m/s)，巡线、缺口直行与丢线搜索都用它。改了之后要
        # 按比例调整 gap_hold_frames，并重新估计 turn_rate_limit。
        self.cruise_speed = float(rospy.get_param("~cruise_speed", 0.19))
        # 【调试】目标列 (px)：黄线应保持在画面中的哪一列。车偏左（贴左线）
        # 就调大，偏右就调小。第 320 行上每像素约 2.2 mm：115 时黄线在车身
        # 中心左侧约 0.45 m（左轮外沿净距约 0.25 m，接近车道中间），140 时
        # 约 0.40 m（净距约 0.20 m）。side_margin 不能大于这里对应的净距，
        # 否则防压线保护会一直拦住左转，弯道处黄线容易移出画面。
        self.target_x = float(rospy.get_param("~target_x", 115.0))
        # 【调试】比例增益 (rad/s per px)。转向迟钝就调大，抖动/画龙就调小。
        self.kp = float(rospy.get_param("~kp", 0.007))
        # 【调试】微分增益 (rad/s per px/帧)，抑制误差快速变化时的转向超调。
        # 默认 0 即纯 P 控制；过弯出现"先冲出去再往回纠"时可从 0.001~0.003
        # 试起，太大会放大像素检测噪声、引起高频抖动。
        self.kd = float(rospy.get_param("~kd", 0.0))
        # 【调试】角速度指令限幅 (rad/s)，所有分支的输出都按 ±该值限幅。
        self.max_ang_vel = float(rospy.get_param("~max_ang_vel", 0.8))
        self.min_ang_vel = -self.max_ang_vel
        # 【调试】检测消息中断超过该时长 (s) 看门狗即发零速，防止检测节点
        # 卡死时底盘保留旧指令继续冲。
        self.message_timeout = float(rospy.get_param("~message_timeout", 0.4))

    def _init_gap_params(self):
        """丢线判断、缺口直行与丢线搜索。"""
        # 【调试】跟踪像素数不大于该值即视为丢线。
        self.min_line_pixels = float(rospy.get_param("~min_line_pixels", 5))
        # 【调试】"前方黄线即将结束"的判断行号：跟踪到的黄线最远端行号不小于
        # 该值、且列坐标不在目标列右侧时，视为快到缺口并直行。正常巡线时
        # 最远端约 241，右转弯中约 280~301，两处缺口前约 300 时黄线开始外弯。
        # 调小会在弯道中误判为缺口而直行；调大则会先跟着末端外弯转一段，
        # 车头偏向外侧，容易压线。
        self.line_end_row = float(rospy.get_param("~line_end_row", 298.0))
        # 【调试】缺口直行最多多少帧（检测约 10 Hz）后改为搜索。两处缺口从
        # 黄线末端外弯到重新看到黄线约 0.8 m，0.19 m/s 下约 43 帧，留余量
        # 取 60。调小会在缺口中途就向右转、冲进内场；调大只影响真正跑出
        # 赛道时多久开始搜索。
        self.gap_hold_frames = int(rospy.get_param("~gap_hold_frames", 60))
        # 【调试】持续丢线后的搜索角速度 (rad/s)，负值为右转（赛道顺时针，
        # 缺口后向右拐）。找线太慢、冲出赛道就调大绝对值；转过头压到另一侧
        # 线就调小绝对值。
        self.turn_ang_vel = float(rospy.get_param("~turn_ang_vel", -0.38))
        # 【调试】搜索方向判断阈值：检测结果 position.y 不大于该值时按
        # turn_ang_vel 转向搜索，否则直行。
        self.side_hint_threshold = float(rospy.get_param("~side_hint_threshold", 340.0))

    def _init_guard_params(self):
        """盲区防压线保护。"""
        # 【调试】左轮外沿与左侧黄线中心至少保留的距离 (m)，含黄线半宽约
        # 0.03 m。target_x=115 配 0.25 时顶部凸起处实测最近约 19 cm。调大
        # 离左线更远，但左转更晚、车被推向右侧线；调小则凸起拐角处更贴线。
        # 不要大于 target_x 对应的直道净距（见 target_x 注释）。
        self.side_margin = float(rospy.get_param("~side_margin", 0.25))
        # 【调试】右轮外沿与右侧黄线（右侧道路线/内侧街区边线）至少保留的
        # 距离 (m)。调大右转更晚、更靠外，太大会被逼向左线；调小右转弯时
        # 更容易切弯压到右侧线。
        self.right_margin = float(rospy.get_param("~right_margin", 0.15))
        # 【调试】沿候选角速度向前预测的时长 (s)。调大更早发现拐角、更早
        # 收住转向，但更保守、黄线更容易移出画面；调小则拐角处容易切进去。
        self.side_horizon = float(rospy.get_param("~side_horizon", 1.5))
        # 【调试】检查车身旁边记忆黄线点的前向范围上限 (m，base_link 前方)。
        # 车身前沿在 0.22 m 处，略向前多看能提前避开拐角；太大会把前方正常
        # 弯道也当成贴线，平白限制转向。
        self.side_check_front = float(rospy.get_param("~side_check_front", 0.45))
        # 【调试】底盘实际能达到的最大转向角速度 (rad/s)，仅用于预测。0.19 m/s
        # 下实测右转约 0.4、左转约 0.25；换车速或底盘时要重新估计。
        self.turn_rate_limit = float(rospy.get_param("~turn_rate_limit", 0.4))
        # 车身中心线到车轮外沿的距离 (m)：URDF 车轮 ±0.154 m、轮宽 0.099 m。
        self.body_half_width = float(rospy.get_param("~body_half_width", 0.20))

        # 车辆几何与预测步长（换车才需要改）。
        self.side_check_rear = -0.30   # 检查范围下限 (m)，覆盖到车尾
        self.rear_axle_offset = 0.22   # 后轴中心在 base_link 后方的距离 (m)
        self.side_dt = 0.1
        self.side_steps = max(1, int(round(self.side_horizon / self.side_dt)))
        # 候选角速度的搜索步长 (rad/s)。
        self.guard_step = 0.05

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def right_callback(self, msg):
        if self.odom_pose is None:
            return
        with self.right_lock:
            for p in msg.points:
                self._remember(self.right_memory, p.x, p.y)

    def pose_callback(self, msg):
        self.last_message_time = rospy.get_time()
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        far_row = msg.orientation.x

        if not self.found_line:
            if x < 0:
                self._publish(0.0, 0.0)
                return
            self.found_line = True
            rospy.loginfo("[graduation_control] 首次检测到黄色车道线，开始巡线。")

        line_missing, just_recovered = self._update_line_state(x, z, far_row)
        lin_vel = self.cruise_speed
        ang_vel = self._base_turn(x, y, line_missing, just_recovered)

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
        now = rospy.get_time()
        if self.last_message_time is None or now - self.last_message_time > self.message_timeout:
            self._publish(0.0, 0.0)

    def clean_shutdown(self):
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # 巡线决策
    # ------------------------------------------------------------------
    def _update_line_state(self, x, z, far_row):
        """判断本帧是否丢线（含快到缺口），更新连续丢线帧数。

        缺口前黄线末端向左外弯，列坐标落在目标列左侧；右转弯时最远端行号
        同样会变大，但列坐标在目标列右侧，不能当成缺口。
        返回 (line_missing, just_recovered)。
        """
        line_ending = far_row >= self.line_end_row and 0 <= x <= self.target_x
        line_missing = z <= self.min_line_pixels or x < 0 or line_ending
        just_recovered = not line_missing and self.prev_line_missing
        self.lost_frame_count = self.lost_frame_count + 1 if line_missing else 0
        self.prev_line_missing = line_missing
        return line_missing, just_recovered

    def _base_turn(self, x, y, line_missing, just_recovered):
        """防压线保护之前的角速度：缺口直行 / 丢线搜索 / PD 控制。"""
        if line_missing:
            if self.lost_frame_count <= self.gap_hold_frames:
                return 0.0
            return self.turn_ang_vel if y <= self.side_hint_threshold else 0.0

        error = self.target_x - x
        # 刚从丢线恢复时 prev_error 是缺口前的旧值，这一帧不用微分项，
        # 以免把缺口期间的位置跳变当成变化率而猛打方向。
        derivative = 0.0 if just_recovered else error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.kd * derivative

    # ------------------------------------------------------------------
    # 盲区防压线保护
    # ------------------------------------------------------------------
    def _remember(self, memory, fwd, left):
        """把 base_link 下的地面点换算到里程计坐标系后存入 memory。"""
        px, py, yaw = self.odom_pose
        c, s = math.cos(yaw), math.sin(yaw)
        memory.append((px + fwd * c - left * s, py + fwd * s + left * c))

    def _limit_turn(self, lin_vel, ang_vel):
        """在期望角速度附近找一个预测轨迹上两侧都不贴线的角速度。

        要求：预测途中左侧净距不小于 side_margin、右侧不小于 right_margin；
        某侧当前已经小于要求时，只要求不再变得更近。候选按离期望值由近到远
        （0, +step, -step, +2step, ...）逐个检查，取第一个满足的；都不满足时
        取总缺口最小的。返回 (角速度, 当前左侧净距, 当前右侧净距)。
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
        req_left = None if now_left is None else min(self.side_margin, now_left)
        req_right = None if now_right is None else min(self.right_margin, now_right)

        ang_vel = max(self.min_ang_vel, min(self.max_ang_vel, ang_vel))
        best_ang, best_short = ang_vel, None
        for k in range(int(4 * self.max_ang_vel / self.guard_step) + 1):
            candidate = ang_vel + self.guard_step * ((k + 1) // 2) * (1 if k % 2 else -1)
            if candidate > self.max_ang_vel + 1e-6 or candidate < self.min_ang_vel - 1e-6:
                continue
            gap_left, gap_right = self._predicted_clearance(left_pts, right_pts, lin_vel, candidate)
            short = self._shortfall(gap_left, req_left) + self._shortfall(gap_right, req_right)
            if short == 0.0:
                return candidate, now_left, now_right
            if best_short is None or short < best_short:
                best_ang, best_short = candidate, short
        return best_ang, now_left, now_right

    def _predicted_clearance(self, left_pts, right_pts, lin_vel, ang_vel):
        """按 (lin_vel, ang_vel) 行驶 side_horizon 秒，返回途中 [左侧, 右侧] 最小净距。

        转向角速度按 turn_rate_limit 截断；阿克曼车身绕后轴转动，按后轴中心积分。
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

    def _clearance(self, points, px, py, yaw, side):
        """位姿 (px, py, yaw) 下车身一侧外沿到记忆黄线的最小距离 (m)。

        side=+1 为左侧，-1 为右侧；车身旁边没有记忆点时返回 None。
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

    @staticmethod
    def _shortfall(gap, required):
        """净距比要求少多少；满足要求或旁边没有黄线时为 0。"""
        if gap is None or required is None:
            return 0.0
        return max(0.0, required - gap)

    def _publish(self, speed, yaw_rate):
        yaw_rate = max(self.min_ang_vel, min(self.max_ang_vel, yaw_rate))
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(yaw_rate)
        self.vel_pub.publish(cmd)


if __name__ == "__main__":
    try:
        rospy.init_node("graduation_control", anonymous=False)
        LaneFollowController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
