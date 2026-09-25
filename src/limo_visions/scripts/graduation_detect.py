#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""纯视觉黄色道路线检测节点，不向底盘发送速度指令。

处理链：光照自适应分割 -> 在候选黄色区域中锁定左侧目标黄线（滑动窗口跟踪）
-> 在跟踪像素中定位目标列坐标 -> 发布 /lane_detect_pose。

本节点沿用原有效果良好的亮度分级、Gamma、HSV/LAB 阈值和形态学分割方案，
发布像素级目标数据，供 graduation_control.py 采用比例控制方式。

Pose 字段被约定为检测数据容器：
  position.x     目标黄线在参考行附近的像素列（未检测到时为 -1）
  position.y     画面下方中央 ROI 内黄色像素的纵向中心（用于丢线时判断搜索方向）
  position.z     本帧跟踪到的黄线像素总数（用于判断是否完全丢线）
  orientation.x  跟踪到的黄线最远端所在图像行号（用于提前发现前方缺口，无像素时为 -1）
  orientation.y  position.x 对应黄线点在地面上距 base_link 的前方距离 (m)
  orientation.z  position.x 对应黄线点在地面上距 base_link 的左侧距离 (m)

另发布 /lane_detect_right_points（geometry_msgs/Polygon）：目标黄线右侧最近的
黄色线（右侧道路线/内侧街区边线）逐行投影到地面的点，x=前方、y=左侧 (m)。

===========================================================================
【调试速查】黄色自适应分割（Gamma 分档、HSV/LAB 阈值、形态学核）效果已经
调好，不需要再动。下面这些跟"检测范围"和"跟踪逻辑"相关的参数在
__init__ 及对应方法里标了【调试】注释，并说明了改动会带来什么效果，
需要调整搜索范围/跟踪表现时优先看这些：
  - target_rows / row_band                 定位目标列所用的参考行 & 容差
  - side_hint_roi                          丢线搜索方向判断用的 ROI
  - poly_pts（ROI 多边形顶点）              黄色检测的感兴趣区域范围
  - nwindows / window_margin / min_recenter_pix
    track_y_top / track_y_bottom           滑动窗口跟踪的搜索范围与窗口大小
  - last_valid_x_px 初始值 / max_lost_tolerance   锁线跟踪的初始状态与容错
  - track_velocity_smoothing / max_track_velocity  跟踪列的预测速度估计
    （用"上一帧位置 + 速度"预测这一帧车道线大概在哪，急弯/起伏路段更
    跟得上，缺口附近也更不容易误锁到无关的黄色物体）
  - sliding_window_tracking 里的 support_floor / 锁线半径 / 权重衰减系数
  - update_track_state 里的跟踪失败判定阈值
===========================================================================
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon, Pose


class AutonomousLaneDetector:
    """从相机图像中自适应分割并跟踪左侧黄线，输出像素级目标数据。"""

    def __init__(self):
        """读取检测参数、初始化跟踪状态，并连接 ROS 话题。"""
        # /lane_detect_image 用于查看黄色掩膜，/lane_detect_pose 供控制器订阅。
        self.image_pub = rospy.Publisher("/lane_detect_image", Image, queue_size=1)
        self.target_pub = rospy.Publisher("/lane_detect_pose", Pose, queue_size=1)
        # 右侧黄线的地面点（x=前方距离, y=左侧距离, m，base_link 下）。
        self.right_pub = rospy.Publisher("/lane_detect_right_points", Polygon, queue_size=1)

        self.bridge = CvBridge()
        image_topic = rospy.get_param("~image_topic", "/color/image_raw")

        # 【调试】参考行：定位目标列坐标时依次尝试这些行（从近到远、上下
        # 交替），用于应对虚线间隙——某一行缺线就依次尝试邻近行。
        # 效果：增删/调整行号会改变"能容忍多大的虚线间隙""目标列坐标响应
        # 的灵敏度"；行号范围应落在跟踪窗口的搜索区间内（见下面的
        # track_y_top ~ track_y_bottom），否则该行永远取不到跟踪像素。
        # 末尾的 270/260/250 是远处兜底行：黄线向左拐、车为了不切弯晚转时，
        # 黄线会先从近处几行的画面左侧移出，远处几行还能看到它，这时用远处
        # 的列坐标继续转向，而不是被当成缺口直行。缺口处远端没有黄线，不受影响。
        self.target_rows = [320, 310, 330, 300, 340, 350, 290, 360, 280, 370, 270, 260, 250]
        # 【调试】每个参考行的容差半宽（像素）。调大更容易在该行命中黄线，
        # 但取到的像素范围变宽、目标列坐标会更不精确；调小则相反。
        self.row_band = 6

        # 相机模型：内参取自 /color/camera_info，安装位姿取自 limo URDF
        # （depth_camera_joint：base_link 前方 0.1848 m、向下俯仰 0.2618 rad），
        # 离地高度已用深度图实测为 0.385 m。仅用于把黄线像素换算到地面，
        # 更换相机或安装位置时必须同步修改。
        self.cam_f = 381.36246688113556
        self.cam_cx = 320.5
        self.cam_cy = 240.5
        self.cam_x = 0.1848
        self.cam_height = 0.385
        self.cam_pitch = 0.2618

        # 【调试】右侧黄线点：在这些图像行里找目标黄线右侧最近的黄色像素
        # （画面第 320 行以下被车身挡住，行号不要超过 320）。行越多右侧线
        # 采样越密，计算量略增；行号越小看得越远。
        self.right_rows = list(range(250, 321, 6))
        # 【调试】离目标黄线至少多少像素才算右侧黄线，用来排除目标黄线自身
        # 的宽度与弯道处的横向展开。调小可能把目标黄线边缘误当右侧线，
        # 调大则两线靠得近时会漏掉右侧线。
        self.right_gap_px = 60

        # 【调试】丢线时判断"原地转向搜索"还是"直行等待"所用的画面下方
        # 中央 ROI：(y0, y1, x0, x1)。效果：这个框决定了判断搜索方向时看
        # 的是画面哪一块区域；如果丢线后转向的方向经常判断反了/不合理，
        # 可以尝试上下移动 y0/y1（离车更近或更远）或左右移动 x0/x1。
        self.side_hint_roi = (360, 460, 270, 320)  # (y0, y1, x0, x1)

        # 分割方案：开运算去小噪点，闭运算连接细小断裂（无需再调）。
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # 预计算 7 档 Gamma 查找表，避免每帧逐像素做幂运算（无需再调）。
        self.gamma_lut = {}
        for g in [0.20, 0.25, 0.35, 0.45, 0.60, 0.80, 1.00]:
            self.gamma_lut[g] = np.array([((i / 255.0) ** g) * 255 for i in range(256)]).astype('uint8')

        # 亮度分级表：按 mean_l 由暗到亮排列的 (上界, Gamma, 场景标签,无需再调）)。
        self.scene_table = [
            (10.0, 0.20, "极夜深渊微光模式"),
            (20.0, 0.25, "深渊暗光模式"),
            (35.0, 0.35, "极暗微光模式"),
            (55.0, 0.45, "微光黄昏模式"),
            (80.0, 0.60, "局部浓荫模式"),
            (110.0, 0.80, "正常日照模式"),
            (float("inf"), 1.00, "高亮强光模式"),
        ]

        # HSV/LAB 阈值边界预先分配为 np.array，避免每帧重新构造（无需再调）。
        self.hsv_lower = np.array([13, 50, 30])
        self.hsv_upper = np.array([35, 255, 255])
        self.lab_lower = np.array([0, 118, 138])
        self.lab_upper = np.array([255, 150, 255])

        # 【调试】ROI 多边形顶点：定义黄色检测的感兴趣区域范围，只裁掉图像
        # 上部、横向保留全幅（以免弯道黄线被截断）。效果：缩小该区域可以
        # 排除远处噪声或画面边缘的干扰物，但如果收得太小会把弯道处的黄线
        # 也裁掉导致丢线；放大则相反，更容易引入干扰。
        # 图像尺寸固定为 640x480，ROI 掩膜与帧无关，这里只栅格化一次。
        self.roi_mask = np.zeros((480, 640), dtype=np.uint8)
        poly_pts = np.array([
            [0, 480],
            [640, 480],
            [640, 200],
            [0, 200]
        ], dtype=np.int32)
        cv2.fillPoly(self.roi_mask, [poly_pts], 255)

        # 【调试】滑动窗口跟踪的搜索范围与窗口大小：
        # nwindows：纵向窗口数。调大跟踪分段更细、对弯道形状更敏感，但计算
        #   量增加；调小则相反，弯道细节容易被平均掉。
        self.nwindows = 9
        # window_margin：窗口水平半宽 (px)。调大能容纳更急的弯道、不容易在
        #   弯道处跟丢，但也更容易把旁边的黄色干扰物（如建筑黄框）纳入窗口。
        self.window_margin = 65
        # min_recenter_pix：窗口内像素数超过此值才用均值重新定位窗口中心。
        #   调大能过滤掉稀疏噪点，但黄线本身较细/较暗时窗口更容易续接不上。
        self.min_recenter_pix = 15
        # track_y_top / track_y_bottom：跟踪窗口纵向搜索范围的上/下边界
        #   （图像行号）。调小 track_y_top 可以看得更远，但远处的黄线通常
        #   更细小、更容易被噪声干扰；track_y_bottom 一般对应画面底部，
        #   不建议超过图像高度 480。
        self.track_y_top = 240
        self.track_y_bottom = 475
        self.window_height = int((self.track_y_bottom - self.track_y_top) / self.nwindows)

        # 历史轨迹：用于下一帧找线；丢线时不把历史位置冒充为当前观测。
        # 【调试】已跟踪黄线靠近画面底部的像素列初始值；建议与
        # graduation_control.py 的 target_x 保持一致，否则刚启动时可能有
        # 短暂的转向偏差。
        self.last_valid_x_px = 115.0
        self.track_confirmed = False   # 内部状态：首次成功跟踪后才按历史位置锁线，不建议手动改
        self.lost_frame_count = 0      # 内部状态：连续跟踪失败帧数，不建议手动改
        # 【调试】连续跟踪失败超过多少帧后清除锁线状态（重新自由搜索）。
        # 应不小于 graduation_control.py 的 gap_hold_frames（60）：过缺口时
        # 左侧黄线的延续段还没进入跟踪窗口，画面右侧却能看到车道右边线；
        # 锁线状态保持期间只接受上一位置 130px 内的候选，右边线会被排除，
        # 左侧延续段出现后才重新锁上。调小（如原来的 5）会在缺口里改锁
        # 右边线，小车随即向右冲进内场；调大则误锁定错误目标后需要更久
        # 才能摆脱。
        self.max_lost_tolerance = 60

        # 【调试】跟踪列速度估计：不是直接用"上一帧的静态位置"当参考点，
        # 而是加上"最近几帧列坐标的变化速度"来预测"这一帧大概会在哪
        # 出现"，这样急弯/起伏路段车道线快速平移时也能预判、跟得上；同时
        # 弯道中出现的黄线缺口附近如果有别的不相关黄色物体，因为它的位置
        # 通常跟"预测位置"对不上，也更不容易被误锁定。
        self.last_valid_x_velocity = 0.0   # 内部状态：像素列变化速度估计 (px/帧)，不建议手动改
        # 速度平滑系数 (0~1)：越大对最新变化越敏感（转弯响应快，但更容易被
        # 噪声带偏）；越小越平滑（更稳，但转弯时预测会滞后）。
        self.track_velocity_smoothing = 0.5
        # 单帧最大允许的预测速度 (px/帧)：防止个别噪声帧的速度估计失控，
        # 导致预测点跑到画面外。按急弯处相邻帧列坐标变化的实际观测值调整；
        # 如果发现急弯仍然跟不上，可以适当调大。
        self.max_track_velocity = 60.0

        # 每收到一帧图像，callback 完成一次检测与发布。
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        rospy.loginfo("自适应检测启动成功")

    def extract_features(self, cv_image):
        """自适应黄色分割，返回掩膜及照度诊断值。

        输入为当前仿真相机的 640x480 BGR 图像；返回 mask、平均亮度、
        Gamma 档位、场景标签。后面三项仅供观察，不参与控制。
        """
        # 在近场路面取样，按平均灰度选择原有的七档 Gamma。
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        mean_l = float(np.mean(gray[280:480, :400]))

        # 阈值与 Gamma 档位来自已有、表现良好的检测方案；查表替代原 7 级 if-elif。
        gamma, scene_mode = self._select_gamma_scene(mean_l)

        # 暗场查表增亮；高亮场景保持原图。
        if gamma < 1.00:
            cv_boosted = cv2.LUT(cv_image, self.gamma_lut[gamma])
        else:
            cv_boosted = cv_image

        # HSV 限定黄色色相和饱和度，排除大部分白色标线。
        hsv = cv2.cvtColor(cv_boosted, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # LAB 的 A/B 范围进一步排除草坪、灰色路面等干扰。
        lab = cv2.cvtColor(cv_boosted, cv2.COLOR_BGR2LAB)
        mask_lab = cv2.inRange(lab, self.lab_lower, self.lab_upper)

        # 两个色彩空间都判为黄的像素才保留，再去噪并连接细小断裂。
        mask_yellow = cv2.bitwise_and(mask_hsv, mask_lab)
        mask_clean = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, self.kernel_open)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, self.kernel_close)

        # ROI 只裁掉图像上部；横向保留全幅，以免弯道黄线被截断。
        # 图像尺寸固定为 640x480 时直接复用预计算好的 self.roi_mask；
        # 万一输入尺寸有变化，兜底按原逻辑现场栅格化一次，保证正确性。
        if cv_image.shape[:2] == self.roi_mask.shape:
            roi_mask = self.roi_mask
        else:
            roi_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            poly_pts = np.array([
                [0, 480],
                [640, 480],
                [640, 200],
                [0, 200]
            ], dtype=np.int32)
            cv2.fillPoly(roi_mask, [poly_pts], 255)

        # 返回单通道 0/255 掩膜；这里尚未决定哪一条是目标黄线。
        mask_final = cv2.bitwise_and(mask_clean, roi_mask)

        return mask_final, mean_l, gamma, scene_mode

    def _select_gamma_scene(self, mean_l):
        """按平均亮度 mean_l 查表返回 (gamma, scene_mode)。

        表的顺序/阈值/取值与原 7 级 if-elif 完全一致，只是换成更易读、
        易调整的查表形式。
        """
        for upper_bound, gamma, scene_mode in self.scene_table:
            if mean_l < upper_bound:
                return gamma, scene_mode
        # 理论上不可达（表的最后一档上界为 inf），保留兜底以防表被误改。
        return self.scene_table[-1][1], self.scene_table[-1][2]

    def sliding_window_tracking(self, mask):
        """从黄色掩膜选出目标黄线，并用自底向上的窗口收集其像素。

        首次按初始位置与列支持量选线；锁定后优先沿上一帧黄线续接。
        直线近端出现缺口时，全高度列统计仍可找到可见的远端部分。
        """
        # 使用 240~475 行；每个窗口的高度由 nwindows 决定（预计算于 __init__）。
        y_bottom = self.track_y_bottom
        y_top = self.track_y_top
        window_height = self.window_height

        # 只对非零掩膜像素做窗口筛选；nonzero() 已返回独立数组，无需再拷贝。
        nonzeroy, nonzerox = mask.nonzero()

        # 全高度列统计给出起始候选，最低支持量可抑制孤立噪点。
        # 弯道处黄线可能进入画面右半侧，因此始终搜索到当前掩膜的右边界。
        x_max = mask.shape[1]
        hist = np.count_nonzero(mask[y_top:y_bottom, 10:x_max], axis=0).astype(float)
        if not np.any(hist):
            return np.array([], dtype=int), np.array([], dtype=int)
        # 【调试】候选列的最低支持像素数（取 6.0 和最大值 5% 里较大者）。
        # 调大能过滤掉更多孤立噪点被误判为候选列，但如果黄线本身像素稀疏
        # （比如远处、虚线间隙），也更容易因为达不到这个下限而找不到候选。
        support_floor = max(6.0, 0.05 * float(np.max(hist)))
        candidates = np.flatnonzero(hist >= support_floor) + 10
        if len(candidates) == 0:
            return np.array([], dtype=int), np.array([], dtype=int)
        # 用"上一帧位置 + 估计速度"预测这一帧车道线大概会出现的列坐标，
        # 而不是只用静态的上一帧位置。急弯/起伏路段车道线快速平移时，
        # 这个预测点能提前跟上趋势；弯道中出现缺口时，附近若有别的不
        # 相关黄色物体，它的位置通常跟预测点对不上，也就不容易被误选中。
        prior_x = float(self.last_valid_x_px + self.last_valid_x_velocity)
        if self.track_confirmed:
            # 【调试】锁线后允许候选列偏离预测位置的最大像素距离（130px）。
            # 调小能更快排除跳到其他黄色物体（如建筑黄框）的候选，更稳；
            # 但太小会在急弯时把正确的黄线也一并排除掉，反而造成丢线。
            candidates = candidates[np.abs(candidates - prior_x) <= 130]
            if len(candidates) == 0:
                return np.array([], dtype=int), np.array([], dtype=int)
            # 锁线后限定与预测位置的距离，避免跳向建筑黄框等无关黄色物体。
            base_x = int(candidates[np.argmin(np.abs(candidates - prior_x)
                                               - 0.05 * hist[candidates - 10])])
        else:
            # 【调试】未锁线时，候选离预测位置越远权重衰减越快的系数
            # （35.0）。调小衰减更快，初次选线会更倾向选择靠近预测位置的
            # 候选（更稳）；但如果预测位置本身不准，反而更难找到正确目标。
            weights = hist[candidates - 10] / (1.0 + np.abs(candidates - prior_x) / 35.0)
            base_x = int(candidates[np.argmax(weights)])

        current_x = base_x
        lane_inds = []

        # 从图像底部向上移动窗口；只在窗口附近重新定位。
        for window in range(self.nwindows):
            win_y_low = y_bottom - (window + 1) * window_height
            win_y_high = y_bottom - window * window_height
            win_x_low = max(0, int(current_x - self.window_margin))
            win_x_high = min(x_max, int(current_x + self.window_margin))

            # 收集窗口内属于候选黄线的像素。
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)

            # 有足够像素才用均值更新中心，避免单个噪点拉偏。
            if len(good_inds) > self.min_recenter_pix:
                current_x = int(np.mean(nonzerox[good_inds]))
            else:
                # 当前窗口偏空时，只在当前轨迹附近尝试续接。
                slice_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= 10) & (nonzerox <= x_max)).nonzero()[0]
                if len(slice_inds) > self.min_recenter_pix:
                    slice_x = nonzerox[slice_inds]
                    nearby_x = slice_x[np.abs(slice_x - current_x) <= self.window_margin]
                    if len(nearby_x) > self.min_recenter_pix:
                        current_x = int(np.median(nearby_x))

        lane_inds = np.concatenate(lane_inds) if len(lane_inds) > 0 else np.array([], dtype=int)
        inlier_x = nonzerox[lane_inds]
        inlier_y = nonzeroy[lane_inds]

        return inlier_x, inlier_y

    def locate_target_column(self, inlier_x, inlier_y):
        """在已锁定的黄线像素中，按参考行列表找目标列坐标（像素）。

        依次尝试 self.target_rows 中的行，命中即返回该行附近像素的
        中位数列坐标；全部缺失时返回 -1，表示当前完全丢线。
        """
        if len(inlier_x) == 0:
            return -1.0, -1.0
        for row in self.target_rows:
            band = np.abs(inlier_y - row) <= self.row_band
            if np.any(band):
                return float(np.median(inlier_x[band])), float(row)
        return -1.0, -1.0

    def pixel_to_ground(self, u, v):
        """把图像像素 (u, v) 投影到地面，返回 base_link 下的 (前方距离, 左侧距离) (m)。"""
        ray_x = 1.0
        ray_y = -(u - self.cam_cx) / self.cam_f
        ray_z = -(v - self.cam_cy) / self.cam_f
        cos_p = np.cos(self.cam_pitch)
        sin_p = np.sin(self.cam_pitch)
        fwd = ray_x * cos_p + ray_z * sin_p
        down = ray_x * sin_p - ray_z * cos_p
        scale = self.cam_height / down
        return self.cam_x + scale * fwd, scale * ray_y

    def compute_side_hint(self, mask):
        """画面下方中央 ROI 内黄色像素的纵向中心，供丢线时判断搜索方向。

        与 follow_lane_contest.py 的 center_y 语义一致：数值越小代表该
        ROI 内越缺少黄色像素（更靠近画面上方或完全没有），此时应原地
        转向寻找车道线；数值较大代表车身正对车道线，直行即可。
        """
        y0, y1, x0, x1 = self.side_hint_roi
        roi = mask[y0:y1, x0:x1]
        ys = np.flatnonzero(np.any(roi == 255, axis=1))
        if len(ys) == 0:
            return float(y0 - 20)  # 默认低值，触发搜索转向
        return float((ys.min() + ys.max()) / 2.0 + y0)

    def find_right_points(self, mask, inlier_x, inlier_y):
        """找出目标黄线右侧最近的黄色像素（右侧道路线/内侧街区边线），
        逐行投影到地面，供控制节点做右侧防压线保护。

        只在目标黄线也出现的行里找：该行目标黄线列坐标右侧 right_gap_px
        以外、离它最近的黄色像素。弯道处目标黄线会绕过画面、同一行里在
        右侧再出现一次，所以与目标黄线连通的像素一律不算。
        """
        polygon = Polygon()
        if len(inlier_x) == 0:
            return polygon
        _, labels = cv2.connectedComponents(mask, connectivity=8)
        line_labels = np.unique(labels[inlier_y, inlier_x])
        other = (labels > 0) & ~np.isin(labels, line_labels)
        for row in self.right_rows:
            band = np.abs(inlier_y - row) <= self.row_band
            if not np.any(band):
                continue
            line_col = float(np.median(inlier_x[band]))
            cols = np.flatnonzero(other[row, :])
            cols = cols[cols > line_col + self.right_gap_px]
            if len(cols) == 0:
                continue
            fwd, left = self.pixel_to_ground(float(cols[0]), float(row))
            polygon.points.append(Point32(x=fwd, y=left, z=0.0))
        return polygon

    def update_track_state(self, inlier_x, inlier_y, num_pts):
        """维护锁线状态与历史像素列，供下一帧续接跟踪使用。"""
        # 【调试】本帧跟踪像素数低于该值（25）就视为本帧"跟踪失败"（不等于
        # 完全丢线，完全丢线的阈值在 graduation_control.py 的
        # min_line_pixels）。调大会更容易判定为跟踪失败、更快清除锁线状态
        # 重新自由搜索；调小则相反，更倾向于坚持当前锁线位置。
        if num_pts < 25:
            self.lost_frame_count += 1
            if self.lost_frame_count > self.max_lost_tolerance:
                self.track_confirmed = False
            # 本帧没有更新到新位置：把速度估计往 0 衰减（打五折），避免
            # 短暂丢线时还按丢线前的速度继续外推，导致预测点越跑越偏。
            self.last_valid_x_velocity *= 0.5
            return

        self.lost_frame_count = 0
        self.track_confirmed = True
        # 【调试】取最靠近画面底部多少行像素（12 行）的中位数来更新历史列
        # 坐标。调大结果更平滑、抗噪能力更强，但对急弯的响应会变慢；调小
        # 则响应更快，但更容易被单帧噪点带偏。
        nearest_rows = inlier_y >= np.max(inlier_y) - 12
        near_x = inlier_x[nearest_rows]
        if len(near_x) > 0:
            new_x = float(np.median(near_x))
            # 用本帧与上一帧的列坐标差更新"速度"估计（指数平滑 + 限幅），
            # 供下一帧 sliding_window_tracking 预测参考位置使用。
            raw_velocity = new_x - self.last_valid_x_px
            raw_velocity = max(-self.max_track_velocity,
                                min(self.max_track_velocity, raw_velocity))
            self.last_valid_x_velocity = (
                self.track_velocity_smoothing * raw_velocity
                + (1.0 - self.track_velocity_smoothing) * self.last_valid_x_velocity
            )
            self.last_valid_x_px = new_x

    def callback(self, data):
        """处理一帧相机图像，并发布检测结果及黄色掩膜。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换错误: %s", str(e))
            return

        # 1. 分割所有黄色区域（自适应光照，效果保留不变）。
        mask, mean_l, gamma, scene_mode = self.extract_features(cv_image)

        # 2. 从黄色区域中跟踪目标黄线（滑动窗口锁线，避免误锁其他黄色物体）。
        inlier_x, inlier_y = self.sliding_window_tracking(mask)
        num_pts = len(inlier_x)
        self.update_track_state(inlier_x, inlier_y, num_pts)

        # 3. 在跟踪像素中定位目标列坐标；计算丢线时的搜索方向辅助量。
        center_x, center_row = self.locate_target_column(inlier_x, inlier_y)
        side_hint_y = self.compute_side_hint(mask)
        if center_x >= 0:
            ground_fwd, ground_left = self.pixel_to_ground(center_x, center_row)
        else:
            ground_fwd, ground_left = 0.0, 0.0

        # 4. /lane_detect_pose 是检测数据接口：
        #      position.x : 目标黄线参考列坐标 (px)；完全丢线时为 -1
        #      position.y : 下方中央 ROI 黄色像素纵向中心 (px)，丢线搜索方向依据
        #      position.z : 本帧跟踪到的黄线像素数，用于判断是否完全丢线
        #      orientation.x : 跟踪到的黄线最远端所在的图像行号（最小 y）；
        #                      数值越大说明前方可见黄线越短（即将到达缺口），
        #                      没有跟踪像素时为 -1
        #      orientation.y / orientation.z : position.x 对应的黄线点投影到
        #                      地面后在 base_link 下的前方/左侧距离 (m)；
        #                      position.x 为 -1 时两者均为 0，不可使用
        objPose = Pose()
        objPose.position.x = center_x
        objPose.position.y = side_hint_y
        objPose.position.z = float(num_pts)
        objPose.orientation.x = float(np.min(inlier_y)) if num_pts > 0 else -1.0
        objPose.orientation.y = ground_fwd
        objPose.orientation.z = ground_left
        self.target_pub.publish(objPose)
        self.right_pub.publish(self.find_right_points(mask, inlier_x, inlier_y))

        # 5. 发布全部黄色区域的二值掩膜供观察。
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except CvBridgeError:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node("graduation_lane_detector", anonymous=False)
        AutonomousLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("退出 graduation_lane_detector 节点。")
        cv2.destroyAllWindows()
