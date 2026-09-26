#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯视觉黄色道路线检测节点（只做检测，不发速度指令）。

每帧处理流程（callback）：
  1. 光照自适应分割：按亮度选 Gamma 增亮，HSV 与 LAB 双重阈值得到黄色掩膜。
  2. 滑动窗口跟踪：从掩膜中锁定车辆左侧的目标黄线，收集其像素。
  3. 在目标黄线像素中按参考行定位目标列，并投影到地面。
  4. 找出目标黄线右侧最近的另一条黄线，逐行投影到地面。
  5. 发布检测结果。

发布话题：
  /lane_detect_pose (geometry_msgs/Pose)，字段约定为检测数据容器：
    position.x     目标黄线在参考行上的像素列；未检测到时为 -1
    position.y     画面下方中央 ROI 内黄色像素的纵向中心 (px)，丢线时判断搜索方向
    position.z     本帧跟踪到的目标黄线像素数，用于判断是否完全丢线
    orientation.x  跟踪到的目标黄线最远端所在图像行号（越大说明前方可见黄线
                   越短、快到缺口）；无像素时为 -1
    orientation.y  position.x 对应黄线点在地面上距 base_link 的前方距离 (m)
    orientation.z  position.x 对应黄线点在地面上距 base_link 的左侧距离 (m)
                   （position.x 为 -1 时这两项为 0，不可使用）
  /lane_detect_right_points (geometry_msgs/Polygon)：目标黄线右侧最近的黄线
    （右侧道路线/内侧街区边线）逐行投影到地面的点，x=前方、y=左侧 (m)。
  /lane_detect_image (sensor_msgs/Image, mono8)：黄色掩膜，仅供观察。

修改字段含义时，必须同步修改控制节点 graduation_control.py。

标【调试】的参数与检测范围、跟踪表现有关，注释里说明了改动的效果；
黄色分割（Gamma、HSV/LAB 阈值、形态学核）已调好，无需改动。
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, Polygon, Pose
from sensor_msgs.msg import Image


class AutonomousLaneDetector:
    """自适应分割黄色掩膜，跟踪左侧目标黄线并发布像素级/地面级检测数据。"""

    def __init__(self):
        self._init_segmentation()
        self._init_tracking()
        self._init_outputs()
        self._init_camera_model()

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/lane_detect_image", Image, queue_size=1)
        self.target_pub = rospy.Publisher("/lane_detect_pose", Pose, queue_size=1)
        self.right_pub = rospy.Publisher("/lane_detect_right_points", Polygon, queue_size=1)
        image_topic = rospy.get_param("~image_topic", "/color/image_raw")
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        rospy.loginfo("自适应检测启动成功")

    # ------------------------------------------------------------------
    # 参数与状态
    # ------------------------------------------------------------------
    def _init_segmentation(self):
        """黄色分割参数（已调好，无需改动）。"""
        # 按近场平均亮度由暗到亮分档：(亮度上界, Gamma)；Gamma < 1 时查表增亮。
        self.gamma_table = [
            (10.0, 0.20),
            (20.0, 0.25),
            (35.0, 0.35),
            (55.0, 0.45),
            (80.0, 0.60),
            (110.0, 0.80),
            (float("inf"), 1.00),
        ]
        self.gamma_lut = {
            g: np.array([((i / 255.0) ** g) * 255 for i in range(256)]).astype("uint8")
            for _, g in self.gamma_table
        }
        # HSV 限定黄色色相/饱和度（排除白色标线），LAB 排除草坪与灰色路面。
        self.hsv_lower = np.array([13, 50, 30])
        self.hsv_upper = np.array([35, 255, 255])
        self.lab_lower = np.array([0, 118, 138])
        self.lab_upper = np.array([255, 150, 255])
        # 开运算去小噪点，闭运算连接细小断裂。
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # 【调试】检测区域上边界（图像行号）：只保留该行以下的黄色像素，横向
        # 保留全幅以免弯道黄线被截断。调大可排除远处干扰物，但弯道远端的
        # 黄线也会被裁掉；调小则看得更远、更容易引入远处的黄色干扰。
        self.roi_top_row = 200

    def _init_tracking(self):
        """滑动窗口跟踪目标黄线的参数与状态。"""
        # 【调试】纵向窗口数。调大跟踪分段更细、更贴合弯道形状，计算量增加；
        # 调小则弯道细节容易被平均掉。
        self.nwindows = 9
        # 【调试】窗口水平半宽 (px)。调大能跟住更急的弯，但更容易把旁边的
        # 黄色干扰物（如建筑黄框）纳入窗口；调小则急弯处容易跟丢。
        self.window_margin = 65
        # 【调试】窗口内像素数超过该值才用均值重新定位窗口中心。调大能过滤
        # 稀疏噪点，但黄线较细/较暗时窗口更容易续接不上。
        self.min_recenter_pix = 15
        # 【调试】跟踪窗口的纵向范围（图像行号）。track_y_top 调小看得更远，
        # 但远处黄线更细、更容易受噪声干扰；track_y_bottom 不要超过 480。
        # 画面约第 320 行以下被车身遮挡，实际有效的是上半段。
        self.track_y_top = 240
        self.track_y_bottom = 475
        self.window_height = int((self.track_y_bottom - self.track_y_top) / self.nwindows)
        # 列统计从第 10 列开始，忽略画面最左缘。
        self.hist_x_min = 10
        # 【调试】候选列的最低支持像素数 = max(support_min, support_ratio × 最高列)。
        # 调大能过滤更多孤立噪点，但像素稀疏的远处黄线也更容易找不到候选。
        self.support_min = 6.0
        self.support_ratio = 0.05
        # 【调试】锁线后候选列允许偏离预测位置的最大距离 (px)。调小能更快
        # 排除跳到其他黄色物体的候选；太小则急弯时会把正确黄线也排除而丢线。
        self.lock_radius = 130
        # 锁线后在锁线半径内选"离预测位置近、支持像素多"的候选，此为像素数的权重。
        self.lock_support_weight = 0.05
        # 【调试】未锁线时，候选权重随离预测位置距离衰减的尺度 (px)。调小
        # 更倾向选靠近预测位置的候选（更稳），但预测不准时更难找到正确目标。
        self.free_search_decay = 35.0

        # 【调试】本帧跟踪像素少于该值即视为"跟踪失败"（完全丢线的判断在
        # 控制节点的 min_line_pixels）。调大更容易判为失败、更快放弃锁线；
        # 调小则更倾向坚持当前锁线位置。
        self.min_track_pixels = 25
        # 【调试】连续跟踪失败超过多少帧后放弃锁线、重新自由搜索。应不小于
        # 控制节点的 gap_hold_frames：过缺口时左侧黄线延续段还没进入窗口，
        # 画面右侧却能看到右侧道路线，保持锁线才能排除它。调得太小会在
        # 缺口里改锁右侧线，小车随即向右冲进内场；调大则误锁错误目标后
        # 需要更久才能摆脱。
        self.max_lost_tolerance = 60
        # 【调试】用最靠近画面底部多少行 (px) 内像素的中位数更新历史列。
        # 调大更平滑抗噪、急弯响应变慢；调小响应快但易被噪点带偏。
        self.track_update_rows = 12
        # 跟踪列速度估计：用"上一帧位置 + 速度"预测本帧黄线位置，急弯时
        # 跟得上，缺口附近也不容易误锁位置对不上的无关黄色物体。
        # 【调试】速度平滑系数 (0~1)：越大转弯响应越快但易受噪声带偏；
        # 越小越平滑但转弯时预测滞后。
        self.track_velocity_smoothing = 0.5
        # 【调试】单帧预测速度上限 (px/帧)，防止噪声帧让预测点跑出画面；
        # 急弯仍跟不上时可适当调大。
        self.max_track_velocity = 60.0

        # 【调试】初始锁线列：建议与控制节点的 target_x 一致，否则刚启动时
        # 可能先锁到别的黄线或有短暂的转向偏差。
        self.last_valid_x_px = 115.0
        # 内部状态（不需要改）。
        self.last_valid_x_velocity = 0.0
        self.track_confirmed = False
        self.lost_frame_count = 0

    def _init_outputs(self):
        """定位目标列、右侧黄线与丢线搜索方向所用的参数。"""
        # 【调试】定位目标列的参考行：从近到远、上下交替依次尝试，某行缺线
        # 就用下一行，从而容忍虚线间隙。行号需落在跟踪窗口范围内。末尾的
        # 270/260/250 是远处兜底行：黄线向左拐、车为了不切弯晚转时，黄线
        # 先从近处几行的画面左侧移出，用远处行继续转向，而不是被当成缺口
        # 直行；缺口处远端没有黄线，不受影响。增加远处行会更早开始转向，
        # 删掉远处行则左弯时更容易被当成缺口。
        self.target_rows = [320, 310, 330, 300, 340, 350, 290, 360, 280, 370, 270, 260, 250]
        # 【调试】参考行的容差半宽 (px)。调大更容易在该行命中黄线，但目标
        # 列更不精确；调小则相反。右侧黄线取样也用这个容差。
        self.row_band = 6

        # 【调试】右侧黄线取样行：行越多采样越密，行号越小看得越远；
        # 第 320 行以下被车身遮挡，行号不要超过 320。
        self.right_rows = list(range(250, 321, 6))
        # 【调试】离目标黄线至少多少像素才算右侧黄线。调小可能把目标黄线
        # 边缘误当右侧线；调大则两线靠得近时会漏掉右侧线。
        self.right_gap_px = 60

        # 【调试】丢线时判断搜索方向用的画面下方中央 ROI：(y0, y1, x0, x1)。
        # 丢线后转向方向经常判断反了时，可上下/左右移动这个框。
        self.side_hint_roi = (360, 460, 270, 320)

    def _init_camera_model(self):
        """相机内参取自 /color/camera_info，安装位姿取自 limo URDF。

        位于 base_link 前方 0.1848 m、下俯 0.2618 rad，离地 0.385 m（深度图实测）。
        更换相机或安装位置时必须同步修改。
        """
        self.cam_f = 381.36246688113556
        self.cam_cx = 320.5
        self.cam_cy = 240.5
        self.cam_x = 0.1848
        self.cam_height = 0.385
        self.cam_pitch = 0.2618
        self.cam_cos_pitch = np.cos(self.cam_pitch)
        self.cam_sin_pitch = np.sin(self.cam_pitch)

    # ------------------------------------------------------------------
    # 每帧处理
    # ------------------------------------------------------------------
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换错误: %s", str(e))
            return

        mask = self.extract_features(cv_image)
        inlier_x, inlier_y = self.sliding_window_tracking(mask)
        num_pts = len(inlier_x)
        self.update_track_state(inlier_x, inlier_y, num_pts)

        center_x, center_row = self.locate_target_column(inlier_x, inlier_y)
        if center_x >= 0:
            ground_fwd, ground_left = self.pixel_to_ground(center_x, center_row)
        else:
            ground_fwd, ground_left = 0.0, 0.0

        pose = Pose()
        pose.position.x = center_x
        pose.position.y = self.compute_side_hint(mask)
        pose.position.z = float(num_pts)
        pose.orientation.x = float(np.min(inlier_y)) if num_pts > 0 else -1.0
        pose.orientation.y = ground_fwd
        pose.orientation.z = ground_left
        self.target_pub.publish(pose)
        self.right_pub.publish(self.find_right_points(mask, inlier_x, inlier_y))

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except CvBridgeError:
            pass

    def extract_features(self, cv_image):
        """光照自适应黄色分割，返回单通道 0/255 掩膜（尚未区分哪条是目标黄线）。"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        mean_l = float(np.mean(gray[280:480, :400]))
        gamma = next(g for upper, g in self.gamma_table if mean_l < upper)
        boosted = cv2.LUT(cv_image, self.gamma_lut[gamma]) if gamma < 1.0 else cv_image

        hsv = cv2.cvtColor(boosted, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(boosted, cv2.COLOR_BGR2LAB)
        mask = cv2.bitwise_and(cv2.inRange(hsv, self.hsv_lower, self.hsv_upper),
                               cv2.inRange(lab, self.lab_lower, self.lab_upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
        mask[:self.roi_top_row] = 0
        return mask

    def sliding_window_tracking(self, mask):
        """从黄色掩膜中选出目标黄线，用自底向上的窗口收集其像素。

        先在跟踪范围内做列统计得到候选起始列：未锁线时按"支持像素多且离
        预测位置近"选择；锁线后只在预测位置 lock_radius 内选择。再从起始列
        向上逐窗口续接。返回目标黄线像素的 (x, y) 数组，找不到时为空。
        """
        empty = (np.array([], dtype=int), np.array([], dtype=int))
        y_top, y_bottom = self.track_y_top, self.track_y_bottom
        x_min, x_max = self.hist_x_min, mask.shape[1]
        nonzeroy, nonzerox = mask.nonzero()

        hist = np.count_nonzero(mask[y_top:y_bottom, x_min:x_max], axis=0).astype(float)
        if not np.any(hist):
            return empty
        support_floor = max(self.support_min, self.support_ratio * float(np.max(hist)))
        candidates = np.flatnonzero(hist >= support_floor) + x_min
        if len(candidates) == 0:
            return empty

        prior_x = float(self.last_valid_x_px + self.last_valid_x_velocity)
        if self.track_confirmed:
            candidates = candidates[np.abs(candidates - prior_x) <= self.lock_radius]
            if len(candidates) == 0:
                return empty
            score = (np.abs(candidates - prior_x)
                     - self.lock_support_weight * hist[candidates - x_min])
            current_x = int(candidates[np.argmin(score)])
        else:
            weights = (hist[candidates - x_min]
                       / (1.0 + np.abs(candidates - prior_x) / self.free_search_decay))
            current_x = int(candidates[np.argmax(weights)])

        lane_inds = []
        for window in range(self.nwindows):
            win_y_low = y_bottom - (window + 1) * self.window_height
            win_y_high = y_bottom - window * self.window_height
            in_rows = (nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
            win_x_low = max(0, int(current_x - self.window_margin))
            win_x_high = min(x_max, int(current_x + self.window_margin))
            good_inds = (in_rows & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)

            if len(good_inds) > self.min_recenter_pix:
                current_x = int(np.mean(nonzerox[good_inds]))
            else:
                # 窗口偏空时，在同一行带内找当前位置附近的像素续接。
                slice_x = nonzerox[in_rows & (nonzerox >= x_min)]
                if len(slice_x) > self.min_recenter_pix:
                    nearby_x = slice_x[np.abs(slice_x - current_x) <= self.window_margin]
                    if len(nearby_x) > self.min_recenter_pix:
                        current_x = int(np.median(nearby_x))

        lane_inds = np.concatenate(lane_inds)
        return nonzerox[lane_inds], nonzeroy[lane_inds]

    def update_track_state(self, inlier_x, inlier_y, num_pts):
        """维护锁线状态、历史列与列速度估计，供下一帧预测起始位置。"""
        if num_pts < self.min_track_pixels:
            self.lost_frame_count += 1
            if self.lost_frame_count > self.max_lost_tolerance:
                self.track_confirmed = False
            # 没有新位置时速度估计减半，避免按丢线前的速度一直外推。
            self.last_valid_x_velocity *= 0.5
            return

        self.lost_frame_count = 0
        self.track_confirmed = True
        near_x = inlier_x[inlier_y >= np.max(inlier_y) - self.track_update_rows]
        if len(near_x) > 0:
            new_x = float(np.median(near_x))
            raw_velocity = new_x - self.last_valid_x_px
            raw_velocity = max(-self.max_track_velocity,
                               min(self.max_track_velocity, raw_velocity))
            self.last_valid_x_velocity = (
                self.track_velocity_smoothing * raw_velocity
                + (1.0 - self.track_velocity_smoothing) * self.last_valid_x_velocity
            )
            self.last_valid_x_px = new_x

    def locate_target_column(self, inlier_x, inlier_y):
        """按 target_rows 顺序找第一行有目标黄线像素的行，返回 (中位数列, 行号)。

        全部缺失时返回 (-1, -1)，表示当前完全丢线。
        """
        for row in self.target_rows:
            band = np.abs(inlier_y - row) <= self.row_band
            if np.any(band):
                return float(np.median(inlier_x[band])), float(row)
        return -1.0, -1.0

    def pixel_to_ground(self, u, v):
        """把图像像素 (u, v) 投影到地面，返回 base_link 下的 (前方距离, 左侧距离) (m)。"""
        ray_y = -(u - self.cam_cx) / self.cam_f
        ray_z = -(v - self.cam_cy) / self.cam_f
        fwd = self.cam_cos_pitch + ray_z * self.cam_sin_pitch
        down = self.cam_sin_pitch - ray_z * self.cam_cos_pitch
        scale = self.cam_height / down
        return self.cam_x + scale * fwd, scale * ray_y

    def compute_side_hint(self, mask):
        """画面下方中央 ROI 内黄色像素的纵向中心 (px)，供丢线时判断搜索方向。

        数值越小说明该区域越缺少黄色像素（没有像素时返回 y0-20），控制节点
        会转向找线；数值较大说明车身正对黄线，直行即可。
        """
        y0, y1, x0, x1 = self.side_hint_roi
        ys = np.flatnonzero(np.any(mask[y0:y1, x0:x1] == 255, axis=1))
        if len(ys) == 0:
            return float(y0 - 20)
        return float((ys.min() + ys.max()) / 2.0 + y0)

    def find_right_points(self, mask, inlier_x, inlier_y):
        """目标黄线右侧最近的黄线逐行投影到地面，供控制节点做右侧防压线保护。

        只在目标黄线也出现的行里找：该行目标黄线列右侧 right_gap_px 以外、
        离它最近的黄色像素。弯道处目标黄线会在同一行右侧再出现一次，所以
        与目标黄线连通的像素一律排除。
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


if __name__ == "__main__":
    try:
        rospy.init_node("graduation_lane_detector", anonymous=False)
        AutonomousLaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("退出 graduation_lane_detector 节点。")
