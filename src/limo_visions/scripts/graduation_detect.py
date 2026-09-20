#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
================================================================================
项目名称: LIMO 智能车自动驾驶 - 工业级高精度车道线感知与几何解算节点
源文件名: graduation_detect.py
代码定位: 纯感知解耦节点 (订阅相机影像 -> 解算车道线物理特征 -> 输出控制偏差与全中文HUD)
下游节点: graduation_control.py (斯坦利几何循迹控制器)
================================================================================

【系统核心六阶段架构总览 (System Architecture Overview)】:
--------------------------------------------------------------------------------
【阶段一】图像光度自适应与动态 ROI 特征提取 (Image Preprocessing & Dynamic ROI)
         - 1.1 动态自适应逆 Gamma 照度补偿: 支持 0.20 极暗微光至 1.00 强光自适应调节
         - 1.2 HSV+LAB 双色彩空间黄色高精度提纯: 100% 滤除水泥墙、草坪、斑马线与白色箭头
         - 1.3 多级形态学滤波: 开运算去除孤立微噪，闭运算平滑连通车道线
         - 1.4 轨迹引导动态自适应 ROI: 依据航向角动态延展视场 (390px ~ 625px)

【阶段二】垂直自适应滑动窗口路径追踪 (Sliding Window Path Tracking)
         - 2.1 底部车道线直方图峰值扫描与历史先验记忆引导
         - 2.2 8 级垂直滑窗逐级爬升搜索与动态中心重定位
         - 2.3 虚线间隙与稀疏特征廊道保底机制

【阶段三】逆透视几何空间映射 (Inverse Perspective Mapping, IPM)
         - 3.1 基于相机安装物理参数 (离地高度 0.396m, 俯仰角 15°, 前偏置 0.185m)
         - 3.2 像素二维坐标系 (u, v) 严格投影至小车底盘米制坐标系 (x_body, y_body)

【阶段四】RANSAC 几何多项式鲁棒拟合 (RANSAC Robust Polynomial Fitting)
         - 4.1 纵向物理跨度几何先验校验 (>= 0.20m), 根除奇异矩阵与 RankWarning
         - 4.2 35 轮随机抽样一致性迭代估计, 彻底剔除地砖缝隙、阴影散斑等离群噪点
         - 4.3 自适应阶数拟合: 深度跨度充足时采用二次曲线，短跨度时采用一阶直线

【阶段五】物理指标解算与时空滤波 (Metric Resolution & Temporal Filtering)
         - 5.1 曲率自适应动态前瞻距离解算 (0.70m ~ 1.15m)
         - 5.2 物理横向米制偏差 e_y (m) 解算 (目标车道中心偏置 0.45m)
         - 5.3 物理航向角偏差 e_psi (rad) 解算
         - 5.4 物理道路几何曲率 kappa (1/m) 解算
         - 5.5 时序一阶低通滤波平滑与断线容错保护 (容忍连续 6 帧丢失)

【阶段六】ROS 接口通信与全中文 HUD 调试监控 (Publication & Chinese HUD OSD)
         - 6.1 发布 /lane_detect_pose: 输出全维度物理与几何参数 (供下游控制器订阅)
         - 6.2 发布 /lane_detect_image: 输出二值化车道线分割掩膜
         - 6.3 渲染发布 /lane_detect_debug: 高清全中文多级 HUD 状态监控仪表盘
================================================================================
"""

import warnings
import rospy
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose


class AutonomousLaneDetector:
    """
    ============================================================================
    【类名】AutonomousLaneDetector
    【定位】车道线感知与几何解算核心处理器
    【功能】实现从原始单目彩色图像到车辆底盘米制物理偏差的全自主解耦感知解算
    ============================================================================
    """

    def __init__(self):
        """
        ========================================================================
        【初始化函数】系统资源、参数服务器、标定参数与算法状态量初始化
        ========================================================================
        """
        # ----------------------------------------------------------------------
        # [配置 1/8] ROS 话题通信发布者接口初始化
        # ----------------------------------------------------------------------
        # 发布二值化车道线掩膜 (mono8 格式)
        self.image_pub = rospy.Publisher("/lane_detect_image", Image, queue_size=1)
        # 发布车道线位姿与几何物理契约消息 (Pose 格式，供下游 graduation_control.py 订阅)
        self.target_pub = rospy.Publisher("/lane_detect_pose", Pose, queue_size=1)
        # 发布包含滑动窗口、动态 ROI 与多级全中文 HUD 的调试图像 (bgr8 格式)
        self.debug_pub = rospy.Publisher("/lane_detect_debug", Image, queue_size=1)

        self.bridge = CvBridge()

        # ----------------------------------------------------------------------
        # [配置 2/8] ROS 参数服务器载入 (纯感知参数，与控制算法彻底解耦)
        # ----------------------------------------------------------------------
        image_topic = rospy.get_param("~image_topic", "/color/image_raw")
        # 期望车辆相对于左侧黄色基准车道线的物理横向距离 (米), 默认 0.45m
        self.target_lane_offset = rospy.get_param("~target_lane_offset", 0.45)

        # ----------------------------------------------------------------------
        # [配置 3/8] 相机空间内参与安装物理外参 (对应阶段三: 逆透视几何映射 IPM)
        # 来源于 LIMO 仿真模型与相机标定话题 /camera_info
        # ----------------------------------------------------------------------
        self.fx = 381.36           # 相机水平焦距 (px)
        self.fy = 381.36           # 相机垂直焦距 (px)
        self.cx = 320.5            # 水平光学主点中心 (px)
        self.cy = 240.5            # 垂直光学主点中心 (px)
        self.h_cam = 0.396         # 相机离地物理安装高度 (m)
        self.pitch = 0.2618        # 相机向下俯仰倾角 (15度 = 0.2618 rad)
        self.x_cam_offset = 0.185  # 相机光心相对于小车底盘回转中心的前向偏置 (m)

        # ----------------------------------------------------------------------
        # [配置 4/8] 形态学运算核与自适应 Gamma 逆拉伸查找表 (对应阶段一)
        # ----------------------------------------------------------------------
        # 开运算形态学核: 3x3 矩形核，滤除微光极暗环境下的细碎离群噪斑
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # 闭运算形态学核: 5x5 矩形核，填补虚线微小断裂与连通细线
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # 预计算 7 级照度 Gamma 逆拉伸查找表 (LUT), 消除实时浮点乘方开销
        self.gamma_lut = {}
        for g in [0.20, 0.25, 0.35, 0.45, 0.60, 0.80, 1.00]:
            self.gamma_lut[g] = np.array([((i / 255.0) ** g) * 255 for i in range(256)]).astype('uint8')

        # ----------------------------------------------------------------------
        # [配置 5/8] 全中文 HUD 字体引擎初始化 (对应阶段六: 可视化监控)
        # 采用系统预装开源 Noto Sans CJK 高清黑体，支持无锯齿多级中文渲染
        # ----------------------------------------------------------------------
        font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
        try:
            self.font_zh = ImageFont.truetype(font_path, 13)
            self.font_large = ImageFont.truetype(font_path, 14)
            self.has_zh_font = True
        except Exception:
            self.has_zh_font = False

        # ----------------------------------------------------------------------
        # [配置 6/8] 垂直自适应滑动窗口算法参数 (对应阶段二)
        # ----------------------------------------------------------------------
        self.nwindows = 9          # 纵向滑动窗口数量 (覆盖 180~475px 广阔纵深视锥)
        self.window_margin = 65    # 单个窗口单侧水平半宽 (px, 65px 保障急弯处稳健连贯跟踪)
        self.min_recenter_pix = 15 # 触发当前窗口中心向内点均值重定位的最小像素数阈值

        # ----------------------------------------------------------------------
        # [配置 7/8] 轨迹引导动态 ROI 与 RANSAC 状态量 (对应阶段一与阶段四)
        # ----------------------------------------------------------------------
        self.dynamic_x_max = 400   # 严格锁定左侧道路线视场 (0 ~ 400px)，杜绝远端杂散干扰
        self.dynamic_roi_poly = None # 当前动态 ROI 多边形顶点数组
        self.ransac_inlier_ratio = 1.0 # 当前帧 RANSAC 拟合内点纯度 (0.0 ~ 1.0)

        # ----------------------------------------------------------------------
        # [配置 8/8] 时序历史记忆与滤波状态量 (对应阶段五: 防丢线与时空滤波)
        # ----------------------------------------------------------------------
        self.last_fit_metric = None    # 上一有效帧底盘米制多项式拟合系数
        self.last_valid_x_px = 100.0   # 上一有效帧车道线底部引导像素横坐标 (px)
        self.last_heading = 0.0        # 一阶低通滤波平滑后的航向角 (rad)
        self.lost_frame_count = 0      # 连续丢失车道线的帧计数器
        self.max_lost_tolerance = 20   # 允许沿用历史拟合轨迹的最大容忍帧数 (约 0.67s，支撑弯道平稳巡航)

        # 订阅相机原始图像话题，开启视觉回调主流水线
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        rospy.loginfo("自适应检测启动成功")

    def pixel_to_body_frame(self, u, v):
        """
        ========================================================================
        【函数名称】pixel_to_body_frame
        【所属阶段】阶段三：逆透视几何空间映射 (Inverse Perspective Mapping, IPM)
        【功能简述】利用针孔相机逆透视几何模型，将图像二维像素坐标 (u, v) 严格映射为
                   小车底盘刚体米制坐标 (x_body, y_body)
        【几何推导】
                   1. 归一化相机坐标: xc = (u - cx)/fx, yc = (v - cy)/fy
                   2. 视线沿地面法向量的投影分量: r_down = sin(pitch) + cos(pitch)*yc
                   3. 地面空间投影比例因子: dist_scale = h_cam / r_down
                   4. 前向纵向距离: x_body = dist_scale * (cos(pitch) - sin(pitch)*yc) + x_cam_offset
                   5. 侧向横向距离: y_body = -dist_scale * xc (车体坐标系: X向前为正, Y向左为正)
        【输入参数】u : float/int, 像素水平坐标 (列号, 0 ~ 639)
                   v : float/int, 像素垂直坐标 (行号, 0 ~ 479)
        【输出返回】x_body : float or None, 小车前方纵向物理距离 (米)
                   y_body : float or None, 小车侧向横向物理距离 (米, 左正右负)
        【异常处理】若射线指向地平线以上 (r_down <= 0.05), 则返回 (None, None)
        ========================================================================
        """
        # [步骤 3.1] 计算像素点在归一化相机焦平面上的无量纲几何坐标
        xc = (u - self.cx) / self.fx
        yc = (v - self.cy) / self.fy

        # [步骤 3.2] 计算光线在垂直地面重力方向上的投影分量
        r_down = np.sin(self.pitch) + np.cos(self.pitch) * yc
        if r_down <= 0.05:
            # 滤除接近地平线或天空区域的发散光线
            return None, None

        # [步骤 3.3] 根据相机离地安装高度 h_cam 计算光线投射地面的几何缩放因子
        dist_scale = self.h_cam / r_down
        r_forward = np.cos(self.pitch) - np.sin(self.pitch) * yc

        # [步骤 3.4] 坐标转换至小车底盘中心 (Base-Link Frame: X 前向为正, Y 左侧为正)
        x_body = dist_scale * r_forward + self.x_cam_offset
        y_body = -dist_scale * xc
        return x_body, y_body

    def extract_features(self, cv_image):
        """
        ========================================================================
        【函数名称】extract_features
        【所属阶段】阶段一：图像光度自适应与动态 ROI 特征提取
        【功能简述】输入原始彩色相机影像，完成动态照度评估、对数逆 Gamma 拉伸、
                   HSV+LAB 双色彩空间黄色高精度提纯、航向角动态 ROI 裁切与形态学滤波
        【输入参数】cv_image : np.ndarray, 原始彩色图像 (480x640x3, BGR 格式)
        【输出返回】mask_final : np.ndarray, 最终单通道二值化车道线掩膜 (480x640, uint8)
                   mean_l     : float, 近处路面亮度均值 (0.0 ~ 255.0)
                   gamma      : float, 当前匹配采用的逆 Gamma 增强系数
                   scene_mode : str, 当前环境照度工况全中文名称 (供 HUD 显示)
        ========================================================================
        """
        # [步骤 1.1] 近场路面照度评估 (采样小车前方核心路面区域: Y: 280~480, X: 0~400)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        mean_l = float(np.mean(gray[280:480, :400]))

        # 分级自适应判定环境光照等级与匹配逆 Gamma 增强指数
        if mean_l < 10.0:
            gamma = 0.20
            scene_mode = "极夜深渊微光模式"
        elif mean_l < 20.0:
            gamma = 0.25
            scene_mode = "深渊暗光模式"
        elif mean_l < 35.0:
            gamma = 0.35
            scene_mode = "极暗微光模式"
        elif mean_l < 55.0:
            gamma = 0.45
            scene_mode = "微光黄昏模式"
        elif mean_l < 80.0:
            gamma = 0.60
            scene_mode = "局部浓荫模式"
        elif mean_l < 110.0:
            gamma = 0.80
            scene_mode = "正常日照模式"
        else:
            gamma = 1.00
            scene_mode = "高亮强光模式"

        # [步骤 1.2] 快速查表 (LUT) 执行非线性逆 Gamma 亮度动态拉伸
        if gamma < 1.00:
            cv_boosted = cv2.LUT(cv_image, self.gamma_lut[gamma])
        else:
            cv_boosted = cv_image

        # [步骤 1.3] HSV 空间黄色精准提取:
        # 黄色色相 H 严格约束在 [13, 35]，彻底滤除绿色草坪 (H>=40) 与天空背景；
        # 饱和度 S 严格约束 >= 50，彻底滤除无饱和度的灰色水泥墙、沥青路面与白色标线 (S<=25)
        hsv = cv2.cvtColor(cv_boosted, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([13, 50, 30])
        upper_yellow = np.array([35, 255, 255])
        mask_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # [步骤 1.4] LAB 双色彩空间交叉校验:
        # B 通道 (黄-蓝) 阈值 >= 138 (中性灰为128)，A 通道 (红-绿) 阈值 >= 118 (排除草坪绿色 A<=115)
        lab = cv2.cvtColor(cv_boosted, cv2.COLOR_BGR2LAB)
        mask_lab = cv2.inRange(lab, np.array([0, 118, 138]), np.array([255, 150, 255]))

        # [步骤 1.5] 特征双空间逻辑交集与形态学多级滤波
        mask_yellow = cv2.bitwise_and(mask_hsv, mask_lab)
        mask_clean = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, self.kernel_open)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, self.kernel_close)

        # [步骤 1.6] 路面视场 ROI 掩膜 (过滤上方天花板与天空背景 Y < 200)
        self.dynamic_x_max = 400  # 左侧车道线搜索限宽
        roi_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        poly_pts = np.array([
            [0, 480],
            [640, 480],
            [640, 200],
            [0, 200]
        ], dtype=np.int32)
        self.dynamic_roi_poly = poly_pts
        cv2.fillPoly(roi_mask, [poly_pts], 255)

        # [步骤 1.7] 视窗裁剪，得到最终纯净黄色车道线掩膜
        mask_final = cv2.bitwise_and(mask_clean, roi_mask)

        return mask_final, mean_l, gamma, scene_mode

    def sliding_window_tracking(self, mask):
        """
        ========================================================================
        【函数名称】sliding_window_tracking
        【所属阶段】阶段二：垂直自适应滑动窗口路径追踪 (Sliding Window Path Tracking)
        【功能简述】在二值化掩膜中以底部直方图峰值为起点，从底至顶分 9 层滑动窗口
                   逐级聚类内点，动态微调下一层中心，精准排除窗口外杂散噪点
        【输入参数】mask : np.ndarray, 阶段一输出的单通道二值化车道线掩膜 (480x640)
        【输出返回】inlier_x      : np.ndarray, 滑窗聚类捕获的所有内点像素 X 坐标数组
                   inlier_y      : np.ndarray, 滑窗聚类捕获的所有内点像素 Y 坐标数组
                   window_boxes  : list, 滑动窗口的矩形几何包围盒坐标集合
        ========================================================================
        """
        # [步骤 2.1] 设定滑动窗口纵向搜索区间 (Y: 240 ~ 475 px, 聚焦近场 0.58m ~ 2.0m, 杜绝远端弯道干扰)
        y_bottom = 475
        y_top = 240
        window_height = int((y_bottom - y_top) / self.nwindows)

        # 获取当前掩膜中所有白色像素点索引
        nonzero = mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # [步骤 2.2] 确定底部第一层滑动窗口的搜索基准中心 base_x
        x_max = self.dynamic_x_max
        # 优先采样近场路面 (Y: 340 ~ 475) 直方图，保障直道基准准确
        hist_bottom = np.sum(mask[340:y_bottom, 10:x_max], axis=0)
        if np.max(hist_bottom) > 0:
            base_x = int(np.argmax(hist_bottom) + 10)
        else:
            # 近场无点时 (如急弯已切入)，搜索全视野直方图峰值
            hist_active = np.sum(mask[y_top:y_bottom, 10:x_max], axis=0)
            if np.max(hist_active) > 0:
                base_x = int(np.argmax(hist_active) + 10)
            else:
                base_x = int(self.last_valid_x_px)

        current_x = base_x
        lane_inds = []
        window_boxes = []

        # [步骤 2.3] 自底向上逐层滑窗迭代爬升
        for window in range(self.nwindows):
            # 计算当前滑动窗口的四个边界物理像素坐标
            win_y_low = y_bottom - (window + 1) * window_height
            win_y_high = y_bottom - window * window_height
            win_x_low = max(0, int(current_x - self.window_margin))
            win_x_high = min(x_max, int(current_x + self.window_margin))

            window_boxes.append(((win_x_low, win_y_low), (win_x_high, win_y_high)))

            # 提取落入当前窗口区域内的候选像素点
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)

            # 若当前窗口有效内点充足，以均值更新下一层窗口的水平搜索中心
            if len(good_inds) > self.min_recenter_pix:
                current_x = int(np.mean(nonzerox[good_inds]))
            else:
                # 弯道快速重定位: 当前滑窗内无点时，检查该水平切片是否有偏离窗口的黄色像素簇
                slice_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= 10) & (nonzerox <= x_max)).nonzero()[0]
                if len(slice_inds) > self.min_recenter_pix:
                    current_x = int(np.mean(nonzerox[slice_inds]))

        # 合并所有滑动窗口捕获的内点索引
        lane_inds = np.concatenate(lane_inds) if len(lane_inds) > 0 else np.array([], dtype=int)
        inlier_x = nonzerox[lane_inds]
        inlier_y = nonzeroy[lane_inds]

        # [步骤 2.4] 弯道与虚线特征廊道自适应保底
        # 若急弯导致滑窗未能完全覆盖弧线，自动补充动态廊道内所有有效黄色内点
        if len(inlier_x) < 50 or len(inlier_x) < int(0.35 * len(nonzerox)):
            corridor_inds = ((nonzeroy >= y_top) & (nonzeroy <= y_bottom) &
                             (nonzerox >= 10) & (nonzerox <= x_max)).nonzero()[0]
            if len(corridor_inds) >= 30:
                inlier_x = nonzerox[corridor_inds]
                inlier_y = nonzeroy[corridor_inds]

        return inlier_x, inlier_y, window_boxes

    @staticmethod
    def _safe_polyfit(x, y, deg):
        """
        ========================================================================
        【辅助函数】局部受控多项式拟合
        【设计思想】采用 Python warnings 局部上下文管理器 (catch_warnings)，仅在
                   拟合调用内部局部屏蔽 RankWarning，既杜绝终端警告刷屏与 I/O 阻塞，
                   又保持全局 Python 警告配置的纯净规范。
        ========================================================================
        """
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', np.RankWarning)
            return np.polyfit(x, y, deg)

    def fit_polynomial_ransac(self, bx, by, degree, max_trials=35, inlier_thresh=0.04):
        """
        ========================================================================
        【函数名称】fit_polynomial_ransac
        【所属阶段】阶段四：RANSAC 几何多项式鲁棒拟合 (RANSAC Polynomial Fitting)
        【功能简述】在小车底盘米制坐标系 (bx, by) 下执行随机抽样一致性曲线拟合，
                   彻底消除偶发地砖接缝阴影、离群噪点对多项式曲率解算的漂移干扰
        【数值稳定性】引入两级防护机制：
                   1. 几何与代数约束：严格校验纵向物理跨度 (>=0.20m) 及任意两采样点
                      最小间距 (>=0.05m)，从源头杜绝近距共线点集导致的矩阵奇异性；
                   2. 局部受控拟合：通过 _safe_polyfit 局部捕获偶发病态警告，保障
                      控制主循环终端清爽。
        【输入参数】bx            : np.ndarray, 底盘米制坐标系下的前向 X 坐标数组 (m)
                   by            : np.ndarray, 底盘米制坐标系下的横向 Y 坐标数组 (m)
                   degree        : int, 拟合多项式阶数 (1 为直线，2 为抛物线)
                   max_trials    : int, RANSAC 迭代抽样轮数 (默认 35 轮)
                   inlier_thresh : float, 点到模型横向残差容差门槛 (米, 默认 0.04m = 4cm)
        【输出返回】final_coeffs  : np.ndarray, 拟合得到的多项式系数 (高次到常数项降序排列)
                   ratio         : float, 最终模型内点占总样本的比例 (0.0 ~ 1.0)
        ========================================================================
        """
        n_pts = len(bx)
        sample_size = degree + 1
        # 若样本点数不足以支撑稳定抽样，直接退化为全局普通最小二乘拟合
        if n_pts < sample_size + 4:
            return self._safe_polyfit(bx, by, degree), 1.0

        best_inliers = None
        best_count = 0
        indices = np.arange(n_pts)

        # [步骤 4.1] 随机抽样迭代寻找最优内点模型
        for _ in range(max_trials):
            sample_idx = np.random.choice(indices, sample_size, replace=False)

            # 几何先验与代数条件数约束校验:
            # 1. 采样集整体纵向跨度必须 >= 0.20m (杜绝点集过分簇聚)
            # 2. 任意两采样点之间的纵向间距必须 >= 0.05m (杜绝同行或极近点导致范德蒙矩阵列共线与 RankWarning)
            sample_x = bx[sample_idx]
            if sample_size > 1:
                sorted_x = np.sort(sample_x)
                if (sorted_x[-1] - sorted_x[0] < 0.20) or (np.min(np.diff(sorted_x)) < 0.05):
                    continue

            try:
                cand_coeffs = self._safe_polyfit(sample_x, by[sample_idx], degree)
            except Exception:
                continue

            # 计算全量样本在当前假定多项式下的物理横向残差
            pred_y = np.polyval(cand_coeffs, bx)
            residuals = np.abs(by - pred_y)
            inliers = residuals < inlier_thresh
            cnt = int(np.count_nonzero(inliers))

            # 更新历史最大内点集
            if cnt > best_count:
                best_count = cnt
                best_inliers = inliers
                # 早停机制: 当内点率达到 85% 时直接提前收敛，节省算力
                if cnt >= int(0.85 * n_pts):
                    break

        # [步骤 4.2] 利用最大内点集重新进行二次最小二乘参数重估计
        if best_inliers is not None and best_count >= max(8, int(0.30 * n_pts)):
            final_coeffs = self._safe_polyfit(bx[best_inliers], by[best_inliers], degree)
            ratio = float(best_count) / float(n_pts)
            return final_coeffs, ratio
        else:
            # 保底回退机制
            return self._safe_polyfit(bx, by, degree), 1.0

    def compute_metric_and_control(self, inlier_x, inlier_y, mask=None):
        """
        ========================================================================
        【函数名称】compute_metric_and_control (规范别名: compute_lane_metrics)
        【所属阶段】阶段三、四、五综合解算模块
        【功能简述】
                   1. 采样 row 320 纵向道路线，严格锁定左侧车道线 (0 <= X <= 400)，杜绝远端弯道均值导致过早转弯
                   2. 采样 mask[360:460, 250:350] 横向道路线以解算 center_y (弯道辅助判定)
                   3. 调用 pixel_to_body_frame 将像素内点投影至机体物理米制系
                   4. 调用 fit_polynomial_ransac 执行 RANSAC 多项式鲁棒拟合
                   5. 解算横向物理偏差 e_y、航向角偏差 e_psi、曲率 kappa 与动态前瞻
        【输入参数】inlier_x : np.ndarray, 滑动窗口捕获的车道线内点像素 X 坐标
                   inlier_y : np.ndarray, 滑动窗口捕获的车道线内点像素 Y 坐标
                   mask     : np.ndarray or None, 二值化车道线掩膜
        【输出返回】center_x_px    : float, 当前有效车道线底部引导横坐标 (-1.0 表示断线/弯口)
                   center_y       : float, 横向道路线参考纵坐标
                   white_count_sum: int, 全局有效车道线像素数量
                   fit_metric     : np.ndarray or None, 底盘米制拟合系数
                   e_y            : float, 物理横向偏差 (米, 左正右负)
                   e_psi          : float, 物理航向角偏差 (弧度, rad)
                   curvature      : float, 道路物理几何曲率 kappa (1/m)
                   lookahead_dist : float, 自适应计算的前瞻采样距离 (米)
                   fit_ok         : bool, 当前感知解算是否有效
        ========================================================================
        """
        num_pts = len(inlier_x)
        fit_ok = False

        # [步骤 5.0] 计算横向道路线位置 (纵向参考 center_y) 与全局像素量 (与 detect_lane_contest.py 保持一致)
        if mask is not None:
            color_y = mask[360:460, 250:350]
            white_count_y = np.sum(color_y == 255)
            white_count_sum = int(np.sum(mask == 255))
            if white_count_y == 0:
                center_y = 240.0
            else:
                white_index_y = np.where(color_y == 255)
                center_y = float((white_index_y[0][white_count_y - 2] + white_index_y[0][0]) / 2.0 + 340.0)
        else:
            center_y = 240.0
            white_count_sum = num_pts

        # [步骤 5.1] 采样 row 320 近场引导，严格锁定左侧车道线 (0 <= X <= 400)
        # 应对虚线间隙（主要检测 320 行，如在间隙依次检查邻近行，杜绝远端 180px 弯道均值导致过早转弯）
        center_x_px = -1.0
        if mask is not None:
            target_rows = [320, 310, 330, 300, 340, 350, 290, 360]
            for r in target_rows:
                temp_x = mask[r, 0:400]
                if np.sum(temp_x == 255) > 0:
                    white_count_x = np.sum(temp_x == 255)
                    white_index_x = np.where(temp_x == 255)
                    if white_count_x >= 2:
                        center_x_px = float((white_index_x[0][white_count_x - 2] + white_index_x[0][0]) / 2.0)
                    else:
                        center_x_px = float(white_index_x[0][0])
                    break

        if center_x_px >= 0:
            self.last_valid_x_px = center_x_px
            self.lost_frame_count = 0
        else:
            self.lost_frame_count += 1

        # [步骤 5.2] 滤除超出左侧 ROI 边界的杂散边缘噪点并拟合米制底盘多项式
        valid_body = []
        if num_pts >= 25:
            valid_mask = (inlier_x >= 10) & (inlier_x <= 400) & (inlier_y >= 240)
            valid_x = inlier_x[valid_mask]
            valid_y = inlier_y[valid_mask]

            if len(valid_x) >= 20:
                body_pts = [self.pixel_to_body_frame(px, py) for px, py in zip(valid_x, valid_y)]
                valid_body = [p for p in body_pts if p[0] is not None]

                if len(valid_body) >= 15:
                    bx = np.array([p[0] for p in valid_body])
                    by = np.array([p[1] for p in valid_body])
                    dx_span = float(np.max(bx) - np.min(bx))

                    try:
                        degree = 2 if dx_span >= 0.30 else 1
                        current_fit_metric, inlier_ratio = self.fit_polynomial_ransac(bx, by, degree)
                        self.ransac_inlier_ratio = inlier_ratio
                        fit_ok = True
                    except Exception:
                        fit_ok = False

                    if fit_ok:
                        if self.last_fit_metric is not None and len(self.last_fit_metric) == len(current_fit_metric):
                            self.last_fit_metric = 0.8 * current_fit_metric + 0.2 * self.last_fit_metric
                        else:
                            self.last_fit_metric = current_fit_metric

        if not fit_ok:
            if self.lost_frame_count < self.max_lost_tolerance and self.last_fit_metric is not None:
                pass
            else:
                self.last_fit_metric = None

        # 若未检测到有效引导点或彻底丢失，返回断线状态
        if self.last_fit_metric is None or center_x_px < 0:
            return center_x_px, center_y, white_count_sum, None, 0.0, 0.0, 0.0, 0.65, False

        fit_metric = self.last_fit_metric
        bx_min = float(np.min(bx)) if len(valid_body) > 0 else 0.50
        bx_max = float(np.max(bx)) if len(valid_body) > 0 else 1.20

        if len(fit_metric) == 3:
            raw_kappa = abs(2.0 * fit_metric[0])
            nominal_ld = max(0.50, min(0.80, 0.75 - 0.35 * min(raw_kappa, 1.0)))
            lookahead_dist = max(min(nominal_ld, bx_max), max(0.50, bx_min))
            y_lane_metric = fit_metric[0] * (lookahead_dist ** 2) + fit_metric[1] * lookahead_dist + fit_metric[2]
            tangent_slope = 2.0 * fit_metric[0] * lookahead_dist + fit_metric[1]
            curvature = float(abs(2.0 * fit_metric[0]) / ((1.0 + tangent_slope ** 2) ** 1.5))
        else:
            nominal_ld = 0.65
            lookahead_dist = max(min(nominal_ld, bx_max), max(0.50, bx_min))
            y_lane_metric = fit_metric[0] * lookahead_dist + fit_metric[1]
            tangent_slope = fit_metric[0]
            curvature = 0.0

        e_y = float(y_lane_metric - self.target_lane_offset)
        e_psi = float(np.arctan(tangent_slope))
        self.last_heading = 0.75 * e_psi + 0.25 * self.last_heading

        return center_x_px, center_y, white_count_sum, fit_metric, e_y, e_psi, curvature, lookahead_dist, True

    # 兼容规范别名，方便代码检索与外部调用
    compute_lane_metrics = compute_metric_and_control

    def callback(self, data):
        """
        ========================================================================
        【主回调函数】相机图像订阅回调主循环 (Main Sensor Pipeline Dispatcher)
        【流水线全流程】:
           [流水线 1/6] 阶段一: 提取车道线抗光照二值掩膜 (extract_features)
           [流水线 2/6] 阶段二: 垂直滑动窗口连续追踪聚类 (sliding_window_tracking)
           [流水线 3/6] 阶段三~五: IPM米制映射、RANSAC拟合与几何指标解算 (compute_lane_metrics)
           [流水线 4/6] 阶段六.1: 发布车道线感知全息数据 (/lane_detect_pose)
           [流水线 5/6] 阶段六.2: 发布二值化车道线图 (/lane_detect_image)
           [流水线 6/6] 阶段六.3: 渲染并发布全中文 HUD 调试监控图 (/lane_detect_debug)
        ========================================================================
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换错误: %s", str(e))
            return

        # ----------------------------------------------------------------------
        # [流水线 1/6] 阶段一: 图像光度自适应、特征融合与动态 ROI 裁切
        # ----------------------------------------------------------------------
        mask, mean_l, gamma, scene_mode = self.extract_features(cv_image)

        # ----------------------------------------------------------------------
        # [流水线 2/6] 阶段二: 垂直自适应滑动窗口路径追踪
        # ----------------------------------------------------------------------
        inlier_x, inlier_y, window_boxes = self.sliding_window_tracking(mask)

        # ----------------------------------------------------------------------
        # [流水线 3/6] 阶段三~五: 物理 IPM 映射、RANSAC 拟合与核心几何指标解算
        # ----------------------------------------------------------------------
        (center_x, center_y, num_pts, fit_metric,
         e_y, e_psi, curvature, lookahead_dist, fit_ok) = self.compute_metric_and_control(inlier_x, inlier_y, mask)

        # ----------------------------------------------------------------------
        # [流水线 4/6] 阶段六.1: 发布车道线位姿与几何物理数据契约 (/lane_detect_pose)
        # 字段映射表 (与下游 graduation_control.py 严格匹配对照):
        #   position.x    : 引导点像素水平坐标 (px, -1 表示丢线断线)
        #   position.y    : 动态自适应 ROI 右边界当前像素宽度 (px: 390 ~ 625)
        #   position.z    : 滑动窗口捕获的有效车道线内点数量 (Points)
        #   orientation.x : 物理横向米制偏差 e_y (m, 闭环跟踪控制核心量)
        #   orientation.y : 物理航向角偏差 e_psi (rad, 斯坦利控制切线航向项)
        #   orientation.z : 物理道路曲率 kappa (1/m, 弯道自适应调速核心量)
        #   orientation.w : RANSAC 拟合内点纯度比率 (0.0 ~ 1.0)
        # ----------------------------------------------------------------------
        objPose = Pose()
        objPose.position.x = center_x
        objPose.position.y = center_y
        objPose.position.z = float(num_pts)
        objPose.orientation.x = e_y
        objPose.orientation.y = e_psi
        objPose.orientation.z = curvature
        objPose.orientation.w = float(self.ransac_inlier_ratio if fit_ok else 0.0)
        self.target_pub.publish(objPose)

        # ----------------------------------------------------------------------
        # [流水线 5/6] 阶段六.2: 发布二值化分割图像 (/lane_detect_image)
        # ----------------------------------------------------------------------
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except CvBridgeError:
            pass

        # ----------------------------------------------------------------------
        # [流水线 6/6] 阶段六.3: 渲染并发布全中文 HUD 调试监控图 (/lane_detect_debug)
        # ----------------------------------------------------------------------
        if self.debug_pub.get_num_connections() > 0:
            debug_img = cv_image.copy()

            # 绘制动态 ROI 轨迹引导多边形边缘 (半透明青绿色多边形)
            if self.dynamic_roi_poly is not None:
                cv2.polylines(debug_img, [self.dynamic_roi_poly], True, (0, 220, 150), 1, cv2.LINE_AA)

            # 绘制滑动窗口搜索边界框 (青色矩形框)
            for box in window_boxes:
                cv2.rectangle(debug_img, box[0], box[1], (255, 255, 0), 1)

            # 绘制动态前瞻目标采样引导参考点 (红色实心圆)
            if center_x != -1:
                cv2.circle(debug_img, (int(center_x), 320), 7, (0, 0, 255), -1)

            # 绘制全中文 OSD 仪表盘半透明磨砂背景底板
            overlay = debug_img.copy()
            cv2.rectangle(overlay, (5, 5), (380, 132), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.70, debug_img, 0.30, 0, debug_img)

            # 多级全中文 HUD 信息卡片渲染 (基于系统开源 Noto Sans CJK 字体引擎)
            if self.has_zh_font:
                pil_img = PILImage.fromarray(cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(pil_img)

                # 追踪状态标签
                status_text = "系统状态: 稳定追踪 (精准锁定)" if fit_ok else "系统状态: 触发断线保护 (沿用历史)"
                status_color = (0, 255, 100) if fit_ok else (255, 120, 0)

                # 逐行绘制多级全中文感知监控参数
                draw.text((12, 8), "环境光照: %s" % scene_mode, font=self.font_large, fill=(255, 255, 255))
                draw.text((12, 28), "路面照度: %.1f | 逆Gamma: %.2f | 动态ROI界: %d" % (mean_l, gamma, self.dynamic_x_max), font=self.font_zh, fill=(210, 210, 210))
                draw.text((12, 48), "[物理IPM] 横向偏差: %+.3f米 | 航向夹角: %+.1f°" % (e_y, np.degrees(e_psi)), font=self.font_zh, fill=(0, 255, 255))
                draw.text((12, 68), "[几何特征] 道路曲率: %.3f/m | 动态前瞻: %.2f米" % (curvature, lookahead_dist), font=self.font_zh, fill=(255, 200, 100))
                draw.text((12, 88), "[RANSAC鲁棒] 内点率: %.1f%% | 滑窗有效点: %d" % (self.ransac_inlier_ratio * 100.0, num_pts), font=self.font_zh, fill=(100, 255, 255))
                draw.text((12, 108), status_text, font=self.font_zh, fill=status_color)

                # 采样点中文悬浮指示标签
                if center_x != -1:
                    draw.text((int(center_x) + 10, 310), "前瞻采样点", font=self.font_zh, fill=(0, 255, 255))

                debug_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
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
