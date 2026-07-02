#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import numpy as np

class lane_converter:
    def __init__(self):
        # 1. 声明发布者 (Publishers)
        # 发布二值化道路图像
        self.image_pub = rospy.Publisher("/lane_detect_image", Image, queue_size=1)
        # 发布黄色道路线位置
        self.target_pub = rospy.Publisher("/lane_detect_pose", Pose, queue_size=1)
        # 发布红色标志点位置
        self.red_pub = rospy.Publisher("/red_detect_pose", Pose, queue_size=1)
        # 发布红色标志点二值化图像
        self.red_image_pub = rospy.Publisher("/red_detect_image", Image, queue_size=1)
        # 2. 声明工具类与订阅者 (Tools & Subscribers)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/limo/color/image_raw", Image, self.callback)

    def detect_yellow_lane(self, hsv, kernel):
        # 识别黄色车道线，返回车道线的横向、纵向中心坐标，总像素面积，以及二值化Mask
        # 设定黄色道路线的 HSV 阈值范围
        lower_yellow = np.array([4, 60, 50])
        upper_yellow = np.array([30, 200, 160])
        # 二值化及闭运算处理
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # 1. 计算横向道路线位置
        # 选择底部的局部感兴趣区域（ROI）
        color_y = mask[360:460, 270:320]
        white_count_y = np.sum(color_y == 255)
        white_count_sum = np.sum(mask == 255)
        if white_count_y == 0:
            center_y = 240
        else:
            white_index_y = np.where(color_y == 255)
            center_y = (white_index_y[0][white_count_y - 2] + white_index_y[0][0]) / 2
            center_y = center_y + 340
        # 2. 计算纵向道路线位置
        # 多行扫描检测，以应对虚线间隙（主要检测 320 行，如在间隙依次检查邻近行）
        target_rows = [320, 310, 330, 300, 340, 350, 290, 360]
        color_x = None
        for r in target_rows:
            temp_x = mask[r, 0:400]
            if np.sum(temp_x == 255) > 0:
                color_x = temp_x
                break
        if color_x is None:
            center_x = -1
        else:
            white_count_x = np.sum(color_x == 255)
            white_index_x = np.where(color_x == 255)
            center_x = (white_index_x[0][white_count_x - 2] + white_index_x[0][0]) / 2
        return center_x, center_y, white_count_sum, mask

    def detect_red_dot(self, hsv, kernel):
        # 识别马路上的红色圆形标志点，返回其中心坐标和像素面积
        # 设定红色 HSV 双区间阈值（解决色调 Hue 跨越 0/180 分界线的问题）
        lower_red_1 = np.array([0, 70, 50])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 70, 50])
        upper_red_2 = np.array([180, 255, 255])
        mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        # 闭运算平滑图像，去除零星噪点
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        # 提取红色轮廓
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 对外轮廓内部进行填充，将空心圆环补全为实心圆（解决光照反射导致的中心空洞问题）
        filled_mask = np.zeros_like(mask_red)
        cv2.drawContours(filled_mask, contours, -1, 255, -1)
        # 使用填补后的实心图像重新提取轮廓
        contours, _ = cv2.findContours(filled_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_x = -1
        red_y = -1
        red_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            # 过滤过小的零星噪点
            if area > 100:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # 如果有多个红点，优先保存面积最大的最可信区域
                    if area > red_area:
                        red_x = cX
                        red_y = cY
                        red_area = area
        return red_x, red_y, red_area, filled_mask

    def callback(self, data):
        # 1. 转换图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge conversion error: %s", str(e))
            return
        # 2. 转换颜色空间与定义形态学处理核
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        # 3. 运行道路线与红色标志点检测器
        center_x, center_y, white_count_sum, mask = self.detect_yellow_lane(hsv, kernel)
        red_x, red_y, red_area, filled_mask = self.detect_red_dot(hsv, kernel)
        # 4. 发布车道线位置数据
        objPose = Pose()
        objPose.position.x = center_x
        objPose.position.y = center_y
        objPose.position.z = white_count_sum
        self.target_pub.publish(objPose)
        #rospy.loginfo_throttle(0.1, "Yellow Lane - x: %.2f, y: %.2f, sum: %.2f" % (center_x, center_y, white_count_sum))
        # 5. 发布红色点位位置数据
        redPose = Pose()
        redPose.position.x = red_x
        redPose.position.y = red_y
        redPose.position.z = red_area
        self.red_pub.publish(redPose)
        # rospy.loginfo_throttle(0.1, "Red Dot - x: %.2f, y: %.2f, area: %.2f" % (red_x, red_y, red_area))
        # 6. 发布二值化图像供调试监控
        try:
            # 发布黄色车道线二值图
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            # 发布填充后的实心红色圆二值图
            self.red_image_pub.publish(self.bridge.cv2_to_imgmsg(filled_mask, "mono8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge publish error: %s", str(e))

if __name__ == '__main__':
    try:
        rospy.init_node("detect_lane", anonymous=False)
        rospy.loginfo("Starting lane detect node...")
        lane_converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down lane_detect node.")
        cv2.destroyAllWindows()
