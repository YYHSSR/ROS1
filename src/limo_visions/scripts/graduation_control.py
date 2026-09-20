#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
================================================================================
项目名称: LIMO 智能车自动驾驶 - 车道线自主巡线与转弯控制节点
源文件名: graduation_control.py
功能定位: 订阅 /lane_detect_pose 感知数据，重构示例巡线算法，实现稳定循迹与弯道流畅转弯
参考基准: follow_lane_contest.py
================================================================================
"""

import rospy
from geometry_msgs.msg import Twist, Pose


class LaneFollowController:
    def __init__(self):
        rospy.loginfo("[graduation_control] 初始化车道线巡线与转弯控制节点...")

        # 1. 参数配置 (可由 launch 文件传参，缺省时采用经过实测的最佳值)
        self.target_x = rospy.get_param("~target_x", 135)             # 车道线目标像素中心列 (px)
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.19)     # 巡航线速度 (m/s)
        self.kp = rospy.get_param("~kp", 0.007)                       # 转向比例增益系数
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 0.8)       # 最大左转角速度 (rad/s)
        self.min_ang_vel = rospy.get_param("~min_ang_vel", -0.8)      # 最小右转角速度 (rad/s)
        self.turn_ang_vel = rospy.get_param("~turn_ang_vel", -1.0)    # 拐弯盲区辅助转向角速度 (rad/s, 右转为负)

        # 2. 内部控制状态量
        self.found_line = False                                       # 是否已首次捕获车道线
        self.last_ang_vel = 0.0                                       # 上一时刻角速度
        self.was_turning = False                                      # 丢线前是否处于转弯状态
        self.lost_time = None                                         # 开始丢线的时间戳

        # 3. ROS 通信发布者与订阅者接口
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.pose_sub = rospy.Subscriber('/lane_detect_pose', Pose, self.pose_callback, queue_size=1)

        # 4. 注册优雅退出钩子
        rospy.on_shutdown(self.clean_shutdown)

        rospy.loginfo("[graduation_control] 节点启动成功! 巡航速度: %.2f m/s, 目标像素: %d, 比例增益: %.4f",
                      self.cruise_speed, self.target_x, self.kp)

    def pose_callback(self, msg):
        """
        接收 graduation_detect.py 发布的 /lane_detect_pose:
          position.x: 车道线引导点像素 X (>=0 为有效像素，-1 表示丢线)
          position.y: 动态 ROI 右边界宽度
          position.z: 有效内点数量 (>=30 为高置信度，<=5 为彻底无点)
          orientation.x: 物理横向偏差 e_y (米)
          orientation.y: 航向角偏差 e_psi (rad)
          orientation.z: 道路曲率 kappa (1/m)
        """
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # 1. 首次捕获检测: 避免未收到图像前误动作
        if not self.found_line:
            if x >= 0:
                self.found_line = True
                rospy.loginfo("[graduation_control] 首次成功识别车道线，正式进入自主巡线模式!")
            else:
                vel = Twist()
                self.vel_pub.publish(vel)
                return

        vel = Twist()
        now = rospy.Time.now()

        # 2. 核心巡线与拐弯控制逻辑 (重构自示例代码 follow_lane_contest.py)
        if z <= 5:
            # 视野内彻底无车道线或完全丢失
            if self.lost_time is None:
                self.lost_time = now
            loss_duration = (now - self.lost_time).to_sec()

            # 弯道盲区容错: 若之前处于转弯状态，且丢失时间 < 1.5 秒，保持扫弯寻线
            if self.was_turning and loss_duration < 1.5:
                lin_vel = self.cruise_speed
                ang_vel = self.turn_ang_vel
                rospy.loginfo_throttle(0.5, "[graduation_control] 处于弯道盲区 (%.2fs)，保持角速度 %.2f rad/s 扫弯寻线...",
                                       loss_duration, ang_vel)
            else:
                lin_vel = 0.0
                ang_vel = 0.0
                rospy.loginfo_throttle(1.0, "[graduation_control] 暂无有效车道线 (z <= 5)，底盘静止等待...")
        else:
            # 视野内存在车道线 (z > 5)
            self.lost_time = None
            if x < 0:
                # 左侧道路线离开 row 320 视窗 (到达直角弯口!)
                # 根据示例代码 follow_lane_contest.py: 检查横向道路线 y <= 340，执行右转弯主动寻线切弯
                if y <= 340:
                    lin_vel = self.cruise_speed
                    ang_vel = self.turn_ang_vel
                    self.was_turning = True
                    rospy.loginfo_throttle(0.5, "[graduation_control] 到达拐弯处 (x < 0, y=%.1f)，执行右转弯主动切弯...", y)
                else:
                    lin_vel = self.cruise_speed
                    ang_vel = 0.0
            else:
                # 正常检测到左侧黄色道路线，比例巡线
                lin_vel = self.cruise_speed
                error = self.target_x - x
                ang_vel = error * self.kp

                # 记录转弯状态供盲区跟踪
                if abs(error) > 40 or ang_vel < -0.3:
                    self.was_turning = True
                else:
                    self.was_turning = False

                # 设定转向速度限幅
                if ang_vel > self.max_ang_vel:
                    ang_vel = self.max_ang_vel
                if ang_vel < self.turn_ang_vel:
                    ang_vel = self.turn_ang_vel

        # 4. 发布速度指令到 /cmd_vel
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel
        self.last_ang_vel = ang_vel
        self.vel_pub.publish(vel)

        # 5. 定期终端运行状态监控看板
        rospy.loginfo_throttle(1.0,
            "[巡线状态] 车速: %.2f m/s | 角速度: %+5.2f rad/s | 引导列 X: %5.1f px | 偏差: %+5.1f px",
            lin_vel, ang_vel, x, (self.target_x - x) if x >= 0 else 0.0)

    def clean_shutdown(self):
        """
        节点终止时确保发布停止指令
        """
        rospy.loginfo("[graduation_control] 节点正在退出，停止底盘运动...")
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        for _ in range(5):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.02)
        rospy.loginfo("[graduation_control] 底盘已安全刹停。")


if __name__ == '__main__':
    try:
        rospy.init_node("graduation_control", anonymous=False)
        LaneFollowController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[graduation_control] 退出巡线控制节点。")