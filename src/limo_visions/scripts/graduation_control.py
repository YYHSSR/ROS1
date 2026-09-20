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

        # 1. 参数配置 (阿克曼转向几何匹配标定)
        self.target_x = rospy.get_param("~target_x", 140)             # 车道线目标像素中心列 (px)
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.20)     # 巡航线速度 (m/s)
        self.kp = rospy.get_param("~kp", 0.007)                       # 转向比例增益系数
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 0.45)      # 最大左转角速度 (rad/s)
        self.min_ang_vel = rospy.get_param("~min_ang_vel", -0.45)     # 最小右转角速度 (rad/s)
        self.turn_ang_vel = rospy.get_param("~turn_ang_vel", -0.38)   # 阿克曼大半径平滑转弯角速度 (rad/s)

        # 2. 内部控制状态量
        self.found_line = False                                       # 是否已首次捕获车道线
        self.last_ang_vel = 0.0                                       # 上一时刻角速度
        self.corner_confirm_count = 0                                 # 弯道确认帧计数器

        # 3. ROS 通信发布者与订阅者接口
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.pose_sub = rospy.Subscriber('/lane_detect_pose', Pose, self.pose_callback, queue_size=1)

        # 4. 注册优雅退出钩子
        rospy.on_shutdown(self.clean_shutdown)

        rospy.loginfo("[graduation_control] 节点启动成功! 巡航速度: %.2f m/s, 目标像素: %d, 转弯角速度: %.2f rad/s",
                      self.cruise_speed, self.target_x, self.turn_ang_vel)

    def pose_callback(self, msg):
        """
        接收 graduation_detect.py 发布的 /lane_detect_pose:
          position.x: 左侧车道线引导点 X (>=0 有效，-1 表示丢线或虚线间隙)
          position.y: 正前方横向道路线位置 Y (<=350 为直角弯口阻挡线，999 为前方无阻挡直道)
          position.z: 有效内点像素数量 (<=5 为完全丢失)
          orientation.x: 近场车道线横坐标 X_near (用于防压线安全约束)
        """
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        x_near = msg.orientation.x

        # 1. 首次捕获检测
        if not self.found_line:
            if x >= 0:
                self.found_line = True
                rospy.loginfo("[graduation_control] 首次成功识别左侧车道线，进入巡线模式!")
            else:
                self.vel_pub.publish(Twist())
                return

        vel = Twist()

        # 2. 核心巡线与直角拐弯控制逻辑
        if z <= 5:
            # 视野内彻底无车道线: 底盘静止等待，绝不盲目原地打转寻线
            lin_vel = 0.0
            ang_vel = 0.0
            self.corner_confirm_count = 0
        else:
            if x < 0:
                # 左侧道路线暂时离开视窗 (到达直角弯口 或 处于直道虚线断线间隙)
                # 只有当正前方有横向道路线 (y <= 350) 且连续确认达到3帧时，才确认为直角弯口！
                if y <= 350.0:
                    self.corner_confirm_count += 1
                    if self.corner_confirm_count >= 3:
                        # 确认为弯口: 边向前进冲出弯口边平滑转弯，杜绝纯差速原地打转压住右侧拐角线
                        lin_vel = self.cruise_speed
                        ang_vel = self.turn_ang_vel
                    else:
                        lin_vel = self.cruise_speed
                        ang_vel = 0.0
                else:
                    # 前方无横向阻挡线 (y == 999): 属于直道虚线断线，保持笔直巡航，绝不过早转弯！
                    self.corner_confirm_count = 0
                    lin_vel = self.cruise_speed
                    ang_vel = 0.0
            else:
                # 正常检测到左侧车道线 (x >= 0)
                self.corner_confirm_count = 0
                lin_vel = self.cruise_speed
                error = self.target_x - x
                ang_vel = error * self.kp

                # 🛡️ 左右双向防压线安全约束机制:
                # 1. 右侧压线防御: 当车身过于靠近右侧道路线 (左侧车道线在画面过于偏左 x_near < 100px)
                #    强制向左打方向 (ang_vel >= +0.15)，绝对禁止继续向右打方向，彻底消除右边压线！
                if 0 < x_near < 100.0:
                    ang_vel = max(ang_vel, 0.15)

                # 2. 左侧压线防御: 当车身过于靠近左侧黄色道路线 (左侧车道线在画面过于偏右 x_near > 175px)
                #    强制向右打方向 (ang_vel <= -0.15)，绝对禁止向左打方向，杜绝左边压线！
                if x_near > 175.0:
                    ang_vel = min(ang_vel, -0.15)

                # 3. 设定转向速度限幅
                if ang_vel > self.max_ang_vel:
                    ang_vel = self.max_ang_vel
                if ang_vel < self.min_ang_vel:
                    ang_vel = self.min_ang_vel

        # 4. 发布速度指令到 /cmd_vel
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel
        self.last_ang_vel = ang_vel
        self.vel_pub.publish(vel)

        # 5. 定期终端运行状态监控看板
        rospy.loginfo_throttle(1.0,
            "[巡线状态] 车速: %.2f m/s | 角速度: %+5.2f rad/s | 引导列 X: %5.1f (近场: %5.1f) | 横向参考 Y: %5.1f",
            lin_vel, ang_vel, x, x_near, y)

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