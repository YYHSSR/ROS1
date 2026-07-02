#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

class follow_lane:
    def __init__(self):
        #订阅道路线位置
        self.Pose_sub = rospy.Subscriber("/lane_detect_pose", Pose, self.velctory)
        #发布速度指令
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        # 注册节点关闭时的回调函数，确保退出时车辆停止
        self.found_line = False
        rospy.on_shutdown(self.clean_shutdown)


    def clean_shutdown(self):
        rospy.loginfo("Shutdown hook triggered. Stopping the robot...")
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

    def velctory(self,Pose):        
        x = Pose.position.x
        y = Pose.position.y
        z = Pose.position.z

        # 如果尚未首次检测到车道线，检查是否现在检测到了
        if not self.found_line:
            if x >= 0:
                self.found_line = True
                rospy.loginfo("Yellow lane line successfully detected for the first time. Starting follow control!")
            else:
                # 尚未检测到线，保持静止，避免启动瞬间因为没有画面而产生错误转向
                vel = Twist()
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                return

        vel = Twist()
        #设置速度范围
        max_ang_vel = 0.8
        min_ang_vel = -0.8
        #当检测不到道路线时，停止
        if z <= 5 :
            lin_vel = 0
            ang_vel = 0
            vel.linear.x  = lin_vel
            vel.angular.z = ang_vel
            self.vel_pub.publish(vel)
            time.sleep(1)
            rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))
            return
        else  : 
            # 如果黄色车道线丢失 (x < 0)
            if x < 0:
                # 检查横向车道线以决定是否执行右转弯寻找车道
                if y <= 340:
                    lin_vel = 0.19
                    ang_vel = -1.2
                else:
                    lin_vel = 0.19
                    ang_vel = 0.0
            else:
                # 正常检测到黄色线，进行比例控制以保持在车道中间（目标列为 60）
                target_x = 135
                lin_vel = 0.19
                error = target_x - x
                ang_vel = error * 0.007
                
            # 设定转向速度范围
        if ang_vel >= max_ang_vel:
            ang_vel = max_ang_vel
        if ang_vel <= min_ang_vel:
            ang_vel = min_ang_vel
        #发布速度指令
        vel.linear.x  = lin_vel
        vel.angular.z = ang_vel
        self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("follow_lane", anonymous=False)
        rospy.loginfo("Starting follow lane")
        follow_lane()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down follow_object node.")
        cv2.destroyAllWindows()











