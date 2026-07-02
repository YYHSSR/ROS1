#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

global out,pose_x,pose_y,front_dist,left_dist,right_dist,red_x,red_y,red_area
out = 2
pose_x = pose_y= 0
front_dist = left_dist = right_dist = 999.0
#red_x：图像横向像素坐标，范围 0 - 640
#red_y：图像纵向像素坐标，范围 0 - 480
#red_area：红色标志点面积，范围 0 - 307200
red_x = red_y = -1
red_area = 0.0

class follow_lane:
    def __init__(self):
        #订阅道路线位置
        self.Pose_sub = rospy.Subscriber("/lane_detect_pose", Pose, self.velctory)
        #订阅雷达数据
        self.Scan_sub = rospy.Subscriber("/limo/scan", LaserScan, self.scan,queue_size=5)
        #订阅里程计坐标 /odom
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=3)
        #订阅红点位置
        self.Red_sub = rospy.Subscriber("/red_detect_pose", Pose, self.red_pose, queue_size=1)
        self.is_stopping = False
        self.is_approaching = False
        self.start_pose_x = 0.0
        self.start_pose_y = 0.0
        self.target_travel_dist = 0.0
        self.red_dot_count = 0
        self.cooldown_until = rospy.Time(0)
        #发布速度指令
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        # 注册节点关闭时的回调函数，确保退出时车辆停止
        self.found_line = False
        rospy.on_shutdown(self.clean_shutdown)

    #节点关闭回调函数
    def clean_shutdown(self):
        rospy.loginfo("Shutdown hook triggered. Stopping the robot...")
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
    
    #获取里程计坐标
    def odom_callback(self, msg):
        global pose_x,pose_y
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        rospy.loginfo_throttle(0.1, "x: %.4f,y:%.4f" % (pose_x,pose_y))

    #获取红点坐标
    def red_pose(self, msg):
        global red_x, red_y, red_area
        red_x = msg.position.x
        red_y = msg.position.y
        red_area = msg.position.z

    #获取雷达数据
    def scan(self,msg):
        global front_dist,left_dist,right_dist
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 999
        ranges[np.isnan(ranges)] = 999
        # 计算每个数据点对应的角度（角度，单位：度）
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles_deg = np.degrees(angles)
        # 过滤掉小于等于 0.07m 的雷达盲区数据（根据 Gazebo 配置，雷达盲区调整为 0.08m）
        valid_mask = (ranges > 0.07) & (ranges < 999)
        # 前方距离 (-15° 到 15°)
        front_idx = np.where((angles_deg >= -15) & (angles_deg <= 15) & valid_mask)[0]
        front_dist = float(np.min(ranges[front_idx])) if len(front_idx) > 0 else 999.0
        # 左侧距离 (45° 到 90°)
        left_idx = np.where((angles_deg >= 45) & (angles_deg <= 90) & valid_mask)[0]
        left_dist = float(np.min(ranges[left_idx])) if len(left_idx) > 0 else 999.0
        # 右侧距离 (-90° 到 -45°)
        right_idx = np.where((angles_deg >= -90) & (angles_deg <= -45) & valid_mask)[0]
        right_dist = float(np.min(ranges[right_idx])) if len(right_idx) > 0 else 999.0
        # 以 20Hz 的频率限流打印雷达数据
        rospy.loginfo_throttle(0.05, "front: %.4f" % (front_dist))

    def run_time(self, lv, av, tim):
        vel = Twist()
        vel.linear.x = lv
        vel.angular.z = av
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < tim and not rospy.is_shutdown():
            self.vel_pub.publish(vel)
            rate.sleep() 
        self.vel_pub.publish(Twist())

    #出库
    def go(self):
        global out
        if out==2 :
            self.run_time(0.25,0,2.9)
            self.run_time(0.12,-0.7,15)
            out = out - 1

    #获取车道线白点数值
    def velctory(self,Pose):        
        x = Pose.position.x
        y = Pose.position.y
        z = Pose.position.z

        global out,pose_x,pose_y,front_dist,left_dist,right_dist,red_x,red_y,red_area
        self.go()

        #定点停车
        # 1. 如果当前正在执行停靠等待，直接发布停止速度并返回（防止多线程回调重入）
        if self.is_stopping:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)
            return
        # 2. 检查是否检测到红点，如果没在冷却期内且没有在逼近，触发检测
        now = rospy.Time.now()
        if now >= self.cooldown_until and not self.is_approaching:
            # 当红圈被识别且到达触发条件时触发
            if red_area > 1500 and red_y >= 300:
                self.red_dot_count += 1
                # 判断当前次数是否执行停车 (1, 3, 5次停车，其他次数跳过)
                if self.red_dot_count in [1, 3, 5]:
                    self.is_approaching = True
                    self.start_pose_x = pose_x
                    self.start_pose_y = pose_y
                    # 动态计算所需的物理行驶距离
                    self.target_travel_dist = 1.3 - 0.0035 * (red_y - 220)
                else:
                    # 不停车的红点，直接设置冷却期，使其在越过该红圈前不重复计数，然后继续正常行驶
                    self.cooldown_until = rospy.Time.now() + rospy.Duration(3.0)
        # 3. 如果在逼近状态下，单独接管速度控制并最终停靠，但转向仍使用本回调传入的 x, y, z 计算出的巡线角速度
        if self.is_approaching:
            # 计算当前已行驶的欧氏距离
            dist_traveled = sqrt((pose_x - self.start_pose_x)**2 + (pose_y - self.start_pose_y)**2)
            dist_error = self.target_travel_dist - dist_traveled
            # 安全冗余：如果车头距离障碍物小于 0.10m，则立即刹车停靠
            if dist_error <= 0.01:
                self.is_stopping = True
                self.is_approaching = False
                self.run_time(0.0, 0.0, 5.5)
                self.is_stopping = False
                self.cooldown_until = rospy.Time.now() + rospy.Duration(3.0)
                return
            else:
                # 比例减速控制，最低速度 0.05m/s，最高速度 0.15m/s
                if dist_error > 0.15:
                    lin_vel = 0.19
                else:
                    lin_vel = np.clip(
                        0.25 * dist_error,
                        0.02,
                        0.19
                    )
            # 逼近期间计算并使用正常的寻线转向控制，不破坏原本的巡线转向计算规则
            if z <= 5:
                ang_vel = 0.0
            else:
                if x < 0:
                    if y <= 340:
                        ang_vel = -1.2
                    else:
                        ang_vel = 0.0
                else:
                    target_x = 140
                    error = target_x - x
                    ang_vel = error * 0.007
            # 设定转向速度范围
            max_ang_vel = 0.8
            min_ang_vel = -0.8
            if ang_vel >= max_ang_vel:
                ang_vel = max_ang_vel
            if ang_vel <= min_ang_vel:
                ang_vel = min_ang_vel
            vel = Twist()
            vel.linear.x = lin_vel
            vel.angular.z = ang_vel
            self.vel_pub.publish(vel)
            return

        #最后一点
        if out == 1 and self.red_dot_count >= 6 and 1.80 <= front_dist <= 1.9 and 3.05<= pose_y <=3.07:
            if rospy.Time.now() >= self.cooldown_until:
                self.is_stopping = True
                self.run_time(0.0, 0.0, 5.5)
                self.is_stopping = False
                self.cooldown_until = rospy.Time.now() + rospy.Duration(3.0)
                out = out - 1
                return

        #入库
        # 第一阶段：达到触发距离，执行第一次转向动作，并将状态 out 置为 -1
        if out == 0 and -1.45 <= pose_x <= -1.40:
            self.run_time(0.19, 0, 0.04)
            self.run_time(0.2, -0.8, 2)
            out = -1
            return
        # 第二阶段：在 out == -1 时正常巡线
        if out == -1 and 3.83 <= front_dist <= 3.88:
            self.run_time(0.19, 0.63, 6.8)
            self.run_time(0.15, 0, 1.2)
            self.run_time(0, 0, 99999)
            return
    
        #巡线逻辑  
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
            rospy.sleep(1)
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
            #打印速度日志
            # rospy.loginfo(
            #             "Publsh velocity command[{} m/s, {} rad/s]".format(
            #                 vel.linear.x, vel.angular.z))

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