#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def record_pose():
    # 初始化节点
    rospy.init_node('record_contest_path', anonymous=False)
    
    # 创建发布者，将轨迹发布 to /pose 话题供 Rviz 可视化
    pub = rospy.Publisher('/pose', PoseStamped, queue_size=3)
    
    # 创建 TF 监听器
    tf_listener = tf.TransformListener()

    # 等待 TF 树建立
    rospy.sleep(1.0)
    
    # 动态检测坐标系，优先使用 map，如果没有则退而求其次使用 odom
    target_frame = "map"
    try:
        rospy.loginfo("Checking for 'map' to 'base_link' transform...")
        tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.5))
        target_frame = "map"
        rospy.loginfo("Successfully connected to 'map' coordinate system.")
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        try:
            rospy.loginfo("Checking for 'odom' to 'base_link' transform...")
            tf_listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(1.5))
            target_frame = "odom"
            rospy.loginfo("Successfully connected to 'odom' coordinate system.")
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Neither 'map' nor 'odom' transform to 'base_link' is available yet. Defaulting to 'map' and will retry in loop.")

    # 设置循环频率 (20Hz)
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        try:
            # 使用 rospy.Time(0) 获取最新的可用变换
            now = rospy.Time(0)
            tf_listener.waitForTransform(target_frame, "base_link", now, rospy.Duration(0.5))
            (trans, rot) = tf_listener.lookupTransform(target_frame, "base_link", now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rate.sleep()
            continue
        
        # 提取坐标信息
        x = trans[0]
        y = trans[1]
        
        # 打印XY坐标到终端
        rospy.loginfo("X: %.4f, Y: %.4f" % (x, y))
        
        # 创建并发布 PoseStamped 消息（用于 Rviz 显示）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = target_frame
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        
        pub.publish(pose_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        record_pose()
    except rospy.ROSInterruptException:
        pass
