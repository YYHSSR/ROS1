#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2021 PS-Micro, Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

BEGIN_DISTANCE = 1
OVER_DISTANCE = 0.5


class MoveToTarget(object):
    def __init__(self):
        rospy.init_node('move_to_target_ar', anonymous=True)
        self.pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.poseCallback, queue_size=10)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        print("Wait for Exploring ...")

    def poseCallback(self, msg):
        for marker in msg.markers:
            qr_x = marker.pose.pose.position.x
            qr_y = marker.pose.pose.position.y
            qr_z = marker.pose.pose.position.z
            rospy.loginfo("Target{} Pose: x:{}, y:{}, z:{}".format(
                marker.id, qr_x, qr_y, qr_z))

            # 默认只追踪 ID 为 1 或 255 的二维码，你也可以根据需要修改此处的 ID
            if marker.id in [1, 255]:
                vel = Twist()
                # 限制速度在0.1到0.2 m/s之间，前方的倍率调整为0.3使得减速更平缓
                vel.linear.x = max(0.1, min(0.2, qr_x * 0.3))  
                # 提高角速度
                vel.angular.z = max(-1.0, min(1.0, qr_y * 2))  # 限制角速度在-1.0到1.0 rad/s之间
                self.vel_pub.publish(vel)
                rospy.loginfo("Publish velocity command[{} m/s, {} rad/s]".format(vel.linear.x, vel.angular.z))


if __name__ == '__main__':
    try:
        MoveToTarget()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploring logistics finished.")
