#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#--------------------------------------------------------------
# Title: 足回り制御を行うPythonモジュール
# Author: Issei Iida, Yusuke Kanazawa
# Date: 2021/10/25
# Memo: [rotateAngle()]はOdometryを使っています
#--------------------------------------------------------------

import rospy
import time
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class BaseControl():
    def __init__(self):
        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCB)
        # Value
        #self.quaternion = (0.0, 0.0, 0.0, 0.0)   # クォータニオン (x, y, z, w)
        #self.current_euler = []   # オイラー角 [roll: x, Pitch: Y, Yaw: z]
        #self.current_deg = 0.0   # 現在の角度（度数法）
        
    def odomCB(self, receive_msg):
        self.quaternion = (
            receive_msg.pose.pose.orientation.x,
            receive_msg.pose.pose.orientation.y,
            receive_msg.pose.pose.orientation.z,
            receive_msg.pose.pose.orientation.w)
        #print(self.quaternion)
        self.current_euler = tf.transformations.euler_from_quaternion(self.quaternion)
        #print(self.current_euler)
        self.current_deg = math.degrees(self.current_euler[2])
        if self.current_deg < 0.0:
            sub_deg = 180 - abs(self.current_deg)
            self.current_deg = 180 + sub_deg
        else:
            pass
        print(self.current_deg)


if __name__ == '__main__':
    rospy.init_node('output_odom')
    base_control = BaseControl()
    rospy.spin()
