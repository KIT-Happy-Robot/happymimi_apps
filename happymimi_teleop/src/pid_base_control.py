#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#--------------------------------------------------------------
# Title: 足回り制御を行うPythonモジュール
# Author: Issei Iida, Yusuke Kanazawa
# Date: 2021/10/25
# Memo: [rotateAngle()]はOdometryを使っています
#       最大180度まで
#--------------------------------------------------------------

import rospy
import time
import math
import tf
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class BaseControl():
    def __init__(self):
        # Publisher
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCB)
        # Value
        self.twist_value = Twist()
        self.target_time = 0.0
        self.rate = rospy.Rate(1.0)
        self.quaternion = (0.0, 0.0, 0.0, 0.0)   # クォータニオン (x, y, z, w)
        self.current_euler = []   # オイラー角 [roll: x, Pitch: Y, Yaw: z]
        self.current_deg = 0.0   # 現在の角度（度数法）
        self.target_deg = 0.0   # 目標の角度（度数法）
        self.remain_deg = 0.0
        self.sub_target_deg = 0.0
        self.judg_deg = 999.0

    def debag(self):
        rospy.Subscriber('/teleop/translate', Float64, self.translateDist)
        rospy.Subscriber('/teleop/rotate', Float64, self.rotateAngle)

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
        #print(self.current_deg)

    def publishLinerX(self):
        print("translateDist")
        start_time = time.time()
        end_time = time.time() + 0.15 # 誤差をカバーする0.15
        while end_time - start_time <= self.target_time:
            #print(end_time - start_time)
            self.twist_pub.publish(self.twist_value)
            end_time = time.time()
            rospy.sleep(0.1)
        self.twist_value.linear.x = 0.0
        self.twist_pub.publish(self.twist_value)
        print("Finish translateDist")

    def publishAnglarZ(self):
        vel_max = 0.3
        ku = 0.06  # 限界感度
        kp = ku/2
        print("rotateAngle")
        while int(self.judg_deg) != int(self.current_deg):
            if self.sub_target_deg == 0.0:
                self.judg_deg = self.target_deg
                vel_z = kp*(self.target_deg - self.current_deg)
            # 0度を時計回りにまたぐとき
            elif self.sub_target_deg > 180:
                self.judg_deg = self.sub_target_deg
                if abs(360 - self.sub_target_deg) < 10.0:
                    vel_max = 0.2
                if self.current_deg < 180:
                    vel_z = -vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg)
            # 0度を反時計回りにまたぐとき
            else:
                self.judg_deg = self.sub_target_deg
                if abs(0 - self.sub_target_deg) < 10.0:
                    vel_max = 0.2
                if self.current_deg > 180:
                    vel_z = vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg)

            if abs(vel_z) > vel_max:
                vel_z = numpy.sign(vel_z)*vel_max
            self.twist_value.angular.z = vel_z
            self.twist_pub.publish(self.twist_value)
            rospy.sleep(0.1)

        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        print("Finish rotateAngle")
        #print("final deg: " + str(self.current_deg))

    def translateDist(self, dist, speed = 0.2):
        try:
            dist = dist.data
        except AttributeError:
            pass
        speed = abs(speed)
        self.target_time = abs(dist / speed)
        self.twist_value.linear.x = dist/abs(dist)*speed
        self.publishLinerX()

    def rotateAngle(self, deg, speed = 0.2):
        try:
            deg = deg.data
        except AttributeError:
            pass
        rospy.sleep(0.2)
        if deg >= 0.0:
            self.target_deg = self.current_deg + deg
            if self.target_deg >= 360:
                self.remain_deg = self.target_deg - 360
                self.sub_target_deg = self.remain_deg
            else:
                pass
            self.twist_value.angular.z = speed
        else:
            self.target_deg = self.current_deg + deg
            if self.target_deg < 0.0:
                self.remain_deg = 360 + self.target_deg
                self.sub_target_deg = self.remain_deg
            else:
                pass
            self.twist_value.angular.z = -speed
        print("current deg: " + str(self.current_deg))
        print("target deg: " + str(self.target_deg))
        print("sub_target deg: " + str(self.sub_target_deg))
        self.publishAnglarZ()

if __name__ == '__main__':
    rospy.init_node('pid_base_control')
    base_control = BaseControl()
    base_control.rotateAngle(10)
    #base_control.debag()
    #rospy.spin()
