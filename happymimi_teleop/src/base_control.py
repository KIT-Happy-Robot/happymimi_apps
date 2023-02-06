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
import numpy
import matplotlib.pyplot as plot 
import tf
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
        self.anglar_vel = 0.0  # 角速度
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
        self.anglar_vel = receive_msg.twist.twist.angular.z
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

    def publishAnglarZ(self, max_speed):
        time_list = []
        deg_list = []
        vel_max = max_speed
        kp = 0.16
        ki = 0.13
        kd = 7.0
        print("rotateAngle")
        start_time = time.time()
        while round(self.judg_deg, 1) != round(self.current_deg, 1) and not rospy.is_shutdown():
            real_time = time.time() - start_time
            # 0度をまたがないとき
            if self.sub_target_deg == 0.0:
                self.judg_deg = self.target_deg
                vel_z = kp*(self.target_deg - self.current_deg) + ki*(self.target_deg - self.current_deg)*real_time - kd*self.anglar_vel
            # 0度を時計回りにまたぐとき
            elif self.sub_target_deg > 180:
                self.judg_deg = self.sub_target_deg
                if abs(360 - self.sub_target_deg) < 10.0:
                    vel_max = 0.2
                if self.current_deg < 180:
                    vel_z = -vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*(self.sub_target_deg - self.current_deg)*real_time - kd*self.anglar_vel
            # 0度を反時計回りにまたぐとき
            else:
                self.judg_deg = self.sub_target_deg
                if abs(0 - self.sub_target_deg) < 10.0:
                    vel_max = 0.2
                if self.current_deg > 180:
                    vel_z = vel_max
                else:
                    vel_z = kp*(self.sub_target_deg - self.current_deg) + ki*(self.sub_target_deg - self.current_deg)*real_time - kd*self.anglar_vel
            if abs(vel_z) > vel_max:
                vel_z = numpy.sign(vel_z)*vel_max
            self.twist_value.angular.z = vel_z
            self.twist_pub.publish(self.twist_value)
            # グラフプロット用リスト
            time_list.append(real_time)
            deg_list.append(self.current_deg)
            rospy.sleep(0.1)
        #print(round(self.judg_deg, 1), round(self.current_deg, 1))
        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        print("Finish rotateAngle")
        return time_list, deg_list
        
    def translateDist(self, dist, speed = 0.2):
        try:
            dist = dist.data
        except AttributeError:
            pass
        speed = abs(speed)
        self.target_time = abs(dist / speed)
        self.twist_value.linear.x = dist/abs(dist)*speed
        self.publishLinerX()

    def rotateAngle(self, deg, speed = 0.5):
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
        else:
            self.target_deg = self.current_deg + deg
            if self.target_deg < 0.0:
                self.remain_deg = 360 + self.target_deg
                self.sub_target_deg = self.remain_deg
            else:
                pass
        print("current deg: " + str(self.current_deg))
        print("target deg: " + str(self.target_deg))
        print("sub_target deg: " + str(self.sub_target_deg))
        return self.publishAnglarZ(speed)

    # ゲイン調整のときに使う
    def odomPlot(self, deg, speed = 0.5):
        time_x = []
        deg_y = []
        time_x, deg_y = self.rotateAngle(deg, speed)
        plot.plot(time_x, deg_y)
        plot.show()


if __name__ == '__main__':
    rospy.init_node('pid_base_control')
    base_control = BaseControl()
    #base_control.debag()
    #rospy.spin()
    base_control.odomPlot(-40, 0.5)  # ゲイン調整用
