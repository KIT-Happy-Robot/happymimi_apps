#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import matplotlib.pyplot as plot
import os
import sys
import roslib
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from find_bag.srv import FindBagSrv, FindBagSrvResponse, GraspBagSrv, GraspBagSrvResponse
from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.srv import ArmControl

teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl


class FindBag():
    def __init__(self):
        # Service
        rospy.Service('/find_bag_server', FindBagSrv, self.srv_FindBag)
        rospy.loginfo("Ready to set /find_bag_server")
        rospy.Service('/grasp_bag_server', GraspBagSrv, self.srv_GraspBag)
        rospy.loginfo("Ready to set /grasp_bag_server")
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        rospy.Subscriber('/cmd_vel', Twist, self.velCB)
        # Manipulation
        self.eef = rospy.Publisher('/servo/endeffector', Bool, queue_size=10)
        self.arm_pose = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.manipulation = rospy.ServiceProxy('/servo/debug_arm', ArmControl)
        # Module
        self.base_control = BaseControl()
        # Value
        self.laser_list = []  # LRFのデータが入るリスト
        self.center_range = []  # 中心から拡張するためのリスト
        self.center_index = 0.0  # 真ん中の要素番号を格納する変数
        self.index_sum = 0.0  # LRFの要素数の合計
        self.step_angle = 0.0  # LRFの1ステップあたりの角度
        self.rotate_value = 0.0  # /cmd_velの値を格納する変数

    def laserCB(self, receive_msg):
        self.laser_list = list(receive_msg.ranges)

    def velCB(self, receive_msg):
        self.rotate_value = receive_msg.angular.z

    def roundHalfUp(self, value):
        decimals = math.modf(value)[0]
        return int(value + 1) if decimals >= 0.5 else int(value)

    def laserIndex(self):
        self.index_sum = len(self.laser_list)
        self.step_angle = 180/self.index_sum
        self.center_index = self.roundHalfUp(self.index_sum/2 - 1)

    def average(self, input_list):
        ave = sum(input_list)/len(input_list)
        return ave

    def laserCheck(self):
        while not self.laser_list and not rospy.is_shutdown():
            rospy.loginfo('No laser data available ...')
            rospy.sleep(0.5)

    def centerSpread(self, spread_value, left_right):
        self.center_range = self.laser_list
        if left_right == 'right':
            del self.center_range[0 : self.center_index]  # 右半分削除
            del self.center_range[-self.center_index+1+spread_value : ]  # リストの先頭から1つずつ追加
        elif left_right == 'left':
            del self.center_range[self.center_index+1 : self.index_sum]  # 左半分削除
            del self.center_range[ : -1-spread_value]  # リストの末尾から1つずつ追加
            self.center_range = list(reversed(self.center_range))
        else:
            rospy.loginfo('Not left_right value')

    def centerIndex(self, left_right):
        bag_range = []
        bag_dist = []
        scan_index_list = []
        scan_data_list = []
        bag_average = 'NULL'
        self.laserIndex()
        count = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.05)
            self.centerSpread(count, left_right)
            laser_average = self.average(self.center_range)
            if laser_average - 0.1 > self.center_range[-1]:
                bag_range.append(count)
                bag_dist.append(self.center_range[-1])
                bag_average = self.average(bag_dist)
            elif bag_average != 'NULL' and bag_average + 0.1 < self.center_range[-1]:
                print('=========================')
                break
            else:
                pass
            scan_index_list.append(count)
            scan_data_list.append(self.laser_list[count])
            print(bag_range)
            count += 1
        return bag_range[self.roundHalfUp(len(bag_range)/2 - 1)]#, scan_index_list, scan_data_list

    def indexToAngle(self, left_right):
        self.laserCheck()
        angle_to_bag = self.centerIndex(left_right)
        return angle_to_bag*self.step_angle if left_right == 'left' else -1*angle_to_bag*self.step_angle
    def bagFocus(self, left_right, rotate_speed=0.2):
        move_angle = self.indexToAngle(left_right)
        print(move_angle)
        self.base_control.rotateAngle(move_angle*0.9, rotate_speed)  # 回転の調整はここ
        while self.rotate_value != 0.0:
            rospy.loginfo('Rotating ...')
            rospy.sleep(0.5)
        rospy.sleep(0.5)
        return self.laser_list[self.center_index]

    def bagGrasp(self, left_right, coordinate=[0.4, 0.55]):
        self.arm_pose('carry')
        self.eef.publish(False)
        self.bagFocus(left_right)
        print(coordinate)
        self.manipulation(coordinate)
        rospy.sleep(0.5)
        self.base_control.translateDist(self.laser_list[self.center_index] - 0.15)
        self.eef.publish(True)
        rospy.sleep(0.5)
        self.arm_pose('carry')
        rospy.loginfo('I have a bag.')

    def srv_FindBag(self, srv_req):
        distance = self.bagFocus(srv_req.left_right, srv_req.rotate_speed)
        return FindBagSrvResponse(result = True, distance = distance)

    def srv_GraspBag(self, srv_req):
        self.bagGrasp(srv_req.left_right, srv_req.data)
        return GraspBagSrvResponse(result = True)

    def scanPlot(self, left_right):
        gavege = 'NULL'
        scan_index_list = []
        scan_data_list = []
        rospy.sleep(0.5)
        gavege, scan_index_list, scan_data_list = self.centerIndex(left_right)
        plot.plot(scan_index_list, scan_data_list)
        plot.show()


if __name__ == '__main__':
    rospy.init_node('find_bag_node')
    fb = FindBag()
    rospy.spin()
    #fb.scanPlot('left')
