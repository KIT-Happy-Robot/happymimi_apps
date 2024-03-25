#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#未完成
import rospy
from happymimi_recognition_msgs.srv import LeftRight2xyz
import roslib
import sys
import os
from std_msgs.msg import String,Bool
from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.srv import ArmControl
import math


teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

class FindBagV8():
    def __init__(self):
        rospy.Subscriber("/left_right_recognition", String, self.LRCB)
        self.bag = rospy.ServiceProxy('Coordinate_PaperBag',LeftRight2xyz)
        self.eef = rospy.Publisher('/servo/endeffector',Bool,queue_size=10)
        self.arm_pose = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.manipulation = rospy.ServiceProxy('/servo/debug_arm', ArmControl)
        self.base_control = BaseControl()

        self.center_x = 320 #画像サイズが640の場合(640/2)

    def LRCB(self, msg):
        self.lrmsg = msg.data

    def get_bag_dist(self):
        while not rospy.is_shutdown():
                self.x,self.y,self.z = self.bag(self.lrmsg)
                if self.x == -1 or self.y == -1 or self.z == -1:
                    rospy.loginfo("paper bag not found")
                else:
                    rospy.loginfo("found a paper bag")
                    break

    def grasp_bag(self):
        self.arm_pose('carry')
        self.eef.publish(False)
        self.get_bag_dist() #xyzの値を取得
        x1 = self.x
        z_theta = self.z * (math.pi / 12) #rθ,θ=pi/12,r=z
        change_units = z_theta / abs(self.center_x - x1) #単位を変更するため

        #計算のための計測
        if x1 < 320: #把持するバッグが中央から左にある場合
            self.base_control.rotateAngle(-15, precision=1, speed=0.7, time_out=20)
        elif x1 > 320:
            self.base_control.rotateAngle(15, precision=1, speed=0.7, time_out=20)    
        
        rospy.sleep(0.5)
        self.get_bag_dist() #xyzの値を更新
        x2 = self.x
        dx = abs(self.center_x - x2)
        move_angle = dx * change_units * (180 / math.pi) #初期角度から回転すべき角度
        add_move_angle = move_angle - 15 #追加で回転する角度

        #追加で回転
        if x1 < 320:
            self.base_control.rotateAngle(-add_move_angle, precision=1, speed=0.7, time_out=20)
        elif x1 > 320:
            self.base_control.rotateAngle(add_move_angle, precision=1, speed=0.7, time_out=20)
        
        rospy.sleep(0.5)

        #接近
        self.get_bag_dist() #xyzの値を更新
        self.base_control.translateDist(self.z - 0.5)
        rospy.sleep(0.5)


        #ここからyoloで対応できるか謎
        #位置合わせ
        #self.get_bag_dist() #xyzの値を更新
        self.eef.publish(True)
        rospy.sleep(0.5)
        self.arm_pose('carry')
        
        rospy.loginfo('I have a bag.')

        
        



if __name__ == '__main__':
    rospy.init_node('find_bag_v8_node')
    fb = FindBagV8()
    rospy.spin()




