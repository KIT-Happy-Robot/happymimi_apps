#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#未完成
import rospy
from happymimi_recognition_msgs.srv import LeftRight2xyz
import roslib
import sys
import os
from std_msgs.msg import String,Bool,Float64
from happymimi_msgs.srv import StrTrg,SetStr
from happymimi_manipulation_msgs.srv import ArmControl
import math
from find_bag.srv import FindBagV8Srv,FindBagV8SrvResponse


teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

class FindBagV8():
    def __init__(self):
        #rospy.Subscriber("/left_right_recognition", String, self.LRCB)
        #rospy.Subscriber("/direction_of_hands", String, self.LRCB)
        self.lrsrv = rospy.ServiceProxy('direction_of_hands_srv',SetStr)
        self.bag = rospy.ServiceProxy('Coordinate_PaperBag',LeftRight2xyz)
        self.eef = rospy.Publisher('/servo/endeffector',Bool,queue_size=10)
        self.arm_pose = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.manipulation = rospy.ServiceProxy('/servo/debug_arm', ArmControl)
        self.arm_debug = rospy.ServiceProxy('/servo/debug_arm',ArmControl)
        self.head = rospy.Publisher('/servo/head',Float64,queue_size=10)
        self.base_control = BaseControl()
        self.h = 0.2    #肩モータとrealsenseの距離[m]
        self.lr = None
        self.center_x = 320 #画像サイズが640の場合(640/2)
        self.center_y = 240 #画像サイズが480の場合(480/2)
        srv = rospy.Service("find_paper_bag_v8",FindBagV8Srv,self.grasp_bag)
        
        rospy.loginfo("start find bag")
        rospy.loginfo("waithing...")

    """
    #人の指さした方向をもらう
    def LRCB(self, msg):
        self.lrmsg = msg.data
        if self.lrmsg != "None":
            self.lr = self.lrmsg
    """
    #紙バッグの座標を返してもらう
    def get_bag_dist(self):
        while not rospy.is_shutdown():
            self.x,self.y,self.z = self.bag(self.lr)
            print("x:{},y:{},z:{}".format(self.x,self.y,self.z))
            if self.x == -1 or self.y == -1 or self.z == -1:
                rospy.loginfo("paper bag not found")
            else:
                rospy.loginfo("found a paper bag")
                break

    def grasp_bag(self,req):
        self.req = req
        try:
            self.head.publish(0)
            rospy.sleep(0.5)
            self.lr = self.lrsrv().result
            print(self.lr)
            self.arm_pose('carry')
            rospy.sleep(0.5)
            self.eef.publish(False)
            rospy.sleep(0.5)
            self.head.publish(20)
            rospy.sleep(0.5)
            print("gc")
            self.get_bag_dist() #xyzの値を取得
            x1 = self.x
            z_theta = self.z * (math.pi / 12) #rθ,θ=pi/12,r=z
            change_units = z_theta / abs(self.center_x - x1) #単位を変更するため
            rospy.sleep(0.5)

            rospy.loginfo('Rotation for measurement')
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

            rospy.loginfo('add Rotation')
            #追加で回転
            if x1 < 320:
                self.base_control.rotateAngle(-add_move_angle, precision=1, speed=0.7, time_out=20)
            elif x1 > 320:
                self.base_control.rotateAngle(add_move_angle, precision=1, speed=0.7, time_out=20)
            
            rospy.sleep(0.5)

            #接近
            rospy.loginfo('approach bag')
            self.get_bag_dist() #xyzの値を更新
            rospy.sleep(0.5)
            self.base_control.translateDist(self.z - 1.0)
            for i in range(21,30):
                self.get_bag_dist()
                if self.y >= 230 and self.y <= 250: #230<=y<=250
                    theta = i
                    break
                else:
                    self.head.publish(i)


            arm_x = self.z * math.cos(theta*(math.pi/180)) - self.h
            distance = self.z * math.sin(theta*(math.pi/180)) - 0.25
            arm_y = self.z * math.sin(theta*(math.pi/180)) - distance
            coordinate = [arm_x,arm_y]
            print(coordinate)
            self.arm_debug(coordinate)
            rospy.sleep(0.5)
            self.base_control.translateDist(distance)
            rospy.sleep(0.5)
            self.eef.publish(True)
            rospy.sleep(0.5)
            self.arm_pose('carry')
            rospy.loginfo('I have a bag.')
            return FindBagV8SrvResponse(True)
        except:
            return FindBagV8SrvResponse(False)

        
if __name__ == '__main__':
    rospy.init_node('find_bag_v8_node')
    fb = FindBagV8()
    rospy.spin()
