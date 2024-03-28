#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するServiceServer
# Author: Yusuke Kanazawa
# Data: 2021/09/17
# Memo: ドアが閉まっていることを前提条件とする
# サービスで「進む距離」と「速度」を指定できる
#-------------------------------------------------------------
import rospy
import roslib
import sys
from sensor_msgs.msg import LaserScan
from enter_room.srv import EnterRoom, EnterRoomResponse
from happymimi_msgs.srv import StrTrg
file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl


# 速度と進む距離を取得
# 
class EnterRoomServer():
    def __init__(self):
        service = rospy.Service('/enter_room_server', EnterRoom, self.execute)
        rospy.loginfo("Ready to set enter_room_server")
        # speak
        self.tts_srv = rospy.ServiceProxy('/tts', StrTrg)
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        # Module
        self.base_control = BaseControl()
        # Value
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, srv_req):
        try:
            safe_dist = 1.0
            target_dist = srv_req.distance + self.front_laser_dist
            print(target_dist)

            self.tts_srv('Prease open the door')
            print("Prease open the door")
            while self.front_laser_dist <= safe_dist:
                rospy.sleep(0.1)
            self.tts_srv('Thank you')
            self.base_control.translateDist(target_dist, srv_req.velocity)
            return EnterRoomResponse(result = True)
        except rospy.ROSInterruptException:
            rospy.logger("!!Interrupt!!")
            return EnterRoomResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('enter_server')
    ers = EnterRoomServer()
    rospy.spin()
