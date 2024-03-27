#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import rospy
import roslib
import tf2_ros
import tf
import rosparam
import actionlib

from geometry_msgs.msg import Point
from happymimi_msgs.srv import SimpleTrg, SimpleTrgResponse
from happymimi_recognition_msgs.srv import MultipleLocalize
from approach_person.msg import PubHumanTFAction, PubHumanTFGoal

file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

#add koya
file_yolo_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
file_yolo_config = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/config/uor_object_location.yaml'
sys.path.insert(1,file_yolo_path)
from yolow_server import YoloWorld_Server

class GenerateHumanCoord():
    def __init__(self):
        self.sac = actionlib.SimpleActionClient('pub_human_tf', PubHumanTFAction)
        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Value
        self.sac.wait_for_server()
        self.goal = PubHumanTFGoal()
        self.rate = rospy.Rate(10.0)
        self.human_dict = {}

    def execute(self, frame_name, dist_x, dist_y):
        self.goal.name = frame_name
        self.goal.dist_x = dist_x
        self.goal.dist_y = dist_y
        self.sac.send_goal(self.goal)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                self.human_dict[frame_name] = []
                self.human_dict[frame_name].append(trans.transform.translation.x)
                self.human_dict[frame_name].append(trans.transform.translation.y)
                self.human_dict[frame_name].append(trans.transform.rotation.z)
                self.human_dict[frame_name].append(trans.transform.rotation.w)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            self.rate.sleep()
        self.sac.cancel_goal()
        print(f"ghc >>> {self.human_dict}")
        return self.human_dict


class HumanCoordGeneratorSrv():
    def __init__(self):
        hcg_srv = rospy.Service('human_coord_generator', SimpleTrg, self.execute)
        rospy.loginfo("Ready to human_coord_generator server")
        # Service
        self.ml_srv = rospy.ServiceProxy('/recognition/multiple_localize', MultipleLocalize)
        # Param
        self.map_range = rospy.get_param('/map_range')
        # Value
        self.dist_data = MultipleLocalize()
        self.ghc = GenerateHumanCoord()
        
        ##add koya
        self.yoloworld = YoloWorld_Server()
        ##
        self.bc = BaseControl()
        self.human_coord_dict = {}
        self.h_dict_count = 0
        self.i = 0

    def saveDict(self):
        param_path = roslib.packages.get_pkg_dir("happymimi_params")
        #os.remove(param_path + '/location/'  + 'tmp_human_location.yaml')
        rospy.set_param('/tmp_human_location', self.human_coord_dict)
        rosparam.dump_params(param_path + '/location/'  + 'tmp_human_location.yaml', '/tmp_human_location')

    def judgeMapin(self, coord):
        self.map_range = rospy.get_param('/map_range')
        rpy = coord
        # print rpy
        if coord[0] < self.map_range["min_x"] or coord[0] > self.map_range["max_x"]:
            jm_result = False
        elif coord[1] < self.map_range["min_y"] or coord[0] > self.map_range["max_y"]:
            jm_result = False
        else:
            jm_result = True
        return jm_result

    # def change_dict_key(self, d, old_key, new_key):
    def change_dict_key(self, d, old_key):
        #self.human_coord_dict.clear()
        new_key = "human_" + str(len(self.human_coord_dict) + 1)
        #new_key = "human_" + str(self.i)
        #new_key = "human_0"
        print(new_key)
        d[new_key] = d[old_key]
        print(old_key)
        del d[old_key]

    def createDict(self, list_len):
        # map座標系に変換してlocation dictを作成
        for i in range(list_len):
            frame_id = "human_" + str(i)
            #frame_id + "human_0"
            human_dict = self.ghc.execute(frame_id, self.dist_data.points[i].x, self.dist_data.points[i].y)
            if self.judgeMapin(human_dict[frame_id]):
                #new_id = "human_" + str(self.h_dict_count)
                if frame_id in self.human_coord_dict:
                    # self.change_dict_key(human_dict, frame_id, new_id)
                    self.change_dict_key(human_dict, frame_id)
                self.human_coord_dict.update(human_dict)
                print (self.human_coord_dict)
                self.h_dict_count += 1
            else:
                pass
        #self.h_dict_count += 1
       # self.i += 1

    def execute(self, srv_req):
        
        print ("count num: " + str(self.h_dict_count))
        # 人がいるか
        self.dist_data = self.ml_srv(target_name = "person")
        
        #add Koya
        self.dist_data = YoloWorld_Server.load_yaml_to(file_yolo_path)
        ##
        
        print(self.dist_data)
        list_len  = len(list(self.dist_data.points))
        if list_len == 0 :
            pass
        else:
            self.createDict(list_len)
        self.saveDict()
        print("====================")
        print(self.human_coord_dict)
        self.human_coord_dict.clear()
        return SimpleTrgResponse(result = True)


if __name__ == '__main__':
    rospy.init_node('human_coord_generator')
    try:
        hcgs = HumanCoordGeneratorSrv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
