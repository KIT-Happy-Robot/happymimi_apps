"""
approach Personから派生して、Itemに近づくための座標などを取得するためのクラス
このプログラムはTidyUpのLongTableAからアイテムをとるためのプログラム
"""
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

class GenerateItemCoord():
    def __init__(self):
        self.sac = actionlib.SimpleActionClient('pub_human_tf', PubHumanTFAction)
        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Value
        self.sac.wait_for_server()
        self.goal = PubHumanTFGoal()
        self.rate = rospy.Rate(10.0)
        self.item_dict = {}

    def execute(self, frame_name, dist_x, dist_y):
        self.goal.name = frame_name
        self.goal.dist_x = dist_x
        self.goal.dist_y = dist_y
        self.sac.send_goal(self.goal)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                self.item_dict[frame_name] = []
                self.item_dict[frame_name].append(trans.transform.translation.x)
                self.item_dict[frame_name].append(trans.transform.translation.y)
                self.item_dict[frame_name].append(trans.transform.rotation.z)
                self.item_dict[frame_name].append(trans.transform.rotation.w)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            self.rate.sleep()
        self.sac.cancel_goal()
        print(f"gic >>> {self.item_dict}")
        return self.item_dict
    
class ItemCoordGeneratorSrv():
    def __init__(self):
        rospy.loginfo("Ready to item_coord_generator server")
        #Service通信のクライアントを作成
        self.multi_localize = rospy.ServiceProxy('/recognition/multiple_localize', MultipleLocalize)
        #paramのよみこみ
        self.table_range = rospy.get_param('/table_range')
        # Value
        self.dist_data = MultipleLocalize()
        self.gic = GenerateItemCoord()
        self.bc = BaseControl()
        self.item_coord_dict = {}
        self.h_dict_count = 0
        self.i = 0
    
    def saveDict(self):
        param_path = roslib.packages.get_pkg_dir('happymimi_params')
        rospy.set_param('/tmp_item_location', self.item_coord_dict)
        rosparam.dump_params(param_path + '/location/tmp_item_location.yaml', '/tmp_item_location')
    
    def judgeMapping(self,coord):
        self.table_range = rospy.get_param('/table_range')
        # print rpy
        if coord[0] < self.table_range[("min_x") or coord[0] > self.table_range["max_x"]]:
            return False
        elif coord[1] < self.table_range[("min_y") or coord[0] > self.table_range["max_y"]]:
            return False
        else:
            return True
        
    def change_dict_key(self, d, old_key):
        new_key = "item_" + str(len(self.item_coord_dict) + 1)
        d[new_key] = d.pop(old_key)
        del d[old_key]

    def createDict(self,list_len):
        for i in range(list_len):
            frame_id = "item_" + str(i)
            item_dict = self.gic.execute(frame_id, self.dist_data.points[i].x, self.dist_data.points[i].y)

            if self.judgeMapping(item_dict[frame_id]):
                if frame_id in self.item_coord_dict:
                    self.change_dict_key(item_dict, frame_id)
                self.item_coord_dict.update(item_dict)
                print(self.item_coord_dict)
                self.h_dict_count += 1
            else:
                pass
    
    def execute(self,srv_req):
        print("count num:" + str(self.h_dict_count))
        # アイテムがあるのかどうか 【！！！！！！複数あるのか要確認】
        self.dist_data = self.multi_localize(target_name = "cup")
        list_len = len(list(self.dist_data.points))

        if (list_len == 0):
            pass
        else:
            self.createDict(list_len)
        
        self.saveDict()
        print("======================")
        self.item_coord_dict.clear()
        return SimpleTrgResponse(result = True)

if __name__ == '__main__':
    rospy.init_node('item_coord_generator_server')
    try:
        icgs = ItemCoordGeneratorSrv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass