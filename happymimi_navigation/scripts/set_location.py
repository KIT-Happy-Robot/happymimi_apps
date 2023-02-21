#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: Issei Iida
#-----------------------------------------------------------
import subprocess as sp
import rospy
import rosparam
import roslib.packages
from tf import TransformListener
from happymimi_navigation.srv import SetLocation, SetLocationResponse


class SetLocationServer():
    def __init__(self):
        service = rospy.Service('/set_location_server', SetLocation, self.checkState)
        rospy.loginfo("Ready to set_location_server")
        # Value
        self.tf = TransformListener()
        self.location_dict = {}
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def getMapPosition(self):
        position, rotation = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
        self.location_pose_x = position[0]
        self.location_pose_y = position[1]
        self.location_pose_z = rotation[2]
        self.location_pose_w = rotation[3]

    def checkState(self, srv_req):
        if srv_req.state == 'add':
            rospy.loginfo("Add location")
            return SetLocationResponse(result = self.addLocation(srv_req.name))
        elif srv_req.state == 'save':
            rospy.loginfo("Save location")
            return SetLocationResponse(result = self.saveLocation(srv_req.name))
        else:
            rospy.logerr("<" + srv_req.state + "> state doesn't exist.")
            return SetLocationResponse(result = False)

    def addLocation(self, name):
        if name in self.location_dict:
            rospy.logerr('<' + name + '> has been registerd. Please enter a different name.')
            return False
        elif name == '':
            rospy.logerr("No location name enterd.")
            return False
        else:
            self.getMapPosition()
            self.location_dict[name] = []
            self.location_dict[name].append(self.location_pose_x)
            self.location_dict[name].append(self.location_pose_y)
            self.location_dict[name].append(self.location_pose_z)
            self.location_dict[name].append(self.location_pose_w)
            print(self.location_dict)
            rospy.loginfo("Registerd <" + name + ">")
            return True

    def saveLocation(self, file_name):
        try:
            param_path = roslib.packages.get_pkg_dir("happymimi_params")
            map_path = roslib.packages.get_pkg_dir("happymimi_navigation")
            rospy.set_param('/location_dict', self.location_dict)
            rosparam.dump_params(param_path + '/location/' + file_name + '.yaml', '/location_dict')
            print(rosparam.get_param('/location_dict'))
            sp.Popen(['rosrun','map_server','map_saver','-f', map_path + '/maps/'+ file_name])
            rospy.loginfo("Saved as <" + file_name + ">")
            return True
        except rospy.ROSInterruptException:
            rospy.logerr("Could not save.")
            return False


if __name__ == '__main__':
    rospy.init_node('set_location_server', anonymous = True)
    try:
        sls = SetLocationServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
