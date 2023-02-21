#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rosparam
import actionlib
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from happymimi_navigation.srv import NaviCoord, NaviCoordResponse


class NaviCoordServer():
    def __init__(self):
        service = rospy.Service('navi_coord_server', NaviCoord, self.sendGoal)
        rospy.loginfo("Ready to navi_coord_server")
        # Action
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Service
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # Value
        self.location_name = "null"

    def sendGoal(self, srv_req):
        location_list = srv_req.loc_coord
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_list[0]
        goal.target_pose.pose.position.y = location_list[1]
        goal.target_pose.pose.orientation.z = location_list[2]
        goal.target_pose.pose.orientation.w = location_list[3]

        self.head_pub.publish(0)
        rospy.sleep(0.5)

        rospy.loginfo("Clearing costmap...")
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmap()

        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        # self.ac.wait_for_result()
        navi_state = self.ac.get_state()
        while not rospy.is_shutdown():
            navi_state = self.ac.get_state()
            if navi_state == 3:
                rospy.loginfo('Navigation success!!')
                return NaviCoordResponse(result = True)
            elif navi_state == 4:
                rospy.loginfo('Navigation Failed')
                return NaviCoordResponse(result = False)
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('navi_coord_server')
    try:
        sls = NaviCoordServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
