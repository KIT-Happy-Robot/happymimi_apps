#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# "navi_location.py" ã® ROS2ç‰?

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup #!!!
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose # NG -> NavigateToPoseGoal
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mimi_navi2.srv import Navi2Location, Navi2LocationResponse


class NaviLocationServer2(Node):
  def __init__(self):
    super().__init__('navi_location_server2')
    service = self.create_service(Navi2Location, 'navi_location_server',
                                  self.searchLocationName)
    self.get_logger().info("Ready to navi_location_server2")
    # Action
    self.ac = self.ActionClient(self, NavigateToPose, 'navigate_to_pose')
    # Publisher
    self.crloc_pub = self.create_publisher(String, '/current_location', queue_size = 1)
    self.head_pub = self.create_publisher(Float64, '/servo/head', queue_size = 1)
    # Service
    #self.clear_costmap = self.create_client(Empty, '/move_base/clear_costmaps')
    #while not self.clear_costmap.wait_for_service(1.0):
    #  self.get_logger().warn('waiting for service... '/move_base/clear_costmaps')
    #future = self.clear_costmap.call_async(request)
    #rclpy.spin_until_future_complete(self, future)
    
    # Value
    self.location_dict = self.declare_parameter('/location')
    self.location_name = "null"

  def searchLocationName(self, req, res):
    if req.location_name in self.location_dict:
      self.location_name = req.location_name
      self.get_logger().info(self.location_dict[self.location_name])
      return self.sendGoal(self.location_dict[self.location_name])
    else:
      self.get_logger().info("<" + req.location_name + "> doesn't exist.")
      #!!! res.result = False
      return res(result = False) # !!!

  def sendGoal(self, location_list):
    time_stamp = Clock.now()
    # set goal_pose
    goal = NavigateToPose.Goal() # !!!
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = time_stamp.to_msg()
    goal.target_pose.pose.position.x = location_list[0]
    goal.target_pose.pose.position.y = location_list[1]
    goal.target_pose.pose.orientation.z = location_list[2]
    goal.target_pose.pose.orientation.w = location_list[3]
    # clearing costmap
    #self.get_logger().info("Clearing costmap...")
    #rospy.wait_for_service('move_base/clear_costmaps')
    #self.clear_costmap()
    #rospy.sleep(0.5)
    # start navigation
    self.head_pub.publish(0)
    self.ac.wait_for_server()
    # send request
    self.future = self.ac.send_goal_async(goal) #,self.naviFeedbackCB
    self.future.add_done_callback(self.naviResponseCB)
    
    # self.ac.wait_for_result()
  
  #def naviFeedbackCB(self, feedback):
      
  def naviResponseCB(self, future):
    gh = future.result()
    if not gh.accepted:
      self.get_logger().info('"NaviToPose" Goal rejected :(')
      return #!!!
    self._gh = gh
    self.get_logger().info('"NaviToPose" Goal accepted :)')
    
    navi_state = self.ac.get_state()
    while not rclpy.shutdown():
      navi_state = self.ac.get_state()
      if navi_state == 3:
        self.get_logger().info('Navigation success!!')
        self.crloc_pub.publish(self.location_name)
        return NaviLocationResponse(result = True)
      elif navi_state == 4:
        self.get_logger().info('Navigation Failed')
        return NaviLocationResponse(result = False)
      else:
        pass

def main(args=None):
  rclpy.init(args=args)
  try:
    # generate 'navi_location_server2'
    nls2 = NaviLocationServer2()
    rospy.spin()
  expt rclpy.ROSInterruptException:
    pass
  
if __name__ == '__main__':
  main()
#  rclpy.init('navi_location_server', anonymous = True)
#  sls = NaviLocationServer2()
#  rospy.spin()
