#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# just a location_name requesst is preferred
# 首の角度は変えない
import rospy
import rosparam
import dynamic_reconfigure.client
import actionlib
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import sys
import roslib
#mimi_navi_path = roslib.packages.get_pkg_dir("happymimi_navigation")
self_path = roslib.packages.get_pkg_dir("monitoring_patrol")
sys.path.insert(0, "/${HOME}/test_ws/src/dev_noe/task/rcj22/final/")
#sys.path.insert(0, mimi_navi_path)

#sys.path.insert(0, "${HOME}/test_ws/src/dev_noe/task/rcj22/final")
self_path = roslib.packages.get_pkg_dir("monitoring_patrol") + "/src"
sys.path.insert(0, self_path)
#from happymimi_navigation.srv import NaviCoord, NaviCoordRequest
from monitoring_patrol.srv import AdNaviSrv, AdNaviSrvResponse, AdNaviSrvRequest



# 拡張版オプション付き自律移動の実行クラス
class AdNaviServer():
  #navi_params_dict = rospy.get_param("/ad_navi_param")
  #server_type_dict = rospy.get_param("/ad_navi_param/AdNaviConstructor")
  #server_type_in = server_type_dict["server_type"]
  def __init__(self):  #, server_type = server_type_in):
    rospy.loginfo("Initialize **AdNaviServer**")
    # TOPIC
    # Subscriber
    #rospy.Subscriber("/apps/ad_navi_acserver"
    # Publisher
    #self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
    
    # SERVICE
    rospy.loginfo("ac_navi_server: ready to **ad_navi_server**")
    self.an_ss = rospy.Service('/apps/ad_navi_server', AdNaviSrv, self.serviceCB)
    self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    # if server_type == "service":   
    #   rospy.loginfo("ac_navi_server: ready to **ad_navi_server**")
    #   self.an_ss = rospy.Service('/apps/ad_navi_server', AdNaviSrv, self.serviceCB)
    # elif server_type == "action":
    #   rospy.loginfo("ac_navi_server: ready to **ad_navi_acserver**")
    #   self.an_ss = actionlib.SimpleActionServer('/apps/ad_navi_acserver', AdNaviAct, self.)
    
    # ACTION
    rospy.loginfo("ac_navi_server: ready to **ad_navi_acserver**")
    self.an_ss = actionlib.SimpleActionServer('/apps/ad_navi_acserver', AdNaviAct,
                                              execute_cb = self.actionVB,
                                              )
    self.move_base_ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    self.dwa_c = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
    
    # Param
    self.navi_def_params = rospy.get_param("/ad_navi_param/FastLoose")
    self.navi_fl_params = rospy.get_param("/ad_navi_param/Default")
    self.location_dict = rospy.get_param("/location_dict")
    self.target_coord = [] #!!
    # Data
    self.mb_goal = MoveBaseGoal
    
  # Requestの"option"に基づき、Naviのパラメーターを変更する関数
  def setParam(self, option = "default"): #!!
    if option == "fast_loose":
      self.dwa_c.update_configuration(self.navi_fl_params)
    elif option == "default":
      self.dwa_c.update_configuration(self.navi_def_params)
      rospy.sleep(0.5)
  
  # ロケーション名の姿勢座標パラメーターを姿勢座標のリスト(self.target_coord)に格納する関数
  def name2Coord(self, location_name):
    if location_name in self.location_dict:
      self.target_coord = self.location_dict[location_name]
      rospy.loginfo("ad_navi: target location_name ->" +location_name)
      return True
    else: 
      rospy.logerr("ad_navi: <"+location_name+"> doesn't exist.")
      return False
  
  # self.target_coordリストに格納する関数
  def setCoord(self, location_name, coordinate):
    #req = AdNaviSrvRequest()
    #req_coord = req.target_coord
    # ロケーション名なし->　self.targetに座標リストRequestを格納
    if location_name == "":
      self.target_coord = coordinate #req_coord #AdNaviSrvRequest(coordinate)
    # ロケーション名あり-> self.targetにロケーション名の座標を格納
    else:
      # Judge name2Coord result
      if self.name2Coord(location_name): return True
      else: rospy.loginfo("ad_navi: Failed to name2Coord"); return False
  
    
  # 姿勢座標のリストからゴールを作成する関数
  def createGoal(self, coord_list):
    rospy.loginfo("ad_navi: Create move base goal")
    # set goal_pose
    self.goal = MoveBaseGoal()
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = coord_list[0]
    self.goal.target_pose.pose.position.y = coord_list[1]
    self.goal.target_pose.pose.orientation.z = coord_list[2]
    self.goal.target_pose.pose.orientation.w = coord_list[3]
    
  def clearCostmap(self):
    # clearing costmap
    rospy.loginfo("ad_navi: Clearing costmap...")
    rospy.wait_for_service('move_base/clear_costmaps')
    self.clear_costmap()
    rospy.sleep(0.5)
    
  def sendGoal(self, goal):
    rospy.loginfo("ad_navi: Send move base goal")
    # 首は上げない
    #self.head_pub.publish(0)
    self.move_base_ac.wait_for_server()
    self.move_base_ac.send_goal(goal)
    self.move_base_ac.wait_for_result()
    navi_act_state = self.move_base_ac.get_state()
    while not rospy.is_shutdown():
      navi_act_state = self.ac.get_state()
      if navi_act_state == 3:
        rospy.loginfo('ad_navi: Navigation success!!')
        #!!
        return True
      elif navi_act_state == 4:
        rospy.loginfo('ad_navi: Navigation Failed')
        return False
      else:
        rospy.loginfo('ad_navi: Unkown MoveBase action state')
        return False
    
  # Requestに基づき、Naviの実行結果を返す
  def serviceCB(self, req):
    rospy.loginfo("Execute **ad_navi_server**")
    navi_param_flg = req.options
    response = AdNaviSrvResponse()
    
    # 目標座標を設定
    if self.setCoord(req.target_location, req.target_coord):
      pass
    else: return response.result == False
    # Naviのパラメーター設定
    self.setParam(option = req.options)
    # MoveBaseのゴール生成
    self.createGoal(self.target_coord)
    # MoveBaseへゴール送信
    if self.sendGoal(self.goal):
      # NaviのDWAPlannerROSパラメーター設定をデフォルトに戻す
      self.setParam(option = "default")
      return response.result == True
    else: return response.result == False
    
  # アクションサーバー
  # 
  def 
    
      
  # Requestに基づき、Naviの実行結果を返す
  def actionCB(self, ):
    rospy.loginfo("Execute **ad_navi_server**")
      
      
def main():
  rospy.init_node('ad_navi_server')
  try:
    ans = AdNaviServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()