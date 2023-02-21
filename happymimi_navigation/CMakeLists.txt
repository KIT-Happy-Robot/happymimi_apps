cmake_minimum_required(VERSION 3.0.2)
project(happymimi_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  tf
  roslib
  rosparam
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  dynamic_reconfigure
)

add_service_files(
  FILES
  NaviLocation.srv
  NaviCoord.srv
  SetLocation.srv
  AdNaviSrv.srv
)

add_action_files(DIRECTORY action
  FILES
  AdNaviAct.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES happymimi_navigation
  CATKIN_DEPENDS roscpp rospy sensor_msgs tf std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(ad_navi_server src/ad_navi_server.cpp)

#target_link_libraries(
#  ad_navi_server
#  ${catkin_LIBRARIES}
#)

catkin_install_python(
  PROGRAMS
  scripts/set_location.py
  scripts/navi_coord.py
  scripts/navi_location.py
  scripts/ad_navi_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
