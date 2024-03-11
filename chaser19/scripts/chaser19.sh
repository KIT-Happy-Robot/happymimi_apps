#!/bin/bash
xterm -geometry 80x5+0+0   -e "/opt/ros/melodic/bin/roslaunch happymimi_bringup minimal.launch" &
sleep 5s
xterm -geometry 80x5+0+130 -e "/opt/ros/noetic/bin/rosrun urg_node urg_node" &
sleep 5s
#xterm -geometry 80x5+0+360 -e "/opt/ros/kinectic/bin/rosbag record -a" &
#sleep 5s
xterm -geometry 80x5+0+910 -e "~/main_ws/devel/lib/chaser19/chaser19"
