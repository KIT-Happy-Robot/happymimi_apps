#!/etc/bin/env python3
# 

from launch import LaunchDesctiption
from launch_ros.actions import Node

def genLaunchDesc():
     #arg name=map_name
     #rosparam file=~/~.yml

     map_server = Node(package='map_server', executable='

     return LaunchDescription([map_server, ])
