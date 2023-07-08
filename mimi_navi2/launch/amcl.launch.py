#!/usr/bin/env python3
#-*- coding: utf-8 -*-

from launch import LaunchDescription
from lunch_ros.actions import Node

def generateLaunchDescription():
    return LaunchDescription([
        Node(
            package='mimi_navi2',
            executable='navi2_location_acserver',
            #name='navi2_location_acserver',
            output='screen',
            emulate_tty=true,
            parameters=[
                {'location_list': ['']}
            ]
        )
    ]) 
