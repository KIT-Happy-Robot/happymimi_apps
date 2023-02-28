#!/usr/bin/env python3

import rospy
import roslib

param_path = roslib.packages.get_dir_pkg("find_bag")
print(type(param_path))
print(param_path)
