#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: デバッグ用
# Author: Issei Iida
#---------------------------------------------------------------------

import sys
import rospy
import actionlib
 
from actplan_executor.msg import APExecutorAction, APExecutorGoal

def main():
    rospy.loginfo("Start ActplanExector")
    ac = actionlib.SimpleActionClient('actplan_executor', APExecutorAction)
    ac.wait_for_server()

    goal = APExecutorGoal()

    # action = ['go', 'grasp', 'go', 'give']
    action = ['find', 'give', 'speak', 'answer']
    data = ['any', 'red cup', 'operator', 'red cup']
    goal.action = action
    goal.data = data

    ac.send_goal(goal)
    ac.wait_for_result()

    result = ac.get_result().data
    if result == 'success':
        rospy.loginfo("Success ExeActionPlan")
        return True
    else:
        rospy.loginfo("Failed ExeActionPlan")
        return False

if __name__ == '__main__':
    rospy.init_node('test')
    main()
