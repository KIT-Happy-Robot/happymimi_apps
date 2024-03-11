#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionPlanから行動を決定して実行するアクションサーバー
# Author: Issei Iida
# Date: 2020/10/18
#-----------------------------------------------------------------------------
import sys
import rospy
import smach
import rosparam
import actionlib
from smach_ros import ActionServerWrapper

from actplan_executor.msg import APExecutorAction
from std_msgs.msg import Float64
from happymimi_msgs.srv import StrTrg
from happymimi_navigation.srv import NaviLocation
from happymimi_manipulation_msgs.srv import RecognitionToGrasping
from happymimi_recognition_msgs.srv import RecognitionCount
from happymimi_voice_msgs.srv import TTS

# speak
tts_srv = rospy.ServiceProxy('/tts', TTS)

class DecideAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['move', 'mani', 'find',
                                               'voice', 'all_finish'],
                             input_keys = ['goal_in', 'a_num_in', 'result_message'],
                             output_keys = ['a_action_out', 'a_data_out',
                                            'a_num_out', 'result_message'])
        # Param
        self.action_state = rosparam.get_param('/act_state')

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_ACTION')
        a_count = userdata.a_num_in
        a_plan = userdata.goal_in
        print(str(a_count))
        print(a_plan)
        if a_count < len(a_plan.action):
            a_name = a_plan.action[a_count]
            userdata.a_action_out = a_name
            userdata.a_data_out = a_plan.data[a_count]
            return self.action_state[a_name]
        else:
            rospy.loginfo('all success')
            userdata.a_num_out = 0
            userdata.result_message.data = 'success'
            return 'all_finish'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['move_finish', 'move_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Service
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        if name == 'go':
            print(data)
            tts_srv('Move to ' + data)
            result = self.navi_srv(data).result
        elif name == 'approach':
            tts_srv('Move to ' + data)
            # 人接近処理を追加する
            result = True
        elif name == 'follow':
            tts_srv("start human following")
            result = True
        else:
            rospy.logerr("Action name failed")
            return 'move_finish'
        if result:
            userdata.a_num_out = a_count + 1
            return 'move_finish'
        else:
            userdata.a_num_out = 0
            return 'move_failed'


class Mani(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['mani_finish', 'mani_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out'])
        # Service
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Param
        self.object_dict = rosparam.get_param('/object_mapping')
        self.grasp_msg = RecognitionToGrasping()

    def execute(self, userdata):
        rospy.loginfo('Executing state: MANI')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        if name == 'grasp':
            obj = self.object_dict[data]
            result = self.grasp_srv(target_name=obj).result
            # result = self.grasp_srv(target_name='cup').result
            # result = True
        elif name == 'place':
            result = self.arm_srv('place').result
            # result = True
        elif name == 'give':
            self.head_pub.publish(-15)
            tts_srv('Here you are')
            result = self.arm_srv('give').result
            # result = True
        else:
            rospy.logerr("Action name failed")
            return 'mani_finish'
        if result:
            userdata.a_num_out = a_count + 1
            return 'mani_finish'
        else:
            userdata.a_num_out = 0
            return 'mani_failed'


class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_finish', 'find_failed'],
                             input_keys = ['action_in', 'data_in', 'num_in'],
                             output_keys = ['a_num_out', 'obj_num_out'])
        # Service
        self.count_srv = rospy.ServiceProxy('/recognition/count', RecognitionCount)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Param
        self.obj_map = rosparam.get_param('/object_mapping')

    def execute(self, userdata):
        rospy.loginfo('Executing state: find')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        if name == 'find':
            # obj = self.obj_map[data]
            # self.head_pub(20)
            # rospy.sleep(1.5)
            # tts_srv('I find ' + data)
            # # obj_num = self.count_srv(obj).num
            # obj_num = 1
            # result = bool(obj_num)
            # obj_num = 1
            result = True
        else:
            rospy.logerr("Action name failed")
            return 'find_finish'
        if result:
            # tts_srv("I found " + str(obj_num) + data)
            # userdata.a_num_out = a_count + 1
            # userdata.obj_num_out = str(obj_num)
            return 'find_finish'
        else:
            tts_srv("I could't find " + data)
            userdata.a_num_out = 0
            return 'find_failed'


class Voice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['voice_finish', 'voice_failed'],
                             input_keys = ['action_in', 'data_in',
                                           'num_in', 'obj_num_in'],
                             output_keys = ['a_num_out'])
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: VOICE')
        a_count = userdata.num_in
        name = userdata.action_in
        data = userdata.data_in
        obj_num = userdata.obj_num_in
        self.head_pub.publish(0)
        if name == 'speak':
            # tts_srv(data)
            result = True
        elif name == 'answer':
            # tts_srv(data)
            # WDYSみたいなサービスを呼ぶ
            result = True
        userdata.a_num_out = a_count + 1
        return 'voice_finish'


if __name__ == '__main__':
    rospy.init_node('exe_actplan')
    sm_top = smach.StateMachine(outcomes = ['success', 'action_failed', 'preempted'],
                          input_keys = ['goal_message', 'result_message'],
                          output_keys = ['result_message'])
    sm_top.userdata.action_num = 0
    sm_top.userdata.obj_num = 'none'
    with sm_top:
        smach.StateMachine.add('DECIDE_ACTION', DecideAction(),
                         transitions = {'move':'MOVE',
                                        'mani':'MANI',
                                        'find':'FIND',
                                        'voice':'VOICE',
                                        'all_finish':'success'},
                         remapping = {'goal_in':'goal_message',
                                      'a_num_in':'action_num',
                                      'a_action_out':'action_name',
                                      'a_data_out':'data_name',
                                      'a_num_out':'action_num',
                                      'result_out':'result_message'})

        smach.StateMachine.add('MOVE', Move(),
                         transitions = {'move_finish':'DECIDE_ACTION',
                                        'move_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num'})

        smach.StateMachine.add('MANI', Mani(),
                         transitions = {'mani_finish':'DECIDE_ACTION',
                                        'mani_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num'})

        smach.StateMachine.add('FIND', Find(),
                         transitions = {'find_finish':'DECIDE_ACTION',
                                        'find_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num',
                                      'obj_num_out':'obj_num'})

        smach.StateMachine.add('VOICE', Voice(),
                         transitions = {'voice_finish':'DECIDE_ACTION',
                                        'voice_failed':'action_failed'},
                         remapping = {'action_in':'action_name',
                                      'data_in':'data_name',
                                      'num_in':'action_num',
                                      'a_num_out':'action_num',
                                      'obj_num_in':'obj_num'})

    asw = ActionServerWrapper('actplan_executor', APExecutorAction,
                              wrapped_container = sm_top,
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['action_failed'],
                              preempted_outcomes = ['preempted'],
                              goal_key = 'goal_message',
                              result_key = 'result_message')
    asw.run_server()
    rospy.spin()
