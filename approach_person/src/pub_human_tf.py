#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import actionlib
import tf2_msgs.msg
from  geometry_msgs.msg import TransformStamped
from approach_person.msg import PubHumanTFAction, PubHumanTFResult


class PubHumanTFAS():
    def __init__(self):
        # Action
        self.sas = actionlib.SimpleActionServer('pub_human_tf', PubHumanTFAction,
                                                execute_cb = self.execute,
                                                auto_start = False)
        self.sas.start()
        rospy.loginfo("Ready to pub_human_tf ActionServer")
        # Publisher
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        # Value
        self.result = PubHumanTFResult()
        self.t = TransformStamped()

    def execute(self, req_msg):
        self.t.header.frame_id = "camera_depth_frame"
        self.t.child_frame_id = req_msg.name
        self.t.transform.translation.x = req_msg.dist_x
        self.t.transform.translation.y = req_msg.dist_y
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.t.header.stamp = rospy.Time.now()
            tfm = tf2_msgs.msg.TFMessage([self.t])
            self.pub_tf.publish(tfm)
            if self.sas.is_preempt_requested():
                self.sas.set_preempted()
                break
        # self.result = True
        # self.sas.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('pub_human_tf')
    pht_action = PubHumanTFAS()
    rospy.spin()
