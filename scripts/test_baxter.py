#!/usr/bin/env python

import rospy
import numpy as np
from data_buffer import DataBuffer

def test():

    rospy.init_node('baxter_test_node')

    x_topic = rospy.get_param('~x_topic', 'robot/limb/right/endpoint_state')
    y_topic = rospy.get_param('~y_topic', 'robot/joint_states')
    buffer_duration = rospy.get_param('~buffer_duration', 0.2)
    buffer_size = rospy.get_param('~buffer_size', 50)
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])    
    Buffer = DataBuffer(x_topic, y_topic, joint_names, buffer_duration, buffer_size)

    while not rospy.is_shutdown():
        print(len(Buffer.Xexp), len(Buffer.Yexp))
        Buffer.clear()
        rospy.Rate(1).sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
