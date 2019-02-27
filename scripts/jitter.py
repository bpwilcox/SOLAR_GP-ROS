#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from bwrobot.srv import Jitter
from copy import deepcopy, copy


class JitRobot():
    def __init__(self, names, joint_index):
        self.cmd = JointState()
        self.joint_names= names
        self.cmd.name = names
        self.cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Y = np.zeros(len(joint_index)).reshape(1, len(joint_index))
        self.ind = joint_index
        rospy.Subscriber('joint_states', JointState, self.callback)

        self.jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.jit_service= rospy.Service('jitter',Jitter, self.jitter)

    def callback(self, data):
        self.cmd = data
        for counter, index in enumerate(self.ind):
            self.Y[0,counter] = data.position[index]

    def jitter(self, req):
        pert = req.degree * 0.0174533 * np.random.uniform(-1.,1.,(req.num_jit, np.size(self.Y,1)))
        YI = self.Y + pert

        position = list(self.cmd.position)
        for y in YI:
            self.cmd.header.stamp = rospy.Time.now()
            for counter, index in enumerate(self.ind):
                position[index] = y[counter]
            self.cmd.position = position
            self.jspub.publish(self.cmd)
            rospy.Rate(5).sleep()

        return True


def jitter_server():

    rospy.init_node('jitter_node')

    all_joint_names = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0',
    'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint',
    'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']

    "Robot settings"
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    #joint_names = ['right_s0', 'right_s1', 'right_e1', 'right_w1']
    ind = []
    for joint in joint_names:
        ind.append(all_joint_names.index(joint))
    
    RobotState = JitRobot(all_joint_names, ind)

    rospy.spin()

if __name__ == '__main__':
    try:
        jitter_server()
    except rospy.ROSInterruptException:
        pass
