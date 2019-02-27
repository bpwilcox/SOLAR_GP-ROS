#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import JointState


def jitter(n, Y_init, deg = 5):
    max_rough=0.0174533
    pert = deg*max_rough * np.random.uniform(-1.,1.,(n,np.size(Y_init,1)))
    Y_start = Y_init + pert
    return Y_start

def initialize():

    rospy.init_node('init_node')

    cmd = JointState()
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)

    all_joint_names = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0',
    'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint',
    'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cmd.name = all_joint_names

    "Robot settings"
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    num_joints = len(joint_names)
    YStart = rospy.get_param('~YStart')
    YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)
    ind = []
    for joint in joint_names:
        ind.append(all_joint_names.index(joint))

    "Initialize Local Models"
    njit = rospy.get_param('~njit')
    YI = jitter(njit, YStart, 5)
    rospy.Rate(0.5).sleep()

    for y in YI:        
        cmd.header.stamp = rospy.Time.now()
        for counter, index in enumerate(ind):
            position[index] = y[counter]
        cmd.position = position
        jspub.publish(cmd)
        rospy.Rate(5).sleep()

if __name__ == '__main__':
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
