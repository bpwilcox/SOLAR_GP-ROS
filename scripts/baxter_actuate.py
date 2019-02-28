#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from bwrobot.srv import *
from bwrobot.msg import *
from copy import copy, deepcopy

class ActRobot():
    def __init__(self):
        self.cmd = JointState()
        self.cmd.name = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0',
            'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint',
            'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
        self.cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.Subscriber('joint_states', JointState, self.callback)

    def callback(self, data):
        self.cmd = data

def actuate():

    "ROS Settings"

    rospy.init_node('actuator_node')
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
    R = rospy.get_param('~controller_pub_rate')
    rate = rospy.Rate(R)

    "Robot settings"
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    RobotState = ActRobot()
    num_joints = len(joint_names)
    ind = []
    for joint in joint_names:
        ind.append(RobotState.cmd.name.index(joint))
    

    "Main Loop"
    while not rospy.is_shutdown():


        "Get Latest Prediction"
        
        data = rospy.wait_for_message('prediction',Arrays)   
        Yexp = np.array(data.array).reshape(1,num_joints)

        cmd = deepcopy(RobotState.cmd)
        cmd.header.stamp = rospy.Time.now()
        position = list(cmd.position)

        "Actuation"
        for counter, index in enumerate(ind):
            position[index] = Yexp[0,counter]

        cmd.position = position
        jspub.publish(cmd)  
#        
        
        "Next"        
        rate.sleep()         

if __name__ == '__main__':
    try:
        actuate()
    except rospy.ROSInterruptException:
        pass
