#!/usr/bin/env python

import rospy
import numpy as np
from teleop_utils import xbox_teleop, phantom_teleop
import trajectory_sender

# def get_xbox_teleoperator():

# def get_phantom_teleoperator():

# def get_bag_teleoperator()

def teleop():
    rospy.init_node('teleop_node')
    R = rospy.get_param('~teleop_pub_rate', 100)
    teleop_device = rospy.get_param('~device', 'bag')
    filename = rospy.get_param('~trajectory_filename', 'first_bag.bag')

    # Available Teleop Modes
    # devices = ['xbox', 'phantom', 'bag']
    if teleop_device == 'xbox':
        Teleoperator = xbox_teleop.XboxTel(R)
    elif teleop_device == 'bag':
        Teleoperator = trajectory_sender.TrajectorySender(filename, R)

    rate = rospy.Rate(R)

    while not rospy.is_shutdown():
        Teleoperator.pub_pose.publish(Teleoperator.currentPose)
        Teleoperator.pub_path.publish(Teleoperator.path)
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
