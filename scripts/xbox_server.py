#!/usr/bin/env python

import rospy
import numpy as np
from teleop_utils import xbox_teleop
        
def teleop():
    rospy.init_node('xbox_teleop_node')
    Teleoperator = xbox_teleop.XboxTel()
    R = rospy.get_param('~teleop_pub_rate', 100)

    rate = rospy.Rate(R)

    while not rospy.is_shutdown():
        Teleoperator.pub_pose.publish(Teleoperator.currentPose)
        Teleoperator.pub_pose_next.publish(Teleoperator.nextPose)
        Teleoperator.pub_path.publish(Teleoperator.path)
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
