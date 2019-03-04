#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from bwrobot.msg import Arrays
from bwrobot.srv import Jitter
from copy import deepcopy, copy
import baxter_interface


class JitRobot():
    def __init__(self, limb, names):

        self.limb = limb
        self.joint_names = names
        self.jit_service= rospy.Service('jitter',Jitter, self.jitter)
        self.prediction = rospy.Publisher('prediction', Arrays, queue_size=10)

    def jitter(self, req):

        Y = []
        for joint in self.joint_names:
            Y.append(self.limb.joint_angle(joint))
        
        Y = np.array(Y).reshape(1,len(self.joint_names))   
        pert = req.degree * 0.0174533 * np.random.uniform(-1.,1.,(req.num_jit, np.size(Y,1)))
        YI = Y + pert

        for y in YI:        
            self.limb.move_to_joint_positions(dict(zip(self.joint_names,y.tolist())))
            self.prediction.publish(y.tolist())

        return True


def jitter_server():

    rospy.init_node('jitter_node')

    "Robot settings"
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    limb = baxter_interface.Limb('right')
    limb.set_joint_position_speed(1.0)
    RobotState = JitRobot(limb, joint_names)

    rospy.spin()

if __name__ == '__main__':
    try:
        jitter_server()
    except rospy.ROSInterruptException:
        pass
