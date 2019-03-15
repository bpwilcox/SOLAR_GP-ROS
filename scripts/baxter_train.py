#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
import GPy
import time
import Trajectory
from TestData import TestTrajectory
import RobotModels
from SOLAR_core import LocalModels

from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
from bwrobot.srv import Jitter, MoveJoint, MoveJointRequest
from bwrobot.msg import LocalGP, OSGPR_GP, Arrays
import baxter_interface
from data_buffer import DataBuffer
from trainer import Solar_Trainer


def train():
    
    rospy.init_node('train_node')
    "Model Parameters"
    njit = rospy.get_param('~njit', 25)
    deg = rospy.get_param('~degree', 3)
    num_inducing = rospy.get_param('~num_inducing', 25)
    w_gen = rospy.get_param('~wgen', 0.975)
    
    Trainer = Solar_Trainer(njit, deg, num_inducing, w_gen, False)
    Trainer.initialize()
    Trainer.run()

if __name__ == '__main__':
    try:
        train()
    except rospy.ROSInterruptException:
        pass
