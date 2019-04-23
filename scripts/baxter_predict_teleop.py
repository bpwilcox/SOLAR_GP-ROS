#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
import GPy
import time
import Trajectory
from TestData import TestTrajectory
import RobotModels
import osgpr_GPy
from SOLAR_core import LocalModels
from sensor_msgs.msg import JointState
from teleop_utils.srv import GetTeleop, GetTeleopResponse, SetPose
from teleop_utils import xbox_teleop
from std_msgs.msg import Float64
from bwrobot.srv import *
from bwrobot.msg import *
import baxter_interface
from copy import copy, deepcopy
from baxter_core_msgs.msg import EndpointState
from predictor import SolarPredictor

class BaxterPredictor(SolarPredictor):
    def __init__(self):
        arm = rospy.get_param('~arm', 'right')
        end_effector_topic = 'robot/limb/' + arm + '/endpoint_state'        
        super().__init__(end_effector_topic)        
    
    def on_teleop(self, state):
        if state.button3:
            self.set_teleop_pose(self.curPose)
        if not state.button2:
            return False
        return True

def pred():
    rospy.init_node('predict_node')

    Predictor = BaxterPredictor()
    Predictor.run()

if __name__ == '__main__':
    try:
        pred()
    except rospy.ROSInterruptException:
        pass
