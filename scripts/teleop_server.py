#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import Trajectory
from geometry_msgs.msg import Point
#from rospy_tutorials.srv import *
from bwrobot.srv import *
from bwrobot.msg import *
import RobotModels
from sensor_msgs.msg import JointState


# global x_tel
# global n
# n = 250
# r = 0.5
# #twolink = RobotModels.nLink2D(Links = [1,1])
# #YStart = np.array(np.deg2rad(YStart)).reshape(1,2)
# #YStart = np.array(np.deg2rad([0,45])).reshape(1,2)

# "Get starting joint positions (currently from launch)"
# YStart = rospy.get_param('~YStart')
# YStart = np.array(np.deg2rad(YStart)).reshape(1,2)

# "Get current task space position"
# jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
# cmd = JointState()
# cmd.name = ['joint1', 'joint2'] # can this be variable from urdf?         
# "Actuation"
# cmd.position = [YStart[0][0], YStart[0][1]]
# jspub.publish(cmd)   
# data = rospy.wait_for_message('experience',Point)
# XStart = np.array([data.x,data.y,data.z]).reshape(1,3)

# #data = rospy.wait_for_message('experience',Point)
# #XStart = np.array([data.x,data.y]).reshape(1,2)

# x0 = XStart[0][0]-r
# y0 = XStart[0][1]
# #x0 = -0.25
# #y0 = 1

# #    trajectory = Trajectory.Star2D(n,5,r,x0,y0)
# trajectory = Trajectory.Circle2D(n,r,x0,y0,arclength = 2*np.pi)
# #    trajectory = Trajectory.nPoly2D(n,4,r,x0,y0)
# #    trajectory = Trajectory.Spiral2D(n,0.6,0.1,x0,y0,6*np.pi)

# # x_tel = trajectory.xtest
# x_tel = np.column_stack((trajectory.xtest, np.zeros(np.size(trajectory.xtest,0))))

class TestTraj():
    
    def __init__(self, trajectory):
        
        self.x_tel = trajectory
        self.n = len(trajectory)
    
    def nextPoint(self,resp):   
        
        i = resp.i%self.n
        x = self.x_tel[i,0]
        y = self.x_tel[i,1]
        z = self.x_tel[i,2]

        return x,y,z        

# def nextPoint(resp):
    
#     i = resp.i%n

#     x = x_tel[i,0]
#     y = x_tel[i,1]
#     z = x_tel[i,2]
#     #z = float(resp.i)

#     #print([x,y])
#     return x,y,z
    
def tel():

    rospy.init_node('Teleop_server')

    num_joints = rospy.get_param('~num_joints')

    "Get starting joint positions (currently from launch)"
    YStart = rospy.get_param('~YStart')
    YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)

    "Get current task space position"
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
    cmd = JointState()
    for i in range(0, num_joints):
        cmd.name.append('joint' + str(i+1))    
    #cmd.name = ['joint1', 'joint2'] # can this be variable from urdf?  
    #cmd.name = ['joint1', 'joint2','joint3','joint4']    

    "Actuation"
    #cmd.position = [YStart[0][0], YStart[0][1]]
    cmd.position = YStart[0,:].tolist()
    jspub.publish(cmd)
    rospy.Rate(0.5).sleep()
    data = rospy.wait_for_message('experience',Point)
    XStart = np.array([data.x,data.y,data.z]).reshape(1,3)

    n = 400
    r = 0.5
    x0 = XStart[0][0]-r
    y0 = XStart[0][1]
    trajectory = Trajectory.Circle2D(n,r,x0,y0,arclength = 2*np.pi)
    x_tel = np.column_stack((trajectory.xtest, np.zeros(np.size(trajectory.xtest,0))))
    Test = TestTraj(x_tel)

    s = rospy.Service('teleop_next',EndEffector,Test.nextPoint)    
    rospy.spin()
        
    
if __name__ == '__main__':
    try:
        tel()
    except rospy.ROSInterruptException:
        pass
