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
    

def decode_ang(q):
    d = int(np.size(q,1)/2)
    decoding = np.arctan2(q[:,:d], q[:,d:]).reshape(np.size(q,0),d)
    return decoding

class SolarPredictor():
    def __init__(self, topic):

        # self.TestStream =  xbox_teleop.XboxTel()
        self.model = LocalModels()
        self.pred_pub = rospy.Publisher('prediction', Arrays, queue_size=10)
        self.curX = []
        self.curPose = PoseStamped()
        rospy.Subscriber('localGP',LocalGP,self.model_callback)
        rospy.Subscriber(topic, EndpointState, self.x_callback, queue_size = 10)

    def x_callback(self, msg):
        self.curPose.header = msg.header
        self.curPose.header.frame_id = '/teleop'
        self.curPose.pose = msg.pose
        self.curX = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).reshape(1,3)

    def model_callback(self,LocMsg):
        
        W = LocMsg.W
        W = np.diag([W[0],W[1],W[2]])

        M = LocMsg.M
        LocalData = []
        Models = []
        xdim = LocMsg.xdim
        ndim = LocMsg.ndim

        for L in LocMsg.localGPs:
            
            X_loc = []
            X_loc.append(L.numloc)
            X_loc.append(L.numloc)
            X_loc.append(np.array(L.xmean).reshape(1,xdim))
            X_loc.append(np.array(L.ymean).reshape(1,ndim))
            X_loc.append(True)
            
            LocalData.append(X_loc)
            X = np.empty([0,xdim]) 
            Y = np.empty([0,ndim]) 
    
            Z = np.empty([0,xdim]) 
            Z_old = np.empty([0,xdim])
            mu_old = np.empty([0,ndim])
            Su_old = np.empty([0,len(L.Z_old)])
            Kaa_old = np.empty([0,len(L.Z_old)])
    
            kern = GPy.kern.RBF(xdim,ARD=True)
            kern.variance = L.kern_var
            kern.lengthscale = L.kern_lengthscale
            
            for x,y in zip(L.X,L.Y):
                X = np.vstack((X,np.array(x.array).reshape(1,xdim)))       
                Y = np.vstack((Y,np.array(y.array).reshape(1,ndim)))       
                           
            for z in L.Z:
                Z = np.vstack((Z,np.array(z.array).reshape(1,xdim)))
                
            for z,mu,su,ka in zip(L.Z_old, L.mu_old,L.Su_old,L.Kaa_old):
                Z_old = np.vstack((Z_old,np.array(z.array).reshape(1,xdim)))        
                mu_old = np.vstack((mu_old,np.array(mu.array).reshape(1,ndim)))        
                Su_old = np.vstack((Su_old,np.array(su.array).reshape(1,len(L.Z_old))))        
                Kaa_old = np.vstack((Kaa_old,np.array(ka.array).reshape(1,len(L.Z_old))))    
                
            m = osgpr_GPy.OSGPR_VFE(X, Y, kern, mu_old, Su_old, Kaa_old, Z_old, Z)    
            m.kern.variance = L.kern_var
            m.kern.lengthscale = np.array(L.kern_lengthscale)
            m.likelihood.variance = L.likelihood_var
            Models.append(m)
            
        local = LocalModels()
        local.W = W
        local.M = M
        local.LocalData = LocalData
        local.Models = Models
        local.xdim = xdim
        local.ndim = ndim
        self.model = local 

def pred():

    rospy.init_node('predict_node')
    R = rospy.get_param('~predict_pub_rate', 100)
    rate = rospy.Rate(R)
    arm = rospy.get_param('~arm', 'right')
    end_effector_topic = 'robot/limb/' + arm + '/endpoint_state'
    d = rospy.get_param('~max_distance', 1)
    Yexp = []
    Predictor = SolarPredictor(end_effector_topic)

    rospy.wait_for_service('set_teleop_pose')
    set_teleop_pose = rospy.ServiceProxy('set_teleop_pose', SetPose)

    rospy.wait_for_service('get_teleop')
    get_teleop = rospy.ServiceProxy('get_teleop', GetTeleop)
    teleop_state = GetTeleopResponse()

    rospy.wait_for_message('localGP',LocalGP)
    print("Ready")
    while not rospy.is_shutdown():
        
        teleop_state = get_teleop()

        if teleop_state.button3:
            set_teleop_pose(Predictor.curPose)

        if not teleop_state.button2:
            continue
        
        data = teleop_state.pose

        # if Predictor.TestStream.button3:
        #     set_teleop_pose(Predictor.curPose)

        # if not Predictor.TestStream.button2:
        #     continue

        # data = Predictor.TestStream.nextPose

        xnext = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z]).reshape(1,3)
        v = xnext - Predictor.curX
        norm_v = np.linalg.norm(v)
        
        if norm_v > d:
            u = v/norm_v
            xnext = Predictor.curX + d*u 

        Ypred, _ = Predictor.model.prediction(xnext, Y_prev = Yexp)
        Yexp = Ypred
        if np.isnan(Yexp).any():
            continue

        Y = decode_ang(Yexp).astype(float)
        
        Predictor.pred_pub.publish(Y[0,:].tolist())
        rate.sleep()     

if __name__ == '__main__':
    try:
        pred()
    except rospy.ROSInterruptException:
        pass
