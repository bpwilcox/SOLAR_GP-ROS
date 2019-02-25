#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
import GPy
import time
import Trajectory
from TestData import TestTrajectory
import RobotModels
import osgpr_GPy
#from Local import LocalModels
from SOLAR_core import LocalModels
from sensor_msgs.msg import JointState
from teleop_utils.srv import GetPose
from teleop_utils import phantom_teleop
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from bwrobot.srv import *
from bwrobot.msg import *
"Prediction node also serves as a client node for an input"

#global local
#global woo
def teleop_client(i):
    rospy.wait_for_service('teleop_next')
    nextPoint = rospy.ServiceProxy('teleop_next',EndEffector)
    data = nextPoint(i)
    return data

def phantom_client():
    rospy.wait_for_service('teleop_pose')
    returnPose = rospy.ServiceProxy('teleop_pose',GetPose)
    data = returnPose()
    return data

def callback(LocMsg):
    local = deconstructMsg(LocMsg)
    return woo
    
    
def deconstructMsg(LocMsg):
    
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
#        kern_var = L.kern_var
#        kern_lengthscale = L.kern_lengthscale
        X = np.empty([0,xdim]) 
        Y = np.empty([0,ndim]) 

        Z = np.empty([0,xdim]) 
        Z_old = np.empty([0,xdim])
        mu_old = np.empty([0,ndim])
        Su_old = np.empty([0,len(L.Z_old)])
        Kaa_old = np.empty([0,len(L.Z_old)])

        kern = GPy.kern.RBF(3,ARD=True)
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
            
#        X = np.array(L.X).reshape(1,2)
#        Y = np.array(L.Y).reshape(1,2)
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
    
    
    return local

class GetLocal():
    def __init__(self, encode = True):
        self.local = LocalModels()
        self.encode_angles = encode
#        self.publisher = 
#        self.subscriber
        self.count = 0

    def callback(self,LocMsg):
        
        W = LocMsg.W
#        W = np.diag([1/(W[0]**2), 1/(W[1]**2)])  
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
    #        kern_var = L.kern_var
    #        kern_lengthscale = L.kern_lengthscale
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
                
    #        X = np.array(L.X).reshape(1,2)
    #        Y = np.array(L.Y).reshape(1,2)
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
#        print(local.W)        
        self.local = local    
        self.count+=1
        
def decode_ang(q):
    d = int(np.size(q,1)/2)
    decoding = np.arctan2(q[:,:d], q[:,d:]).reshape(np.size(q,0),d)
    return decoding

def predict():

    "ROS Settings"
    rospy.init_node('predict_node')
    prediction = rospy.Publisher('prediction', Arrays, queue_size=10)
        
    R = rospy.get_param('~predict_pub_rate')
    wait_for_train = rospy.get_param('~wait_for_train')
    rate = rospy.Rate(R)
    num_joints = rospy.get_param('~num_joints')
    
    YStart = rospy.get_param('~YStart')
    YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)
    i = 0
    Loc = GetLocal()    
    Yexp = Loc.local.encode_ang(YStart)
    
    Teleoperator = phantom_teleop.phantom_teleop()    
#    local2 = LocalModels()
    if not wait_for_train:
        LocMsg = rospy.wait_for_message('localGP',LocalGP)
        local = deconstructMsg(LocMsg)  
        Loc.local = local

    "Main Loop"
    while not rospy.is_shutdown():
        "Get next model"   
        if wait_for_train:
            LocMsg = rospy.wait_for_message('localGP',LocalGP)
            local = deconstructMsg(LocMsg)  
            Loc.local = local
            Loc.count +=1
        else:
            rospy.Subscriber('localGP',LocalGP,Loc.callback)

        "Grab Teleoperator command"
        print("get teleop")
        
        # data = teleop_client(i) 
        # xnext = np.array([data.x,data.y,data.z])
        
        # data = phantom_client() 
        # xnext = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
        data = Teleoperator.nextPose
        xnext = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])


        "Predict "
        if Loc.encode_angles:
            Ypred,_ = Loc.local.prediction(xnext.reshape(1,3),Y_prev = Yexp)
            Yexp = Ypred
            Y = decode_ang(Yexp).astype(float)
            
        else:
            Ypred,_ = Loc.local.prediction(xnext.reshape(1,3),Y_prev = Yexp)
            Ypost = np.vstack((Yexp,Ypred))   
            Ypost = np.unwrap(Ypost,axis=0)
            Yexp = Ypost[-1].reshape(1,2)
            Y = Yexp
        
        # Y = [float(Yexp[0][0]), float(Yexp[0][1])],float(Yexp[0][2]),float(Yexp[0][3])
        #prediction.publish(Y)

        prediction.publish(Y[0,:].tolist())
               
        "Next"        
        i+=1
        rate.sleep()         

if __name__ == '__main__':
    try:
        predict()
    except rospy.ROSInterruptException:
        pass
