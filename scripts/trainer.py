#!/usr/bin/env python

import rospy
import numpy as np
import GPy
from SOLAR_core import LocalModels

from bwrobot.srv import Jitter, SetNeutral
from bwrobot.msg import LocalGP, OSGPR_GP, Arrays
import baxter_interface
from data_buffer import DataBuffer
from std_msgs.msg import Float64
import time

class Solar_Trainer():
    
    def __init__(self, njit, degrees, num_inducing, wgen, use_old_Z = False):
        self.solar = []
        self.pub_solar = rospy.Publisher('solarGP', LocalGP,queue_size=10, latch = True)
        self.njit = njit
        self.degrees = degrees
        self.num_inducing = num_inducing
        self.wgen = wgen
        self.use_old_Z = use_old_Z
        self.joint_names = []
        self.TrainData = []
        self.rate = []
        self.stop = False
        self.pub_traintime = rospy.Publisher('traintime', Float64, queue_size=10)

        rospy.wait_for_service('set_neutral')
        self.set_neutral = rospy.ServiceProxy('set_neutral', SetNeutral)

        rospy.wait_for_service('jitter')
        self.jitter_init = rospy.ServiceProxy('jitter', Jitter)   

    def initialize(self):

        R = rospy.get_param('~train_pub_rate', 100)
        self.rate = rospy.Rate(R)
        self.buffer_duration = rospy.get_param('~buffer_duration', 0.1)
        self.buffer_size = rospy.get_param('~buffer_size', 500)
        self.setup_robot()
        
        XI,YI = self.jitter_robot()
        num_joints = np.size(YI,1)
        self.solar = LocalModels(self.num_inducing, wgen = self.wgen, xdim =3, ndim = num_joints*2)
        self.solar.initializeF(XI,YI)
        SolarMsg = self.constructMsg(self.solar)
        self.pub_solar.publish(SolarMsg)
        
        self.TrainData = DataBuffer(self.x_topic, self.y_topic, self.joint_names, self.buffer_duration, self.buffer_size)

    def setup_robot(self):
        print("Setup Robot not implemented")
        return False

    def jitter_robot(self):
        XI = []
        YI = []
        print("Jitter Robot not implemented")
        
#        self.set_neutral()
#        self.TrainData = DataBuffer(self.x_topic, self.y_topic, self.joint_names, self.buffer_duration, self.buffer_size)
#        self.jitter_init(self.njit, self.degrees)
#        
#        XI = np.asarray(self.TrainData.Xexp).reshape(len(self.TrainData.Xexp),3)
#        YI = np.asarray(self.TrainData.Yexp).reshape(len(self.TrainData.Yexp),len(self.joint_names))
#        rospy.loginfo("Number of initial points: %s", len(XI))
#        self.TrainData.clear()
        
        return XI, YI

    def jitter(self, n, Y_init, deg = 5):
        max_rough=0.0174533
        pert = deg*max_rough * np.random.uniform(-1.,1.,(n,np.size(Y_init,1)))
        Y_start = Y_init + pert
        return Y_start

    def constructMsg(self, local):
        
        LocMsg = LocalGP()
        L = []
        for count,m in enumerate(local.Models):
            GP = OSGPR_GP()
            GP.kern_var = m.kern.variance[0]
            GP.kern_lengthscale = np.array(m.kern.lengthscale) .tolist()   
            GP.likelihood_var = m.likelihood.variance[0]
            GP.xmean = local.LocalData[count][2][0].tolist()
            GP.ymean = local.LocalData[count][3][0].tolist()
            GP.numloc = local.LocalData[count][0]
            Z = np.array(m.Z)
            Z_old = np.array(m.Z_old)
            mu_old = np.array(m.mu_old)
            Su_old = np.array(m.Su_old)
            Kaa_old = np.array(m.Kaa_old)
            
            
            X_arr = []
            Y_arr = []
            Z_arr = []
            Z_old_arr = []
            mu_old_arr = []
            Su_old_arr = []
            Kaa_old_arr = []
    #        

            for j in range(0,np.shape(m.X)[0]):
                X_row = Arrays()
                Y_row = Arrays()
                X_row.array = np.array(m.X[j,:]).tolist()
                Y_row.array = np.array(m.Y[j,:]).tolist()
                X_arr.append(X_row)
                Y_arr.append(Y_row)
            
            for j in range(0,np.shape(Z)[0]):
                Z_row = Arrays()
                Z_row.array = Z[j,:].tolist()
                Z_arr.append(Z_row)
    #       
            for j in range(0,np.shape(Z_old)[0]):
                
                Z_old_row = Arrays()
                mu_old_row = Arrays()
                Su_old_row = Arrays()
                Kaa_old_row = Arrays()
                
                Z_old_row.array = Z_old[j,:].tolist()
    #            print(Z_old_row.array)
                mu_old_row.array = mu_old[j,:].tolist()
                Su_old_row.array = Su_old[j,:].tolist()
                Kaa_old_row.array = Kaa_old[j,:].tolist()
                
                Z_old_arr.append(Z_old_row)
                mu_old_arr.append(mu_old_row)
                Su_old_arr.append(Su_old_row)
                Kaa_old_arr.append(Kaa_old_row)            
    #            
            GP.X = X_arr
            GP.Y = Y_arr
            GP.Z = Z_arr
            GP.Z_old = Z_old_arr
            GP.mu_old = mu_old_arr
            GP.Su_old = Su_old_arr
            GP.Kaa_old = Kaa_old_arr
            
            L.append(GP)
            
        LocMsg.localGPs= L
    #    LocMsg.W = np.array(local.mdrift.kern.lengthscale).tolist() 
        LocMsg.W = local.W.diagonal().tolist()
        LocMsg.M = local.M
        LocMsg.xdim = local.xdim
        LocMsg.ndim = local.ndim
        
        
        return LocMsg

    def run(self):
        
        while not rospy.is_shutdown() and not self.stop:
            t1 = time.time()

            "Get Yexp"
            if not self.TrainData.Xexp:
                continue
            else:
                try:
                    Xexp = np.asarray(self.TrainData.Xexp).reshape(len(self.TrainData.Xexp),3)
                    Y = np.asarray(self.TrainData.Yexp).reshape(len(self.TrainData.Yexp),len(self.joint_names))
                    Yexp = self.solar.encode_ang(Y)
                except:
                    continue

            self.TrainData.clear()
            try:
                mdrift = self.solar.doOSGPR(Xexp, Yexp, self.solar.mdrift, 100 ,use_old_Z = True, driftZ = False)
                mkl = []
                for j in range(0, self.solar.xdim):
                    mkl.append(1/(mdrift.kern.lengthscale[j]**2))
                    
                W = np.diag(mkl)
                self.solar.W = W
                self.solar.mdrift = mdrift
            except:
                pass

            self.solar.partition(Xexp.reshape(len(Xexp),self.solar.xdim),Yexp.reshape(len(Yexp),self.solar.ndim))
            try:
                self.solar.train()
            except:
                pass
            LocMsg = self.constructMsg(self.solar)
            self.pub_solar.publish(LocMsg)  
            t2 = time.time()
            self.pub_traintime.publish(t2-t1)
            self.rate.sleep()
            