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

from std_msgs.msg import Int32
from std_msgs.msg import Float64
from bwrobot.srv import *
from bwrobot.msg import *
   
def jitter(n, Y_init, deg = 5):
    max_rough=0.0174533
    pert = deg*max_rough * np.random.uniform(-1.,1.,(n,np.size(Y_init,1)))
    Y_start = Y_init + pert
    return Y_start

def initialize():

    cmd = JointState()
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)

    all_joint_names = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0',
    'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint',
    'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cmd.name = all_joint_names

    "Robot settings"
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    num_joints = len(joint_names)
    YStart = rospy.get_param('~YStart')
    YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)
    ind = []
    for joint in joint_names:
        ind.append(all_joint_names.index(joint))

    "Initialize Local Models"
    njit = rospy.get_param('~njit')
    deg = rospy.get_param('~degree', 5)

    YI = jitter(njit, YStart, deg)
    XI = np.empty([0,3])
    rospy.Rate(1).sleep()
    for y in YI:        
        cmd.header.stamp = rospy.Time.now()
        for counter, index in enumerate(ind):
            position[index] = y[counter]
        cmd.position = position
        jspub.publish(cmd)
        rospy.Rate(5).sleep()
        data = rospy.wait_for_message('experience',Point)
        XI = np.vstack((XI,np.array([data.x,data.y, data.z]).reshape(1,3)))
    return XI, YI, ind

def constructMsg(local):
       
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

def train():

    "ROS Settings"
    rospy.init_node('train_node')
    R = rospy.get_param('~train_pub_rate')
    rate = rospy.Rate(R)
    traintime = rospy.Publisher('traintime', Float64, queue_size=10)
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
    GPpub = rospy.Publisher('localGP',LocalGP,queue_size=10, latch = True)

    "Model Parameters"
    num_inducing = rospy.get_param('~num_inducing')
    w_gen = rospy.get_param('~wgen')
    d = rospy.get_param('~drift')

    # cmd = JointState()

    # all_joint_names = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0',
    # 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint',
    # 'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    # position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # cmd.name = all_joint_names

    # "Robot settings"
    # joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    # num_joints = len(joint_names)
    # YStart = rospy.get_param('~YStart')
    # YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)
    # ind = []
    # for joint in joint_names:
    #     ind.append(all_joint_names.index(joint))

    # "Initialize Local Models"
    # njit = rospy.get_param('~njit')
    # YI = jitter(njit, YStart, 5)
    # XI = np.empty([0,3])
    # rospy.Rate(1).sleep()
    # for y in YI:        
    #     cmd.header.stamp = rospy.Time.now()
    #     for counter, index in enumerate(ind):
    #         position[index] = y[counter]
    #     cmd.position = position
    #     jspub.publish(cmd)
    #     rospy.Rate(5).sleep()
    #     data = rospy.wait_for_message('experience',Point)
    #     XI = np.vstack((XI,np.array([data.x,data.y, data.z]).reshape(1,3)))


    "Initialize"
    XI,YI, ind = initialize()
    num_joints = np.size(YI,1)
    local = LocalModels(num_inducing, wgen = w_gen, xdim =3, ndim = num_joints*2)
    local.initializeF(XI,YI)

    LocMsg = constructMsg(local)
    GPpub.publish(LocMsg)
    
    Xtot = local.XI
    Ytot = local.YI
    Y = np.empty([1,num_joints])
    i = 1
    
    "Main Loop"
    while not rospy.is_shutdown():

        "Get Yexp"
        data = rospy.wait_for_message('joint_states',JointState) # should I use this vs prediction?
        for counter, index in enumerate(ind):
            Y[0,counter] = data.position[index]
        
        # Y = np.array(data.position).reshape(1,num_joints)      
        Yexp = local.encode_ang(Y)

        "Get Xexp"
        data = rospy.wait_for_message('experience',Point)
        Xexp = np.array([data.x,data.y,data.z]).reshape(1,3)
        
        t1 = time.time()

        "Training"
        if Xtot.shape[0] > 50:
            Xtot = np.delete(Xtot,0,0)
            Ytot = np.delete(Ytot,0,0)
            
        Xtot = np.vstack((Xtot,Xexp))
        Ytot = np.vstack((Ytot,Yexp))   
        
        if i % d == 0:
            ndrift = 10
            mdrift = local.doOSGPR(Xtot[-d:], Ytot[-d:], local.mdrift,local.num_inducing ,use_old_Z=False, driftZ = False)
            W = np.diag([1/(mdrift.kern.lengthscale[0]**2), 1/(mdrift.kern.lengthscale[1]**2)])  

            # mdrift = GPy.models.GPRegression(Xtot[-ndrift:], Ytot[-ndrift:], GPy.kern.RBF(local.xdim,ARD=True))
            # mdrift.optimize(messages = False)
            
            mkl = []
            for j in range(0, local.xdim):
                mkl.append(1/(mdrift.kern.lengthscale[j]**2))
                
            W = np.diag(mkl)

            local.W = W
            local.mdrift = mdrift


        local.partition(Xexp.reshape(len(Xexp),local.xdim),Yexp.reshape(len(Yexp),local.ndim))
        try:
            local.train()
        except:
            pass
        LocMsg = constructMsg(local)
        GPpub.publish(LocMsg)
        
        t2 = time.time() 
        
        traintime.publish(t2-t1)
        

        "Next"        
        i+=1
        rate.sleep()


class Trainer():
    def __init__(self, num_inducing, w_gen, x_dim, y_dim):
        self.solarGP = LocalModels(num_inducing, wgen = w_gen, xdim = x_dim, ndim = y_dim)
        self.Yexp = []
        self.Xexp = []
        self.Xtot = []
        self.Ytot = []
        rospy.loginfo("init")

    def x_callback(self, xdata):
        self.Xexp = np.array([xdata.x,xdata.y,xdata.z]).reshape(1,3)         

    def y_callback(self, ydata):
        Y = np.array(ydata.position).reshape(1,int(self.solarGP.ndim/2))        
        self.Yexp = self.solarGP.encode_ang(Y)

    def train(self,i,d,ndrift):
        if self.Xtot.shape[0] > 50:
            self.Xtot = np.delete(self.Xtot,0,0)
            self.Ytot = np.delete(self.Ytot,0,0)
            
        self.Xtot = np.vstack((self.Xtot,self.Xexp))
        self.Ytot = np.vstack((self.Ytot,self.Yexp))   
        
        if i % d == 0:
            # mdrift = local.doOSGPR(Xtot[-d:], Ytot[-d:], local.mdrift,local.num_inducing ,use_old_Z=False, driftZ = False)
            # W = np.diag([1/(mdrift.kern.lengthscale[0]**2), 1/(mdrift.kern.lengthscale[1]**2)])  
            # local.W = W
            # local.mdrift = mdrift

            mdrift = GPy.models.GPRegression(self.Xtot[-ndrift:], self.Ytot[-ndrift:], GPy.kern.RBF(self.solarGP.xdim,ARD=True))
            mdrift.optimize(messages = False)
            
            mkl = []
            for j in range(0, self.solarGP.xdim):
                mkl.append(1/(mdrift.kern.lengthscale[j]**2))
                
            W = np.diag(mkl)

            self.solarGP.W = W
            self.solarGP.mdrift = mdrift


        self.solarGP.partition(self.Xexp,self.Yexp)
        try:
            self.solarGP.train()
        except:
            pass


def train2():

    "ROS Settings"
    rospy.init_node('train_node')
    R = rospy.get_param('~train_pub_rate')
    rate = rospy.Rate(R)
    traintime = rospy.Publisher('traintime', Float64, queue_size=10)
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)
    GPpub = rospy.Publisher('localGP',LocalGP,queue_size=10)
    num_joints = rospy.get_param('~num_joints')

    cmd = JointState()
    for i in range(0, num_joints):
        cmd.name.append('joint' + str(i+1))

    "Initialize Local Models"
    njit = rospy.get_param('~njit')
    YStart = rospy.get_param('~YStart')
    YStart = np.array(np.deg2rad(YStart)).reshape(1,num_joints)
    num_inducing = rospy.get_param('~num_inducing')
    w_gen = rospy.get_param('~wgen')
    d = rospy.get_param('~drift')

    rospy.loginfo("Start Model Initialization")

    Model = Trainer(num_inducing, w_gen, 3, num_joints*2)
    rospy.loginfo("through init")

    YI = Model.solarGP.jitY(njit, YStart)
    XI = np.empty([0,3])
    rospy.loginfo("Finished jitter")

    for y in YI:        
        cmd.header.stamp = rospy.Time.now()
        cmd.position = y.tolist()
        jspub.publish(cmd)

        rospy.Rate(5).sleep()
        data = rospy.wait_for_message('experience',Point)
        rospy.loginfo("got experience")

        XI = np.vstack((XI,np.array([data.x,data.y, data.z]).reshape(1,3))) 
        # XI = np.vstack((XI, Model.Xexp))
    rospy.loginfo("Got initial points")

    Model.solarGP.initializeF(XI,YI)
    LocMsg = constructMsg(Model.solarGP)
    GPpub.publish(LocMsg)
    
    Model.Xtot = Model.solarGP.XI
    Model.Ytot = Model.solarGP.YI
    rospy.loginfo("Finished Model Initialization")

    rospy.Subscriber('joint_states',JointState,Model.x_callback)
    rospy.Subscriber('experience',Point,Model.y_callback)
    rospy.loginfo("Created Subscribers")

    i = 1
   
    "Main Loop"
    while not rospy.is_shutdown():
        
        t1 = time.time()
        
        Model.train(i,d,10)
        traintime.publish(t2-t1)

        LocMsg = constructMsg(Model.solarGP)
        GPpub.publish(LocMsg)
        
        t2 = time.time() 
        
        "Next"        
        i+=1
        rate.sleep()


if __name__ == '__main__':
    try:
        train()
    except rospy.ROSInterruptException:
        pass
