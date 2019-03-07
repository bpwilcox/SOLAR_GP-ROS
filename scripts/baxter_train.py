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
from bwrobot.msg import *
import baxter_interface
from data_buffer import DataBuffer

def jitter(n, Y_init, deg = 5):
    max_rough=0.0174533
    pert = deg*max_rough * np.random.uniform(-1.,1.,(n,np.size(Y_init,1)))
    Y_start = Y_init + pert
    return Y_start

def initialize(limb, default_neutral = True):
    
    # Grab controllable joints
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    limb_joint_names = limb.joint_names()
    num_joints = len(joint_names)

    # Set robot to "default" position
    if default_neutral:
        limb.move_to_neutral()
        YStart = []
        for joint in joint_names:
            YStart.append(limb.joint_angle(joint))
    else:
        YStart = rospy.get_param('~YStart')
        angles = dict(zip(joint_names,np.deg2rad(YStart)))
        limb.move_to_joint_positions(angles)
        YStart = []
        for joint in joint_names:
            YStart.append(limb.joint_angle(joint))        

    "Initialize Local Models"
    YStart = np.array(YStart).reshape(1,num_joints)
    njit = rospy.get_param('~njit')
    deg = rospy.get_param('~degree', 5)
    YI = jitter(njit, YStart, deg)
    XI = np.empty([0,3])
    rospy.Rate(1).sleep()

    for y in YI:        
        limb.move_to_joint_positions(dict(zip(joint_names,y.tolist())))
        end_pose = limb.endpoint_pose()
        XI = np.vstack((XI,np.array([end_pose['position'].x, end_pose['position'].y, end_pose['position'].z]).reshape(1,3)))
    return XI, YI

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
    GPpub = rospy.Publisher('localGP',LocalGP,queue_size=10, latch = True)
    x_topic = rospy.get_param('~x_topic', 'robot/limb/right/endpoint_state')
    y_topic = rospy.get_param('~y_topic', 'robot/joint_states')
    buffer_duration = rospy.get_param('~buffer_duration', 0.1)
    buffer_size = rospy.get_param('~buffer_size', 100)
    joint_names = rospy.get_param('~joints', ['right_s0', 'right_s1', 'right_e1', 'right_w1'])
    njit = rospy.get_param('~njit')
    deg = rospy.get_param('~degree', 5)
    YStart = rospy.get_param('~YStart', [0,0,0,0])
   
    "Model Parameters"
    num_inducing = rospy.get_param('~num_inducing')
    w_gen = rospy.get_param('~wgen')
    d = rospy.get_param('~drift')

    # rospy.wait_for_service('move_joints')
    # move_joints = rospy.ServiceProxy('move_joints', MoveJoint)
    # start = MoveJointRequest()
    # start.joints.position = YStart
    # start.joints.name = joint_names
    # start.neutral = True
    # move_joints(start)

    # TrainData = DataBuffer(x_topic, y_topic, joint_names, 0.5, buffer_size)
    # rospy.wait_for_service('jitter')
    # jitter_pos = rospy.ServiceProxy('jitter', Jitter)
    # jitter_pos(njit, deg)

    # XI = np.asarray(TrainData.Xexp).reshape(len(TrainData.Xexp),3)
    # YI = np.asarray(TrainData.Yexp).reshape(len(TrainData.Yexp),len(joint_names))
    # TrainData.duration = buffer_duration
    # TrainData.clear()

    "Initialize"
    limb = baxter_interface.Limb('right')
    XI,YI = initialize(limb)
    num_joints = np.size(YI,1)
    local = LocalModels(num_inducing, wgen = w_gen, xdim =3, ndim = num_joints*2)
    local.initializeF(XI,YI)

    LocMsg = constructMsg(local)
    GPpub.publish(LocMsg)
    
    Xtot = local.XI
    Ytot = local.YI
    i = 1

    TrainData = DataBuffer(x_topic, y_topic, joint_names, buffer_duration, buffer_size)
    
    "Main Loop"
    while not rospy.is_shutdown():

        "Get Yexp"
        # Wait for prediction before training 
        # rospy.wait_for_message('prediction', Arrays)
        if not TrainData.Xexp:
            continue
        else:
            Xexp = np.asarray(TrainData.Xexp).reshape(len(TrainData.Xexp),3)
            Y = np.asarray(TrainData.Yexp).reshape(len(TrainData.Yexp),num_joints)
            Yexp = local.encode_ang(Y)

        TrainData.clear()
        
        # Y = []
        # for joint in joint_names:
        #     Y.append(limb.joint_angle(joint))
        
        # Y = np.array(Y).reshape(1,num_joints)      
        # Yexp = local.encode_ang(Y)

        # "Get Xexp"
        # end_pose = limb.endpoint_pose()
        # Xexp = np.array([end_pose['position'].x, end_pose['position'].y, end_pose['position'].z]).reshape(1,3)

        # "Training"
        # if Xtot.shape[0] > 50:
        #     Xtot = np.delete(Xtot,0,0)
        #     Ytot = np.delete(Ytot,0,0)
            
        # Xtot = np.vstack((Xtot,Xexp))
        # Ytot = np.vstack((Ytot,Yexp))

        t1 = time.time()
        
        if i % d == 0:
            ndrift = 10
            # mdrift = local.doOSGPR(Xtot[-d:], Ytot[-d:], local.mdrift,local.num_inducing ,use_old_Z=False, driftZ = False)
            mdrift = local.doOSGPR(Xexp, Yexp, local.mdrift,local.num_inducing ,use_old_Z=False, driftZ = False)
            # W = np.diag([1/(mdrift.kern.lengthscale[0]**2), 1/(mdrift.kern.lengthscale[1]**2)])  

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



if __name__ == '__main__':
    try:
        train()
    except rospy.ROSInterruptException:
        pass
