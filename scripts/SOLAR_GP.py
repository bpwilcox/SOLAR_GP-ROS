import GPy
import matplotlib.pyplot as pl
from mpl_toolkits.mplot3d.axes3d import Axes3D
GPy.plotting.change_plotting_library('matplotlib')
import numpy as np
import time
from SOLAR_core import LocalModels
from copy import copy, deepcopy
#from sklearn.metrics import mean_squared_error

#%matplotlib inline
#%matplotlib qt5
#np.random.seed(0)


class SOLAR_GP():
    
    def __init__(self, robot, test, model = []):
        
        self.robot = robot
        self.test = test
        self.model = model
        self.xpath = []
        self.ypath = []

    def make_plot(self, Q, elevation, azimuth, test_trajectory= True, real_trajectory = True, save = False, name = 'test'):
        "Plot robot"
        if self.encode_angles:
            Q = self.decode_ang(Q)
        ax = self.robot.plot3D(Q, elevation, azimuth) 
        njit = len(self.model.XI)
        "Plot test trajectory and actual trajectory"
        if test_trajectory:
            ax.plot3D(self.test.xtest[:,0], self.test.xtest[:,1], self.test.xtest[:,2],'-')
        if real_trajectory:
            ax.plot3D(self.xpath[njit:,0], self.xpath[njit:,1], self.xpath[njit:,2])
        
        "Save Plots"
        if save:     
            fname = 'Figures/' + str(name) 
            pl.savefig(fname)
            
        pl.show(block=True)
    
        #pl.cla()        
    
    def decode_ang(self, q):
        d = int(np.size(q,1)/2)
        decoding = np.arctan2(q[:,:d], q[:,d:]).reshape(np.size(q,0),d)
        return decoding
    
    def runTest(self, init = True, njit = 25, num_inducing = 10, wgen = 0.9, drift = 1, encode_angles = False, train_flag = True, partition_flag = True, loop_trajectory = False, show_plots = True, save_plots = False, elev = 30, azim = 30):
        
        X_test = self.test.xtest
        
        self.encode_angles = encode_angles
        
        "Initialize Local Models"
        if init:
            self.model = LocalModels(num_inducing, wgen, robot = self.robot)
            self.model.initialize(njit, self.robot.currentY, self.encode_angles)

        self.xpath = self.model.XI # stack of experiences
        self.ypath = self.model.YI

        "Timing"
        tc = np.zeros([len(X_test)-1,1])

        if self.encode_angles:
            Yexp = self.model.encode_ang(self.robot.currentY)
        else:
            Yexp = self.robot.currentY
        i = 0 
        Error = np.empty([])
        keep = len(X_test) + njit
        batchsize = 2
        "Main Loop"
        while i < len(X_test):
            "Plotting"
            print("test input: " + str(i))        
            if i==250: #stop early condition
                break
            elif i%10==0: # create plot after n iterations
                self.make_plot(Yexp[-1,:].reshape(1,self.model.ndim), elev, azim, name = str(i), save = save_plots)
            
            t1 = time.time()
            
            new_X = X_test[i:i+batchsize,:]
            Xexp = np.empty([0, self.model.xdim])
            " Predict "
            if self.encode_angles:
                Ypred,var = self.model.prediction(new_X.reshape(batchsize,self.model.xdim))
                Yexp = Ypred
                for row in Yexp:
#                    print(row)
                    Xp, _ = self.robot.fkin(self.decode_ang(row.reshape(1,self.model.ndim)))
                    Xexp = np.vstack((Xexp, Xp))
                
            else:
                Ypred,var = self.model.prediction(new_X.reshape(batchsize,self.model.xdim))
                Ypost = np.vstack((self.ypath[-2:],Ypred))
                Ypost = np.unwrap(Ypost,axis=0) # unwrap angles to be unbounded         
                Yexp = Ypost[-1].reshape(1,self.model.ndim)
                Xexp, _ = self.robot.fkin(Yexp)

#            if self.encode_angles:
#                Ypred,var = self.model.mdrift.predict(X_test[i].reshape(1,self.model.xdim))
#                Yexp = Ypred
#                Xexp, _ = self.robot.fkin(self.decode_ang(Yexp))
#                
#            else:
#                Ypred,var = self.model.prediction(X_test[i].reshape(1,self.model.xdim),Y_prev = self.ypath[-2:])
#                Ypost = np.vstack((self.ypath[-2:],Ypred))
#                Ypost = np.unwrap(Ypost,axis=0) # unwrap angles to be unbounded         
#                Yexp = Ypost[-1].reshape(1,self.model.ndim)
#                Xexp, _ = self.robot.fkin(Yexp)                
            
            self.xpath = np.vstack((self.xpath,Xexp))
            self.ypath = np.vstack((self.ypath,Yexp))   
            if self.xpath.shape[0] > keep-1:
                self.xpath = self.xpath[-(keep-1):,:]
                self.ypath = self.ypath[-(keep-1):,:]
             
            " Train 'drift' GP"
            if i % drift == 0:
                mdrift = self.model.doOSGPR(self.xpath[-drift:], self.ypath[-drift:], self.model.mdrift,num_inducing ,use_old_Z=True)
                
                mkl = []
                for j in range(0, self.model.xdim):
                    mkl.append(1/(mdrift.kern.lengthscale[j]**2))
                W = np.diag(mkl)
                
                self.model.W = np.copy(W)
                self.model.mdrift = mdrift    
              
            "Partition experience "    
            self.model.partition(Xexp.reshape(len(Xexp),self.model.xdim),Yexp.reshape(len(Yexp),self.model.ndim))
        #    self.model.train()
            " Train Local Models "
#            try:
            self.model.train()
#            except:
#                pass
            
        
                   
            t2 = time.time()              
            tc[i-1]= t2-t1
            print(t2-t1)
            error = np.linalg.norm(X_test[i]-Xexp) 
            Error = np.hstack((Error,error))  
            i +=batchsize
            
            "Loop trajectory"
            if loop_trajectory:
                if i >= len(X_test):
        #            train_flag = False # Turn off local model training
        #            partition_flag = False   # Turn off partitioning  
                    i = 0

        # Plot Final
        self.make_plot(Yexp[-1,:].reshape(1,self.model.ndim), elev, azim, name = str(i), save = save_plots)


# =============================================================================
# 
#         # Plot final support points
#         for i in range(0,len(self.model.Models),1):
#             pl.figure(1)
#             pl.plot(self.model.LocalData[i][2][0,0],self.model.LocalData[i][2][0,1],'k+') # model centers
#             pl.plot(self.model.Models[i].Z[:,0],self.model.Models[i].Z[:,1],'.')
#         
#         pl.show(block=True)
#         #pl.savefig(fname)
# =============================================================================
