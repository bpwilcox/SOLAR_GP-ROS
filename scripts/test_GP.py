import GPy
import matplotlib.pyplot as pl
from mpl_toolkits.mplot3d.axes3d import Axes3D
GPy.plotting.change_plotting_library('matplotlib')
import numpy as np
import time
from SOLAR_core import LocalModels
from copy import copy, deepcopy

n =250
X = np.linspace(-2*np.pi,2*np.pi,n).reshape(n,1)
Y = np.sin(X)

#"Base Sparse GP"
#model = GPy.models.SparseGPRegression(X,Y)
#model.optimize()
##model.plot()
#
##testX = np.linspace(-2.5*np.pi,2.5*np.pi,500).reshape(500,1)
##testY, _ = model.predict(testX)
##pl.plot(testX,testY)


#"Sequential Sparse GP"
#L = LocalModels(xdim = 1)
#i = 3
#a = X[0:i, 0].reshape(i,1)
#b = Y[0:i, 0].reshape(i,1)
#num_inducing = 50
#m = L.train_init(a, b, num_inducing)
#count = 0
#t1 = time.time()
#for x, y in zip(X[i:],Y[i:]):
#    m = L.doOSGPR(x.reshape(1,1),y.reshape(1,1),m, num_inducing, use_old_Z = False)
#    count+=1
##    print(count)
#    
#t2 = time.time()
#print('Total training time: ', t2-t1)
#print('Average training time per iteration: ', (t2-t1)/count)
#print('Average Rate: ', round(1/((t2-t1)/count),2), ' Hz')
#
#testX = np.linspace(-2*np.pi,2*np.pi,500).reshape(500,1)
#testY, _ = m.predict(testX)
#pl.plot(testX,testY)
#pl.plot(X,Y,'-.')


"SOLAR GP"
i = 30
num_inducing =10
M = LocalModels(num_inducing, xdim = 1, ndim = 1, wgen = 0.9)
XI = X[0:i, 0].reshape(i,1)
YI = Y[0:i, 0].reshape(i,1)
mkl = np.empty([M.xdim,1])
M.initializeF(XI,YI, False)
#print("init W:", M.W)
print("kern lengthscale: ", M.Models[0].kern.lengthscale[0])
num_models = M.M
print('initialized')
count = 0
t1 = time.time()
for x, y in zip(X[i:],Y[i:]):
    print(count)
#    if count == 2:
#        break
    if count % 1 == 0:    
        try:
            mdrift = M.doOSGPR(x.reshape(1,1),y.reshape(1,1), M.mdrift, num_inducing ,use_old_Z=False, driftZ = False)
            for j in range(0, M.xdim):
                mkl[j,0] = 1/(mdrift.kern.lengthscale[j]**2)
                
            W = np.diag(mkl)
#            print("W:", W)
            
            M.W = W
            M.mdrift = mdrift   
            num_models = M.M
        except:
            print("pass")
            pass
     
    M.partition(x.reshape(1,1),y.reshape(1,1))
    
    try:
        M.train()
    except:
        pass
    count +=1
t2 = time.time()
print('Total training time: ', t2-t1)
print('Average training time per iteration: ', (t2-t1)/count)
print('Average Rate: ', round(1/((t2-t1)/count),2), ' Hz')

testX = np.linspace(-2*np.pi,2*np.pi,500).reshape(500,1)    
testY, _ = M.prediction(testX)
pl.plot(testX,testY)
pl.plot(X,Y,'-.')


#"SOLAR GP Batch Mode"
#np.random.seed(22)
#i = 5
#num_inducing = 10
#
#M = LocalModels(num_inducing, xdim = 1, ndim = 1, wgen = 0.95)
#XI = X[0:i, 0].reshape(i,1)
#YI = Y[0:i, 0].reshape(i,1)
#mkl = np.empty([M.xdim,1])
#M.initializeF(XI,YI, False)
#print('initialized')
#
#mdrift = M.doOSGPR(X[i:],Y[i:], M.mdrift, num_inducing ,use_old_Z=False, driftZ = False)
#for j in range(0, M.xdim):
#    mkl[j,0] = 1/(mdrift.kern.lengthscale[j]**2)
#    
#W = np.diag(mkl)
#
#M.W = W
#M.mdrift = mdrift    
#  
#M.partition(X[i:],Y[i:])
#
#try:
#    M.train()
#except:
#    pass
#
#    
#testX = np.linspace(-2*np.pi,2*np.pi,500).reshape(500,1)    
#testY, _ = M.prediction(testX)
#pl.plot(testX,testY)
#pl.plot(X,Y,'-')
