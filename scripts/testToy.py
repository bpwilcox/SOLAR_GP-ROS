import GPy
import matplotlib.pyplot as pl
from mpl_toolkits.mplot3d.axes3d import Axes3D
GPy.plotting.change_plotting_library('matplotlib')
import numpy as np
import time
from SOLAR_core import LocalModels
from copy import copy, deepcopy
from sklearn.metrics import mean_squared_error
from GPy import likelihoods


def rmse(predictions, targets):
    MSE = mean_squared_error(targets, predictions)
#    MSE = np.mean((targets - predictions)**2)
    return np.sqrt(MSE)

def nmse(predictions, targets):

    MSE = mean_squared_error(targets, predictions)
    Variance = np.mean((targets - np.mean(targets))**2)
    return MSE/Variance

def variance(T):
    X = np.atleast_2d(T)
    if np.shape(X)[1] > 1:
        return np.mean(np.diag(np.cov(X)))
    else:
        return np.mean((X - np.mean(X))**2)
#    return np.mean(np.dot(X-Xm ,np.transpose(X-Xm)))

n =200
t = np.linspace(0,5,n).reshape(n,1)
#X = np.linspace(-2*np.pi,2*np.pi,n).reshape(n,1)
X = t
Y = np.exp(-t)*np.sin(2*np.pi*X)
#Y = np.sin(2*np.pi*X)
#Y = 2*X

#pl.plot(X,Y,'--')
#model




#
#"Base Sparse GP"
##pl.figure(1)
#pl.plot(X,Y,'--')
##
#num_inducing = 25
#Z = X[np.random.permutation(X.shape[0])[0:num_inducing], :]
#K  = GPy.kern.RBF(input_dim=1)
#
#
##model = GPy.core.SparseGP(X,Y,Z,K, likelihood = likelihoods.Gaussian())
#model = GPy.models.SparseGPRegression(X,Y,K,Z)
#model.optimize()
#print(model)
##model.plot()
###
##
##
##testX = np.linspace(-2.5*np.pi,2.5*np.pi,500).reshape(500,1)
#testY, _ = model.predict(X)
#pl.plot(X,testY)
##pl.plot(model.Z, np.zeros_like(model.Z), 'ro')
#mu, _ = model.predict(model.Z)
#pl.plot(model.Z, mu, 'o')

#pl.xlim([0, 5])
#pl.ylim([-1,1])

##
#"Sequential Sparse GP"
##pl.figure(2)
#pl.plot(X,Y,'--')
#
#
#L = LocalModels(xdim=1)
#i = 25
#num_inducing = 25
#a = X[0:i, :]
#b = Y[0:i, :]
#
##count = 0
#
#batchsize = 600
#size = len(X)
#m_stream = L.train_init(a, b, num_inducing)
##m_stream = L.train_init(a, b, num_inducing)
##m_stream = model
#
##print(m_stream)
##print(m_stream.Z)
##m_prev = deepcopy(m_stream
#done = False
#curr = i
##test_Y, _ = m_stream.predict(a)
##test_Y = np.atleast_2d(test_Y)
#
#while not done:
#    if curr >= size:
#        break
#        
#    new_X = X[curr:curr+batchsize,:]
#    new_Y = Y[curr:curr+batchsize,:]
#    
#    m_stream = L.doOSGPR(new_X, new_Y, m_stream, num_inducing, use_old_Z = True, fixZ = False, fixTheta = False)
#    
##    test_batch, _ = m_stream.predict(new_X)
##    print(np.shape(test_batch))
##    print(np.shape(test_Y))
##    test_Y = np.vstack((test_Y, np.atleast_2d(test_batch)))
#    print(curr)
#    curr += batchsize
#    
###    count+=1
#
##m_stream.optimize()
#
#testY_stream, _ = m_stream.predict(X)
#pl.plot(X,testY_stream)
#mu, _ = m_stream.predict(m_stream.Z)
#pl.plot(m_stream.Z, mu, 'o')
##pl.plot(X[:curr],test_Y)
#
##pl.plot(m_stream.Z, np.zeros_like(m_stream.Z), 'ro')
#
#pl.xlim([0, 5])
#pl.ylim([-1,1])
###print('num_inducing: ' + str(len(m_stream.Z)))
###print(m_stream.Z)
#print(m_stream)
#print(m_stream.gradient)

"SOLAR GP"
i = 25
num_inducing = 15
solar = LocalModels(num_inducing, xdim = 1, ndim = 1, wgen = 0.1)
XI = X[0:i, 0].reshape(i,1)
YI = Y[0:i, 0].reshape(i,1)
mkl = np.empty([solar.xdim,1])
solar.initializeF(XI,YI, False)

count = 0
t1 = time.time()
done = False
curr = i
size = len(X)
batchsize = 50
while not done:
    if curr  >= size:
        done = True
        break
    
    new_X = X[curr:curr+batchsize,:]
    new_Y = Y[curr:curr+batchsize,:]
    mdrift = solar.doOSGPR(new_X, new_Y, solar.mdrift, 25, use_old_Z = True)
    print(mdrift)
    mkl = []
    for j in range(0, solar.xdim):
        mkl.append(1/(solar.mdrift.kern.lengthscale[j]**2))
    
    W = np.diag(mkl)
    solar.W = W    
    solar.mdrift = mdrift
    solar.partition(new_X, new_Y)
    solar.train()
    print(curr)
    curr += batchsize
    count+=1
    
t2 = time.time()
print('Total training time: ', t2-t1)
print('Average training time per iteration: ', (t2-t1)/count)
print('Average Rate: ', round(1/((t2-t1)/count),2), ' Hz')

#testX = np.linspace(-2*np.pi,2*np.pi,500).reshape(500,1)    
Ypred, _ = solar.prediction(X)
pl.plot(X,Ypred)
pl.plot(X,Y,'--')
for model in solar.Models:
    mu, _ = model.predict(model.Z)
    pl.plot(model.Z, mu, 'o')
    
RMSE = rmse(Ypred, Y)
NMSE = nmse(Ypred, Y)
#print(RMSE)
print(NMSE)

#print(solar.LocalData)
