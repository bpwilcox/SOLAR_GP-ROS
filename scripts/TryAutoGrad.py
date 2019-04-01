import autograd.numpy as np
from autograd import grad
from autograd.scipy.linalg import solve_triangular

def fun(params):


    A = np.array([params[0], params[1]])
    B = np.array(params[2:]).reshape(2,2)
    One = np.array([1,1]).reshape(1,2)
    V = np.dot(np.dot(One, B), One.T)
    value= A[0]*np.sin(2*np.pi*A[1]) + V
    return value

def fun2(params):
    K = np.array(params[:16]).reshape(4,4)
    a = np.linalg.cholesky(K)
    b = np.array(params[16:])
    x = solve_triangular(a,b, lower= True)
    
    return np.sum(x)

#gradient = grad(fun)
#B = np.array([1,2,3,4]).reshape(2,2)
#params = np.array([1.0,1.0])
#params = np.hstack((params, np.ndarray.flatten(B)))
##print(params)
#print(gradient(params))


a = np.array([[3, 0, 0, 0], [2, 1, 0, 0], [1, 0, 1, 0], [1, 1, 1, 1]])
K = np.dot(a, a.T)
b = np.array([4., 2., 4., 2.])
params = np.hstack((np.ndarray.flatten(K), b))

gradient = grad(fun2)
print(gradient(params))

#
#x = solve_triangular(a, b, lower=True)
#print(x)
#print(np.dot(a,x))
