from __future__ import absolute_import
#import tensorflow as tf
import numpy as np
import scipy as sp
import GPy
from GPy import Model
from GPy.core.gp import GP
from GPy.core.sparse_gp import SparseGP
from GPy import likelihoods
#from paramz import Param
from GPy.core.parameterization.variational import VariationalPosterior
from GPy.core.parameterization.param import Param
#from autograd.numpy import sqrt
from autograd import grad
#from ...util.linalg import tdot
#from ... import util
from GPy.util.linalg import tdot_numpy
#from numpy import fill_diagonal
import autograd.numpy as anp
import autograd.scipy as asp
from autograd.scipy.linalg import solve_triangular

#from gpflow.model import GPModel
#from gpflow.param import Param, DataHolder
#from gpflow.mean_functions import Zero
#from gpflow import likelihoods
#from gpflow._settings import settings
#from gpflow.densities import multivariate_normal
#from gpflow._settings import settings
#float_type = settings.dtypes.float_type
float_type = np.float64
#
#tf.zeros(tf.stack([tf.shape(X)[0], 1]), dtype=float_type)
#np.zeros(np.vstack((np.shape(X)[0],1)),dtype= float_type)



class OSGPR_VFE(GP):
    """
    Online Sparse Variational GP regression.
    
    Streaming Gaussian process approximations
    Thang D. Bui, Cuong V. Nguyen, Richard E. Turner
    NIPS 2017
    """

    def __init__(self, X, Y, kern, mu_old, Su_old, Kaa_old, Z_old, Z, mean_function=None):
        """
        X is a data matrix, size N x D
        Y is a data matrix, size N x R
        Z is a matrix of pseudo inputs, size M x D
        kern, mean_function are appropriate gpflow objects
        mu_old, Su_old are mean and covariance of old q(u)
        Z_old is the old inducing inputs
        This method only works with a Gaussian likelihood.
        """


        
#        X = X
#        Y=Y
        
        
        self.X = Param('input',X)
        self.Y = Param('output',Y)
        
        
        likelihood = likelihoods.Gaussian()
#        GPModel.__init__(self, X, Y, kern, likelihood, mean_function)
        GP.__init__(self, X, Y, kern, likelihood, mean_function, inference_method = GPy.inference.latent_function_inference.VarDTC())
#        GP.__init__(self, X, Y, kern, likelihood, mean_function)
     
#        SparseGP.__init__(self, X, Y, Z, kern, likelihood, mean_function, inference_method = GPy.inference.latent_function_inference.VarDTC())
#        SparseGP.__init__(self, X, Y, Z, kern, likelihood, mean_function, inference_method = None)
       
        self.Z = Param('inducing inputs',Z)       
        self.link_parameter(self.Z)
        self.mean_function = mean_function
        self.num_data = X.shape[0]
        self.num_latent = Y.shape[1]

        self.mu_old = mu_old
        self.M_old = Z_old.shape[0]
        self.Su_old = Su_old
        self.Kaa_old = Kaa_old
        self.Z_old = Z_old
        self.ARD = True
        self.grad_fun = grad(self.objective)
        
    def _build_common_terms(self):

        Mb = np.shape(self.Z)[0]
        Ma = self.M_old
#        jitter = settings.numerics.jitter_level
        jitter = 1e-4
        sigma2 = self.likelihood.variance
        sigma = np.sqrt(sigma2)

        Saa = self.Su_old
        ma = self.mu_old

        # a is old inducing points, b is new
        # f is training points
        # s is test points
        Kbf = self.kern.K(self.Z, self.X)
        Kbb = self.kern.K(self.Z) + np.eye(Mb, dtype=float_type) * jitter
#        print(self.kern.lengthscale)
#        print(self.Z)
#        print(np.linalg.eigvalsh(Saa))
        
        
        Kba = self.kern.K(self.Z, self.Z_old)
        Kaa_cur = self.kern.K(self.Z_old) + np.eye(Ma, dtype=float_type) * jitter
        Kaa = self.Kaa_old + np.eye(Ma, dtype=float_type) * jitter

#        err = self.Y - self.mean_function(self.X)
        err = self.Y 

        Sainv_ma = np.linalg.solve(Saa, ma)
        Sinv_y = self.Y / sigma2
        c1 = np.matmul(Kbf, Sinv_y)
        c2 = np.matmul(Kba, Sainv_ma)
        c = c1 + c2

        Lb = np.linalg.cholesky(Kbb)
#        print(Lb)
#        print(c)
        Lbinv_c = sp.linalg.solve_triangular(Lb, c, lower=True)
        Lbinv_Kba = sp.linalg.solve_triangular(Lb, Kba, lower=True)
        Lbinv_Kbf = sp.linalg.solve_triangular(Lb, Kbf, lower=True) / sigma
        d1 = np.matmul(Lbinv_Kbf, np.transpose(Lbinv_Kbf))

        LSa = np.linalg.cholesky(Saa)
        Kab_Lbinv = np.transpose(Lbinv_Kba)
        LSainv_Kab_Lbinv = sp.linalg.solve_triangular(
            LSa, Kab_Lbinv, lower=True)
        d2 = np.matmul(np.transpose(LSainv_Kab_Lbinv), LSainv_Kab_Lbinv)

        La = np.linalg.cholesky(Kaa)
        Lainv_Kab_Lbinv = sp.linalg.solve_triangular(
            La, Kab_Lbinv, lower=True)
        d3 = np.matmul(np.transpose(Lainv_Kab_Lbinv), Lainv_Kab_Lbinv)

        D = np.eye(Mb, dtype=float_type) + d1 + d2 - d3
        D = D + np.eye(Mb, dtype=float_type) * jitter
        
#        E = np.linalg.eigvalsh(D)
#        
#        print(np.any(np.linalg.eigvalsh(D) < 0))
#        print(np.min(E))
#        print(np.linalg.eigvalsh(D))

        LD = np.linalg.cholesky(D)
        

        LDinv_Lbinv_c = sp.linalg.solve_triangular(LD, Lbinv_c, lower=True)

        return (Kbf, Kba, Kaa, Kaa_cur, La, Kbb, Lb, D, LD,
                Lbinv_Kba, LDinv_Lbinv_c, err, d1)

#    def log_likelihood(self):
#        """
#        Construct a function to compute the bound on the marginal
#        likelihood. 
#        """
#        
##        print("calling log likelihood")
#        Mb = np.shape(self.Z)[0]
#        Ma = self.M_old
##        jitter = settings.numerics.jitter_level
#        jitter = 1e-4
#        sigma2 = self.likelihood.variance
#        sigma = np.sqrt(sigma2)
#        N = self.num_data
#
#        Saa = self.Su_old
#        ma = self.mu_old
#
#        # a is old inducing points, b is new
#        # f is training points
#        Kfdiag = self.kern.Kdiag(self.X)
#        (Kbf, Kba, Kaa, Kaa_cur, La, Kbb, Lb, D, LD,
#            Lbinv_Kba, LDinv_Lbinv_c, err, Qff) = self._build_common_terms()
#
#        LSa = np.linalg.cholesky(Saa)
#        Lainv_ma = sp.linalg.solve_triangular(LSa, ma, lower=True)
#
#        bound = 0
#        # constant term
#        bound = -0.5 * N * np.log(2 * np.pi)
#        # quadratic term
#        bound += -0.5 * np.sum(np.square(err)) / sigma2
#        # bound += -0.5 * tf.reduce_sum(ma * Sainv_ma)
#        bound += -0.5 * np.sum(np.square(Lainv_ma))
#        bound += 0.5 * np.sum(np.square(LDinv_Lbinv_c))
#        # log det term
#        bound += -0.5 * N * np.sum(np.log(sigma2))
#        bound += - np.sum(np.log(np.diag(LD)))
#
#        # delta 1: trace term
#        bound += -0.5 * np.sum(Kfdiag) / sigma2
#        bound += 0.5 * np.sum(np.diag(Qff))
#
#        # delta 2: a and b difference
#        bound += np.sum(np.log(np.diag(La)))
#        bound += - np.sum(np.log(np.diag(LSa)))
#
#        Kaadiff = Kaa_cur - np.matmul(np.transpose(Lbinv_Kba), Lbinv_Kba)
#        Sainv_Kaadiff = np.linalg.solve(Saa, Kaadiff)
#        Kainv_Kaadiff = np.linalg.solve(Kaa, Kaadiff)
#
#        bound += -0.5 * np.sum(
#            np.diag(Sainv_Kaadiff) - np.diag(Kainv_Kaadiff))
#        
#        return bound
    
    def log_likelihood(self):
#        print("calling log_likelihood")
        return self.objective(self.flatten_params())
    
    def predict(self, Xnew, full_cov=False):
        """
        Compute the mean and variance of the latent function at some new points
        Xnew. 
        """

        # jitter = settings.numerics.jitter_level
        jitter = 1e-4

        # a is old inducing points, b is new
        # f is training points
        # s is test points
        Kbs = self.kern.K(self.Z, Xnew)
        (Kbf, Kba, Kaa, Kaa_cur, La, Kbb, Lb, D, LD,
            Lbinv_Kba, LDinv_Lbinv_c, err, Qff) = self._build_common_terms()

        Lbinv_Kbs = sp.linalg.solve_triangular(Lb, Kbs, lower=True)
        LDinv_Lbinv_Kbs = sp.linalg.solve_triangular(LD, Lbinv_Kbs, lower=True)
        mean = np.matmul(np.transpose(LDinv_Lbinv_Kbs), LDinv_Lbinv_c)

        if full_cov:
            Kss = self.kern.K(Xnew) + jitter * np.eye(np.shape(Xnew)[0], dtype=float_type)
            var1 = Kss
            var2 = - np.matmul(np.transpose(Lbinv_Kbs), Lbinv_Kbs)
            var3 = np.matmul(np.transpose(LDinv_Lbinv_Kbs), LDinv_Lbinv_Kbs)
            var = var1 + var2 + var3
        else:
            var1 = self.kern.Kdiag(Xnew)
            var2 = -np.sum(np.square(Lbinv_Kbs), 0)
            var3 = np.sum(np.square(LDinv_Lbinv_Kbs), 0)
            var = var1 + var2 + var3

        return mean, var

    def parameters_changed(self):
#        print("parameters changed")
#        pass
        
#        _, _, self.grad_dict = \
#        self.inference_method.inference(self.kern, self.X, self.Z, self.likelihood,
#                                        self.Y_normalized, Y_metadata=self.Y_metadata,
#                                        mean_function=self.mean_function)
        
        self._log_marginal_likelihood = self.log_likelihood()
        
#        self._update_gradients()
        self._update_grads()
#
    def _build_objective_terms(self, Z, kern_variance, kern_lengthscale, noise_variance):

        Mb = anp.shape(Z)[0]
        Ma = self.M_old
        jitter = 1e-4
        sigma2 = noise_variance
        sigma = anp.sqrt(sigma2)

        Saa = self.Su_old
        ma = self.mu_old

        # a is old inducing points, b is new
        # f is training points
        # s is test points
#        print(Z)
#        print(kern_lengthscale)
        Kbf = self.K(kern_variance, kern_lengthscale, Z, self.X)
#        Kbb = self.rbf_kernel(kern_variance, kern_lengthscale, Z) + anp.eye(Mb, dtype=float_type) * jitter
        Kbb = self.K(kern_variance, kern_lengthscale, Z) + anp.eye(Mb, dtype=float_type) * jitter
#        print("GPy kernel function")
#        print(Kbb)
#        print("custom kernel function")
#        print(Kbb1)
#        Kbb = anp.dot(Z, Z.T)
#        print(np.linalg.eigvalsh(Saa))
#        print(Z)
        
        Kba = self.K(kern_variance, kern_lengthscale, Z, self.Z_old)
        Kaa_cur = self.K(kern_variance, kern_lengthscale, self.Z_old) + anp.eye(Ma, dtype=float_type) * jitter
        Kaa = self.Kaa_old + anp.eye(Ma, dtype=float_type) * jitter

#        err = self.Y - self.mean_function(self.X)
        err = self.Y 

        Sainv_ma = anp.linalg.solve(Saa, ma)
        Sinv_y = self.Y / sigma2
        c1 = anp.matmul(Kbf, Sinv_y)
        c2 = anp.matmul(Kba, Sainv_ma)
        c = c1 + c2

        Lb = anp.linalg.cholesky(Kbb)
        Lbinv_c = solve_triangular(Lb, c, lower=True)
        Lbinv_Kba = solve_triangular(Lb, Kba, lower=True)
        Lbinv_Kbf = solve_triangular(Lb, Kbf, lower=True) / sigma
        d1 = anp.matmul(Lbinv_Kbf, anp.transpose(Lbinv_Kbf))
#        d1 = Kbb
        
        
        
        LSa = anp.linalg.cholesky(Saa)
        Kab_Lbinv = anp.transpose(Lbinv_Kba)
        LSainv_Kab_Lbinv = solve_triangular(
            LSa, Kab_Lbinv, lower=True)
        d2 = anp.matmul(anp.transpose(LSainv_Kab_Lbinv), LSainv_Kab_Lbinv)

        La = anp.linalg.cholesky(Kaa)
        Lainv_Kab_Lbinv = solve_triangular(
            La, Kab_Lbinv, lower=True)
        d3 = anp.matmul(anp.transpose(Lainv_Kab_Lbinv), Lainv_Kab_Lbinv)

        D = anp.eye(Mb, dtype=float_type) + d1 + d2 - d3
        D = D + anp.eye(Mb, dtype=float_type) * jitter
        
        LD = anp.linalg.cholesky(D)
        

        LDinv_Lbinv_c = solve_triangular(LD, Lbinv_c, lower=True)

        return (Kbf, Kba, Kaa, Kaa_cur, La, Kbb, Lb, D, LD,
                Lbinv_Kba, LDinv_Lbinv_c, err, d1)

    def flatten_params(self):
        
        params = np.ndarray.flatten(self.Z) # Size [mxd] 
#        params = self.kern.lengthscale
        params = np.hstack((params, self.kern.lengthscale))
        params = np.hstack((params, self.kern.variance))
        params = np.hstack((params, self.likelihood.variance))
        
        return params

    def rbf_kernel(self, variance, lengthscale, x, xp = None):
        output_scale = anp.exp(variance)
        lengthscales = anp.exp(lengthscale)
        if xp is None:
            xp = x
        diffs = anp.expand_dims(x /lengthscales, 1) - anp.expand_dims(xp/lengthscales, 0)
        
        return output_scale * anp.exp(-0.5 * anp.sum(diffs**2, axis=2))
    
    # Kernel functionality needed for autograd
    def K_of_r(self, r, variance):
        return variance * anp.exp(-0.5 * r**2)

    def Kdiag(self, X, variance):
#        ret = np.empty(X.shape[0])
        ret = anp.ones(X.shape[0])*variance
#        print(X.shape[0])
#        print(ret)
#        ret[:] = variance
        return ret
    
    def K(self, variance, lengthscale, X, X2=None):
        """
        Kernel function applied on inputs X and X2.
        In the stationary case there is an inner function depending on the
        distances from X to X2, called r.
        K(X, X2) = K_of_r((X-X2)**2)
        """
        r = self._scaled_dist(lengthscale, X, X2)
#        print(np.shape(r))
        return self.K_of_r(r, variance)

    def _scaled_dist(self, lengthscale, X, X2=None):
        """
        Efficiently compute the scaled distance, r.
        ..math::
            r = \sqrt( \sum_{q=1}^Q (x_q - x'q)^2/l_q^2 )
        Note that if thre is only one lengthscale, l comes outside the sum. In
        this case we compute the unscaled distance first (in a separate
        function for caching) and divide by lengthscale afterwards
        """
#        print(X)
        if self.ARD:
            if X2 is not None:
                X2 = X2 / lengthscale
            return self._unscaled_dist(X/lengthscale, X2)
        else:
            return self._unscaled_dist(X, X2)/lengthscale
        
    def _unscaled_dist(self, X, X2=None):
        """
        Compute the Euclidean distance between each row of X and X2, or between
        each pair of rows of X if X2 is None.
        """
        #X, = self._slice_X(X)
        if X2 is None:
            Xsq = anp.sum(anp.square(X),1)
#            r2 = -2.*tdot_numpy(X) + (Xsq[:,None] + Xsq[None,:])
            
            r2 = -2.*anp.dot(X, X.T) + (Xsq[:,None] + Xsq[None,:])

#            diag.view(r2)[:,]= 0. # force diagnoal to be zero: sometime numerically a little negative
            r2 = r2 - anp.diag(anp.diag(r2))
#            fill_diagonal(r2,0)
            r2 = anp.clip(r2, 1e-100, anp.inf)
            return anp.sqrt(r2)
        else:
#            X2, = self._slice_X(X2)
            X1sq = anp.sum(anp.square(X),1)
            X2sq = anp.sum(anp.square(X2),1)
            r2 = -2.*anp.dot(X, X2.T) + (X1sq[:,None] + X2sq[None,:])
            r2 = anp.clip(r2, 1e-100, anp.inf)
            return anp.sqrt(r2)

    def objective(self, params):
        
        kern_variance = params[-2]
        kern_lengthscale = anp.array(params[-2-len(self.kern.lengthscale):-2])
        noise_variance = params[-1]
        Z = anp.array(params[:anp.size(self.Z)]).reshape(np.shape(self.Z))
        
        sigma2 = noise_variance
        N = self.num_data
        Saa = self.Su_old
        ma = self.mu_old

        # a is old inducing points, b is new
        # f is training points
        Kfdiag = self.Kdiag(self.X, kern_variance)
        (Kbf, Kba, Kaa, Kaa_cur, La, Kbb, Lb, D, LD,
            Lbinv_Kba, LDinv_Lbinv_c, err, Qff) = self._build_objective_terms(Z, kern_variance, kern_lengthscale, noise_variance)

        LSa = anp.linalg.cholesky(Saa)
        Lainv_ma = solve_triangular(LSa, ma, lower=True)

        bound = 0
        # constant term
        bound = -0.5 * N * anp.log(2 * anp.pi)
        # quadratic term
        bound += -0.5 * anp.sum(anp.square(err)) / sigma2
        bound += -0.5 * anp.sum(anp.square(Lainv_ma))
        bound += 0.5 * anp.sum(anp.square(LDinv_Lbinv_c))
#        # log det term
        bound += -0.5 * N * anp.sum(anp.log(sigma2))
        bound += - anp.sum(anp.log(anp.diag(LD)))
#
#        # delta 1: trace term
        bound += -0.5 * anp.sum(Kfdiag) / sigma2
        bound += 0.5 * anp.sum(anp.diag(Qff))
#
#        # delta 2: a and b difference
        bound += anp.sum(anp.log(anp.diag(La)))
        bound += - anp.sum(anp.log(anp.diag(LSa)))
#
        Kaadiff = Kaa_cur - anp.matmul(anp.transpose(Lbinv_Kba), Lbinv_Kba)
        Sainv_Kaadiff = anp.linalg.solve(Saa, Kaadiff)
        Kainv_Kaadiff = anp.linalg.solve(Kaa, Kaadiff)
#
        bound += -0.5 * anp.sum(
            anp.diag(Sainv_Kaadiff) - anp.diag(Kainv_Kaadiff))
                
        return bound


    def _update_grads(self):
#        print("updating gradients")
        gradients = self.grad_fun(self.flatten_params())
#        gradients[0] = 0
#        print(gradients)
#        self.likelihood.gradient = gradients[-1]
        self.likelihood.update_gradients(gradients[-1])
#        self.kern.lengthscale.gradient = gradients[-2-len(self.kern.lengthscale):-2]
#        self.kern.gradient = np.hstack((gradients[-2], gradients[-2-len(self.kern.lengthscale):-2]))
        self.kern.update_gradients_direct(gradients[-2], gradients[-2-len(self.kern.lengthscale):-2])
#        self.kern.variance.gradient = gradients[-2]
        self.Z.gradient = np.reshape(gradients[:np.size(self.Z)], np.shape(self.Z))
#        self.Z.gradient = gradients[:np.size(self.Z)]
        
#        self.likelihood.update_gradients(self.grad_dict['dL_dthetaL'])
#        #gradients wrt kernel
#        self.kern.update_gradients_diag(self.grad_dict['dL_dKdiag'], self.X)
#        kerngrad = self.kern.gradient.copy()
#        self.kern.update_gradients_full(self.grad_dict['dL_dKnm'], self.X, self.Z)
#        kerngrad += self.kern.gradient
#        self.kern.update_gradients_full(self.grad_dict['dL_dKmm'], self.Z, None)
#        self.kern.gradient += kerngrad
#        #gradients wrt Z
#        self.Z.gradient = self.kern.gradients_X(self.grad_dict['dL_dKmm'], self.Z)
#        self.Z.gradient += self.kern.gradients_X(self.grad_dict['dL_dKnm'].T, self.Z, self.X)
            
        self._Zgrad = self.Z.gradient.copy()


    def _update_gradients(self):
#        print("updating gradients")
        self.likelihood.update_gradients(self.grad_dict['dL_dthetaL'])
        if self.mean_function is not None:
            self.mean_function.update_gradients(self.grad_dict['dL_dm'], self.X)

        if isinstance(self.X, VariationalPosterior):
            #gradients wrt kernel
            dL_dKmm = self.grad_dict['dL_dKmm']
            self.kern.update_gradients_full(dL_dKmm, self.Z, None)
            kerngrad = self.kern.gradient.copy()
            self.kern.update_gradients_expectations(variational_posterior=self.X,
                                                    Z=self.Z,
                                                    dL_dpsi0=self.grad_dict['dL_dpsi0'],
                                                    dL_dpsi1=self.grad_dict['dL_dpsi1'],
                                                    dL_dpsi2=self.grad_dict['dL_dpsi2'])
            self.kern.gradient += kerngrad

            #gradients wrt Z
            self.Z.gradient = self.kern.gradients_X(dL_dKmm, self.Z)
            self.Z.gradient += self.kern.gradients_Z_expectations(
                               self.grad_dict['dL_dpsi0'],
                               self.grad_dict['dL_dpsi1'],
                               self.grad_dict['dL_dpsi2'],
                               Z=self.Z,
                               variational_posterior=self.X)
        else:
            #gradients wrt kernel
            self.kern.update_gradients_diag(self.grad_dict['dL_dKdiag'], self.X)
            kerngrad = self.kern.gradient.copy()
            self.kern.update_gradients_full(self.grad_dict['dL_dKnm'], self.X, self.Z)
            kerngrad += self.kern.gradient
            self.kern.update_gradients_full(self.grad_dict['dL_dKmm'], self.Z, None)
            self.kern.gradient += kerngrad
            #gradients wrt Z
            self.Z.gradient = self.kern.gradients_X(self.grad_dict['dL_dKmm'], self.Z)
            self.Z.gradient += self.kern.gradients_X(self.grad_dict['dL_dKnm'].T, self.Z, self.X)
            
#             #gradients wrt kernel
#            self.kern.update_gradients_diag(self.grad_dict['dL_dKdiag'], self.X)
#            kerngrad = self.kern.gradient.copy()
#            self.kern.update_gradients_full(self.grad_dict['dL_dKnm'], self.X, self.Z)
#            kerngrad += self.kern.gradient
#            self.kern.update_gradients_full(self.grad_dict['dL_dKmm'], self.Z, None)
#            self.kern.gradient += kerngrad
#            #gradients wrt Z
#            self.Z.gradient = self.kern.gradients_X(self.grad_dict['dL_dKmm'], self.Z)
#            self.Z.gradient += self.kern.gradients_X(self.grad_dict['dL_dKnm'].T, self.Z, self.X)
            
        self._Zgrad = self.Z.gradient.copy()
>>>>>>> autograd implementation

