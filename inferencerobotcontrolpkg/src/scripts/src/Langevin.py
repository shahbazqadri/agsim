#!/usr/bin/env python
#imports
import sys
import numpy as np
import scipy.linalg as scl
import itertools
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, pi, log


class langevin_sampler(object):
    def __init__(self, x0, iterations, sampleiterations, Trajsteps, n, un):
        super(langevin_sampler, self).__init__()

        # parameters of the initial prior state distribution
        #self.priormean = [0.25, -0.25]
        self.x0 = x0
        self.kldiv     = []
        self.iterforplot = []
        self.iterations = iterations
        self.sampleiterations = sampleiterations
        self.Trajsteps = Trajsteps
        self.n  = n
        self.un = un
        self. Q = (0.0001**2)*np.eye(self.n)
        self.H  = np.eye(self.un)
        self.dt    = 0.001
        self.gamma = 0.55

    def langevin(self):
        Trajsteps = self.Trajsteps
        iterations = self.iterations
        sampleiterations = self.sampleiterations
        n = self.n

        self.y       = np.zeros(((iterations)*n,Trajsteps))
        self.y_sorted_state = np.zeros((iterations,Trajsteps))
        self.ymean   = np.zeros((n, Trajsteps))
        self.yvar    = np.zeros((n,Trajsteps))
        nabla_u = np.zeros((Trajsteps,iterations*n))
        kldiv   = self.kldiv
        iterforplot = self.iterforplot

        # Langevin update
        for k in range(0,iterations):
            print k
            # y indices
            k_p = k*n
            k_1 = (k+1)*n
            k_2 = (k+2)*n
            
            #self.y_k = np.random.normal(0, 1, size=(self.n*(self.Trajsteps-1),1))
            self.y_k = np.random.randn(self.n*(self.Trajsteps-1),1)
            # y_k
            for i in range(0, sampleiterations):
                
                self.y_tilde = np.concatenate((self.x0, self.y_k), axis= 0)

                # nabla U(y_k)
                nabla_u = self.get_nabla_u()

                #assert nabla_u[:,k_p:k_1].shape == y[:,k_p:k_1].shape, "nabla_u dimension error in langevin.langevin()"

            
                # y_{k+1}
                self.y_k = self.y_k - (self.dt/(0.05*i+1)**self.gamma)* nabla_u + sqrt(2*self.dt/(0.05*i+1)**self.gamma)* (np.random.randn(n*(Trajsteps-1),1))

            self.y[k_p:k_1,:] = np.concatenate((self.x0,self.y_k.reshape(Trajsteps-1,self.n).transpose()) , axis= 1)

            print("y= ", self.y[k_p:k_1,:])

        for j in range(0,self.n):
            for i in range(0, self.iterations):
                idx        = i*self.n + j
                self.y_sorted_state[i:i+1,:]  = self.y[idx:idx+1,:]

            self.ymean[j:j+1,:] =  np.mean(self.y_sorted_state, axis=0) 
            self.yvar[j:j+1, :] =  np.var(self.y_sorted_state, axis=0)

        return self.ymean, self.y, self.yvar


    def index(self, A_tilde):

        A11 = A_tilde[0:self.n,0:self.n]
        A12 = A_tilde[0:self.n,self.n:self.n*self.Trajsteps +1]
        A21 = A_tilde[self.n:self.n*self.Trajsteps +1, 0:self.n]
        A22 = A_tilde[self.n:self.n*self.Trajsteps +1, self.n:self.n*self.Trajsteps +1]

        return A11, A12, A21, A22

    def get_nabla_u(self):
       
        A_tilde, a_tilde, S_tilde = self.get_system_dynamics(self.n)
        R_tilde, r_tilde = self.get_cost_matrices(self.n)

        A11, A12, A21, A22 = self.index(A_tilde)
        S11, S12, S21, S22 = self.index(S_tilde)

        a11 = a_tilde[0:self.n]
        a21 = a_tilde[self.n:self.n*self.Trajsteps +1]

        nabla_u_1 = np.dot(A12.transpose(), np.dot(S11,np.dot(A11,self.x0))) + np.dot(A12.transpose(), np.dot(S11, np.dot(A12,self.y_k))) - np.dot(A12.transpose(),np.dot(S11, a11))
        nabla_u_2 = np.dot(A22.transpose(), np.dot(S22, np.dot(A21,self.x0))) + np.dot(A22.transpose(), np.dot(S22, np.dot(A22,self.y_k))) - np.dot(A22.transpose(), np.dot(S22,a21))
        nabla_u_3 = np.dot(R_tilde, self.y_k)
        
        nabla_u = nabla_u_1 + nabla_u_2 + nabla_u_3
        return nabla_u


    def get_f(self, xhat):

        s = xhat
        f = np.zeros(s.shape)
    
        return f

    def get_g(self, xhat):

        g = np.eye(xhat.shape[0])
            
        return g

    def get_system_dynamics(self,n):


        A_tilde = np.zeros((self.Trajsteps*n, self.Trajsteps*n))
        a_tilde = np.zeros((self.Trajsteps*n, 1))
        S_tilde = np.zeros((self.Trajsteps*n, self.Trajsteps*n))

        A_tilde[0:n,0:n] = np.eye(n)
        S_tilde[0:n,0:n] = 1e-10*np.eye(n)

        for t in range(1,self.Trajsteps):
         

            #indices
            tn = t*self.n
            t1 = (t -1)*self.n
            t2 = (t +1)*self.n

            A_tilde[tn:t2,t1:tn] = -np.eye(n)
            A_tilde[tn:t2,tn:t2] = np.eye(n)

            a_tilde[tn:t2] = self.dt*self.get_f(self.y_tilde[t1:tn])

        for t in range(1,self.Trajsteps):
         

            #indices
            tn = t*self.n
            t1 = (t -1)*self.n
            t2 = (t +1)*self.n

            g = self.get_g(self.y_tilde[tn:t2])
            S = self.Q + np.dot(g.transpose(), np.dot(np.linalg.inv(self.H), g))
            S_tilde[tn:t2,tn:t2] = np.linalg.inv(S)

            


        return A_tilde, a_tilde, S_tilde


    def get_cost_matrices(self,n):

        R_tilde = np.zeros(((self.Trajsteps-1)*n, (self.Trajsteps-1)*n))
        r_tilde = np.zeros(((self.Trajsteps-1)*n, 1))
        

   

        for t in range(0,self.Trajsteps-1):
         

            #indices
            tn = t*self.n
            t2 = (t +1)*self.n

           
            R_tilde[tn:t2,tn:t2] = np.eye(n)


        return R_tilde, r_tilde
        
    

    # KL divergence of two gaussians
    def get_kldivergence(self, currmean, currcov, prevmean,prevcov):

        return 0.5*( log(np.linalg.det(prevcov)/np.linalg.det(currcov)) - (currcov.shape)[0] + np.dot(np.dot((currmean - prevmean).transpose(), np.linalg.inv(prevcov)), (currmean - prevmean)) + np.trace(np.dot(np.linalg.inv(prevcov), currcov)))


def main():

    
    x0 = np.array([[-10.0], [12], [-2.58]])
    x0_tolist = list(itertools.chain.from_iterable(x0))
    iterations = 100
    sampleiterations = 30000
    Trajsteps  = 10
    n          = 3
    un         = 3

    Lsol = langevin_sampler(x0, iterations, sampleiterations, Trajsteps, n, un)
    yT, y, var  =  Lsol.langevin()
    print("yT= ", yT)
    print("sample variance=,", var)

    '''print iterforplot
    print kldiv'''
    np.savetxt('y_1D.csv', y, delimiter= ",")
    '''plt.plot(iterforplot, kldiv, linewidth= 1.5)
    plt.show()'''
    tforplot = np.arange(0,Trajsteps  )
    plt.figure()
    for i in range(0,n):
        plt.plot(tforplot, yT[i:i+1,:].transpose(), label= 'x_{i}'.format(i = i))
    plt.legend()
    plt.title('optimal trajectory for linear system with x_0 = [{x0}]'.format(x0 = x0_tolist))
    plt.show()

    for i in range(0,n):
        plt.plot(tforplot, var[i:i+1,:].transpose(), label= 'x_{i}'.format(i = i))
    plt.legend()
    plt.title('sample variance for linear system with x_0 = [{x0}]'.format(x0 = x0_tolist))
    plt.show()
if __name__=='__main__':
    main()
