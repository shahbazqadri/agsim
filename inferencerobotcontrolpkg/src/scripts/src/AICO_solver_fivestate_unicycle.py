#!/usr/bin/env python
import sys
import numpy as np
from math import sin, cos, exp, sqrt
import itertools
import matplotlib.pyplot as plt


class AICO_solver(object):
    def __init__(self, model, xo, xf, dx, dy):
        super(AICO_solver, self).__init__()

        self.model = model #name of the model as input argument to the class
        self.T =1500 #Total number of Timesteps
        self.t = 0 

        # Model Initialization
        if self.model == 'linear':
            self.n = 3 #dimension of state
            self.un = 3 #dimension of the input
            self.x0 = np.array([[-100],[20],[3.14]]) #initial state
            self.xT =[0,0,0] #goal state
            self.trajectorytolerance = 1e-5 #trajectory tolerance

        elif self.model == 'unicycle':
            self.n = 5 #dimension of state
            self.vel = 100 # linear velocity of the unicycle model in m/s
            self.un = 1 #dimension of the input
            self.x0 =  xo#np.array([[-100],[20],[3.14],[self.vel],[0]]) #initial state
            self.xT =xf#[0,0] #goal state
            self.omega_limit = [1, -8] # angular velocity limits for the unicycle model
            self.h  = 0.001 # stepsize
            self.trajectorytolerance = 1e-4 #trajectory tolerance
        
        else:
            pass

        self.x0_tolist = list(itertools.chain.from_iterable(self.x0)) #initial state as list for plotting
        self.H = np.eye(self.un) #control matrix
        self.alpha = 0.9 #convergence rate
        self.tolerance = 1e-1 #1e-5 #belief tolerance

        # mean and covariance of the forward message
        self.s = np.zeros((self.n, self.T +1))
        self.S = np.zeros((self.n, self.n * (self.T+1)))
        self.Sinv = np.zeros((self.n, self.n * (self.T+1)))
        ## Initialization
        self.s[:, 0:1] = self.x0
        self.S[:,0:self.n] = 1e-10*np.eye(self.n)
        self.Sinv[:,0:self.n] = 1e10*np.eye(self.n)
        if self.model == 'unicycle':
            self.S[4:5,4:5] = 1
            self.Sinv[4:5,4:5] = 1

        # mean and covariance of the backward message
        self.Vinv = np.zeros((self.n, self.n * (self.T+1)))
        self.V = np.zeros((self.n, self.n * (self.T+1)))
        self.v = np.zeros((self.n, self.T +1))

        # feedback controller
        self.o = np.zeros((self.un, self.T +1))
        self.O = np.zeros((self.un, self.n * (self.T+1)))
        self.u   = np.zeros((self.un, self.T +1))

        #mean and covariance of the belief distribution
        self.belief    = np.zeros((self.n, self.T +1))
        self.beliefcov = np.zeros((self.n, self.n * (self.T+1)))
        ## Initialization
        self.belief[:, 0:1] = self.x0
        self.beliefcov[:,0:self.n] = 1e-10*np.eye(self.n)

        # mean and covariance of the task message
        self.R = np.zeros((self.n, self.n * (self.T+1)))
        self.r = np.zeros((self.n, self.T +1))
        ##Initialization 
        self.R[:,0:self.n] = np.eye(self.n)

        # Inferred Trajectory 
        self.X      = np.zeros((self.n, self.T +1))
        #self.data      = np.zeros((self.K*self.n, self.T +1))
        self.X[:,0:1] = self.x0

        '''# Trajectory inferred using control inference
        self.Xu      = np.zeros((self.n, self.T +1))
        #self.datau      = np.zeros((self.K*self.n, self.T +1))
        self.Xu[:,0:1] = self.x0'''

        # Trajectory validation using omega
        self.Xomega      = np.zeros((3, self.T +1))
        #self.dataomega      = np.zeros((self.K*self.n, self.T +1))
        self.Xomega[:,0:1] = self.x0[0:3]

        # System matrices in the form x_{t+1} = A_t * x_t + a_t + B_t * u_t
        self.A = np.zeros((self.n, self.n * (self.T+1)))
        self.B = np.zeros((self.n, self.un * (self.T+1)))
        self.a = np.zeros((self.n, self.T +1))

        ## Initialization
        if self.model == 'linear':
            self.Q = 0.0001*np.eye(self.n) # process noise covariance
            self.A[:,0:self.n] = np.eye(self.n) 
            self.B[:,0:self.un] = np.eye(self.un)
            self.a[:, 0:1] = np.zeros((self.n,1))
        elif self.model == 'unicycle':
            self.Q = 0.0001*np.eye(self.n)*(self.h) # process noise covariance
            self.Q[4:5,4:5] = 1e-10*(self.h)
            Jx= np.array([[0,0,-self.x0[3]*sin(self.x0[2]),cos(self.x0[2]),0],[0,0,self.x0[3]*cos(self.x0[2]),sin(self.x0[2]),0],[0,0,0,0,1],[0,0,0,0,0],[0,0,0,0,0]])
            self.A[:,0:self.n] = np.eye(self.n) +  self.h*Jx
            self.a[:, 0:1] = self.h * np.array([[self.x0[3]*cos(self.x0[2])],[self.x0[3]*sin(self.x0[2])],[self.x0[4]], [0], [0]]) - self.h*np.dot(Jx,self.x0)
            self.B[:,0:self.un] = self.h * np.array([[0],[0],[0], [0], [1]])

    # Task message 
    def compute_task_message(self,xhat):

        '''To compute the cost matrices R, r in cost = x^T R x - 2 r^T x + u^T H u
            Task message: \mu_{z_t\rightarrow x_t}(x_t) &= P(z_t|x_t) =  \mathcal{N}[x_t|r_t, R_t]
            where,
            R_t &= \sum_{i=1}^n \rho_{i,t} \hat{J}_i^T \hat{J}_i\\
            r_t &= \sum_{i=1}^n \rho_{i,t} \hat{J}_i^T(y_{i,t}^* - \phi_i(\hat{x}_t) + \hat{J}_i\hat{x}_t)'''

        t = self.t #current timestep
        #indices
        tn = self.t*self.n
        t1 = (self.t+1)*self.n
       
        # Linear case
        if self.model == 'linear':
            #precisions for task variables
            prec1 = [1e-4, 1e2] #task space position variable
            self.R[:,tn:t1] = np.eye(self.n)
            self.r[:, t:t+1] = np.zeros((self.n,1))
        # Unicycle model
        elif self.model == 'unicycle':
            #precisions for task variables
            prec1 = [1e-4, 1e2]  # for task space position 
            prec2 = [1e13, 1e13] # for linear velocity constraint
            prec3 = [1e13, 1e13] # for angular velocity constraint 

            xhat_tolist = list(itertools.chain.from_iterable(xhat))  # convert xhat to list
            xhatold_tolist = list(itertools.chain.from_iterable(self.Xold[:,t:t+1]))

            # Jacobian of the map from state-space to task-space 
            J1 =np.array([[1,0,0, 0, 0],[0,1,0, 0,0]])
            J2 =np.array([[0,0,0,1,0]])

            # Desired Task space variables
            y1 = np.array([[self.xT[0]],[self.xT[1]]])
            y2 = np.array([[self.vel]])
            y3 = np.array([[0]])
            y4 = np.array([[0]])

            # Map from state-space to task-space y = phi(xhat)
            phi1 = np.array([[xhat_tolist[0]],[xhat_tolist[1]]])
            phi2 = np.array([xhat_tolist[3]])

            # impositng cost for violation of constrain on angular velocity
            if xhat[4] > self.omega_limit[0]:
                phi3 = np.array([xhat_tolist[4]-self.omega_limit[0]])
                J3   = np.array([[0,0,0,0,1]])
            elif xhat[4] < self.omega_limit[1]:
                phi3 = np.array([self.omega_limit[1] - xhat_tolist[4]])
                J3   = np.array([[0,0,0,0,-1]])
            else:
                phi3 = np.array([0])
                J3   = np.array([[0,0,0,0,0]])

            phi4 = np.array([xhat_tolist[2]- xhatold_tolist[2] - self.h* xhat_tolist[4]])
            J4   = np.array([[0,0,1,0,self.h]])
            
            # Task message updates
            if t < self.T:
                self.R[:,tn:t1] = prec1[0]*np.dot(J1.transpose(),J1) + prec2[0]*np.dot(J2.transpose(), J2) + prec3[0]*np.dot(J3.transpose(), J3) 

                self.r[:, t:t+1]=  prec1[0]*np.dot(J1.transpose(),(y1 - phi1 + np.dot(J1, xhat))) + prec2[0]*np.dot(J2.transpose(),(y2 - phi2 + np.dot(J2,xhat))) + prec3[0]*np.dot(J3.transpose(),(y3 - phi3 + np.dot(J3,xhat))) 
            else:
                self.R[:,tn:t1] =   prec1[1]*np.dot(J1.transpose(),J1) + prec2[1]*np.dot(J2.transpose(), J2) + prec3[1]*np.dot(J3.transpose(), J3) 
                self.r[:, t:t+1]=   prec1[1]*np.dot(J1.transpose(),(y1 - phi1 + np.dot(J1, xhat))) + prec2[0]*np.dot(J2.transpose(),(y2 - phi2 + np.dot(J2,xhat))) + prec3[1]*np.dot(J3.transpose(),(y3 - phi3 + np.dot(J3,xhat)))
        else:
            pass

   
    # Get system dynamics
    def get_system_dynamics(self,xhat):
    
        t = self.t # current timestep
        #indices
        tn = self.t*self.n
        tun = self.t*self.un
        t1 = (self.t+1)*self.n
        tu1= (self.t+1)*self.un
        
        # Linear model
        if self.model == 'linear':
            # Update system matrices for the current state estimate
            self.A[:,tn:t1] = np.eye(self.n)
            self.a[:, t:t+1] = np.zeros((self.n,1))
            self.B[:,tun:tu1] = np.eye(self.n)

        # Unicycle model
        elif self.model == 'unicycle':
            #velocity disturbances
            self.mu = dx# 10.0#sin(xhat[0]) # x-direction in m/s
            self.gamma= dy #7.5 #cos(xhat) # y-direction in m/s
            
            # Update system matrices for the current state estimate
            Jx= np.array([[0,0,-xhat[3]*sin(xhat[2]),cos(xhat[2]),0],[0,0,xhat[3]*cos(xhat[2]),sin(xhat[2]),0],[0,0,0,0,1],[0,0,0,0,0],[0,0,0,0,0]])
            self.A[:,tn:t1] = np.eye(self.n) +  self.h*Jx
            self.a[:, t:t+1] = self.h * np.array([[xhat[3]*cos(xhat[2]) + self.mu],[xhat[3]*sin(xhat[2]) + self.gamma],[xhat[4]], [0], [0]]) - self.h*np.dot(Jx,xhat)
            self.B[:,tun:tu1] = self.h * np.array([[0],[0],[0], [0], [1]])

        else:
            pass
            
        
    # Forward message
    def compute_forward_message(self):

        ''' compute the forward message for the current timestep
        forward message: \mu_{x_{t-1}\rightarrow x_t}(x_t) = \mathcal{N}(x_t|s_t, S_t))
        
        s_t = a_{t-1} + A_{t-1}(S_{t-1}^{-1} + R_{t-1})^{-1}(S_{t-1}^{-1}s_{t-1}+ r_{t-1})
        S_t = Q + B_{t-1}H^{-1}B_{t-1}^T + A_{t-1}(S_{t-1}^{-1} + R_{t-1})^{-1}A_{t-1}^{T}'''

        t = self.t # current timestep

        # indices
        tn = self.t*self.n
        tun = self.t*self.un
        t1 = (self.t -1)*self.n
        t2 = (self.t +1)*self.n
        tu1 = (self.t -1)*self.un
        tu2 = (self.t +1)*self.un

        # distribution of forward message for previous timestep
        sp = self.s[:,t-1:t]
        Sp = self.S[:,t1:tn]
        Sinvp = self.Sinv[:,t1:tn]

        # system matrices for the previous timestep
        Ap = self.A[:,t1:tn]
        Bp = self.B[:,tu1:tun]
        ap = self.a[:,t:t+1]

        # task/cost matrices for the previous timestep
        rp = self.r[:,t-1:t]
        Rp = self.R[:,t1:tn]
       
        #process noise covariance
        Q = self.Q
        #control matrix
        H = self.H

        # Forward message update for the current timestep
        Sbar = np.linalg.inv(Sinvp+Rp)
        W    = Q + np.dot(np.dot(Bp,np.linalg.inv(H)),Bp.transpose())

        s = ap + np.dot(Ap, np.dot(Sbar,(np.dot(Sinvp,sp)+rp)))

        S =  W + np.dot(np.dot(Ap, np.linalg.inv(Sinvp+Rp)), Ap.transpose())

        # update self
        self.s[:,t:t+1] = s
        self.S[:,tn:t2] = S
        self.Sinv[:,tn:t2] = np.linalg.inv(S)


    # Backward message
    def compute_backward_message(self):

        ''' compute the backward message for the current timestep
        backward message: \mu_{x_{t+1}\rightarrow x_t}(x_t) = \mathcal{N}(x_t|\nu_t, V_t)
        
        \nu_t = -A_t^{-1}a_t + A_t^{-1}(V_{t+1}^{-1} + R_{t+1})^{-1})(V_{t+1}^{-1}\nu_{t+1} + r_{t+1})
        V_t = A_t^{-1}[Q + B_tH^{-1}B_t^{T} + (V_{t+1}^{-1} + R_{t+1})^{-1}]A_t^{-T}'''

        t = self.t #current timestep

        #indices
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        t2 = (self.t +2)*self.n
        tun = self.t*self.un
        tu1 = (self.t +1)*self.un

        #distribution of backward message for the next timestep
        vn = self.v[:,t+1:t+2]
        Vn = self.V[:,t1:t2]
        Vinvn = self.Vinv[:,t1:t2]

        # system matrices for the current timestep
        Ac = self.A[:,tn:t1]
        Bc = self.B[:,tun:tu1]
        ac = self.a[:,t:t+1]

        # task/cost matrices for the next timestep
        rn = self.r[:,t+1:t+2]
        Rn = self.R[:,t1:t2]

        #process noise covariance 
        Q = self.Q
        #control matrix
        H = self.H

        # Backward message update for the current timestep
        Vbar = np.linalg.inv(Vinvn +Rn)
        W    = Q + np.dot(np.dot(Bc,np.linalg.inv(H)),Bc.transpose())

        v = -np.dot(np.linalg.inv(Ac), ac) + np.dot(np.linalg.inv(Ac),np.dot(Vbar,(np.dot(Vinvn,vn)+rn))) 
        V = np.dot(np.linalg.inv(Ac), np.dot(W+Vbar, np.linalg.inv(Ac.transpose())))
        

        #update self
        self.v[:,t:t+1] = v 
        self.V[:,tn:t1] = V
        self.Vinv[:,tn:t1] = np.linalg.inv(V)

    # Belief computation
    def compute_belief(self):
        
        '''compute the belief distribution
        belief:  b(x_t) = \mu_{x_{t-1}\rightarrow x_t}(x_t) * \mu_{x_{t+1}\rightarrow x_t}(x_t) * \mu_{z_t\rightarrow x_t}(x_t) = \mathcal{N}(b_t, B_t)
        
        b_t = (S_t^{-1} + V_t^{-1} + R_t)^{-1}(S_t^{-1}s_t + V_t^{-1}v_t + r_t)
        B_t = (S_t^{-1} + V_t^{-1} + R_t)^{-1}'''

        t = self.t #current timestep

        #indices
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        
        # Forward message distribution for the current timestep
        s    = self.s[:,t:t+1]
        Sinv = np.linalg.inv(self.S[:,tn:t1])

        # Backward message distribution for the current timestep
        v    = self.v[:,t:t+1]
        Vinv = self.Vinv[:,tn:t1]

        # task/cost matrices for the current timestep
        R    = self.R[:,tn:t1]
        r    = self.r[:,t:t+1]
        
        # belief distribution, self update 
        self.bcov  = Sinv + Vinv + R
        self.bmean = np.dot(np.linalg.inv(self.bcov),(np.dot(Sinv,s) + np.dot(Vinv,v) + r))
        self.beliefcov[:,tn:t1] = self.bcov
        #import pdb; pdb.set_trace()
        
        
    def compute_feedback_controller(self):
        
        '''Infer controls from the messages and implement an optimal feedback controller
        Control distribution conditioned on the current state:
            p(u_t|x_t) &= p(x_t,u_t)/p(x_t)
                = \mathcal{N}\Bigg(u_t|M_t^{-1} \bigg( B_t^T (Q_t + \bar{V}_{t+1}^{-1})^{-1}\big(\bar{V}_{t+1}^{-1}\bar{\nu}_{t+1} - A_t x_t - a_t\big) \bigg), M_t^{-1}\Bigg)
        
        where, M_t = B_t^T(Q_t + \bar{V}_{t+1}^{-1})^{-1}B_t
               \bar{V}_{t+1} = V_{t+1}^{-1} + R_{t+1}
               \bar{\nu}_{t+1} = V_{t+1}^{-1}\nu_{t+1} + r_{t+1}
               
        optimal feedback controller:
            u_t &= o_t + O_t x_t

       such that, o_t &= M_t^{-1}\bigg( B_t^T (Q_t + \bar{V}_{t+1}^{-1})^{-1}\bar{V}_{t+1}^{-1}\bar{\nu}_{t+1}  - B_t^T (Q_t + \bar{V}_{t+1}^{-1})^{-1} a_t \bigg)\\
                  O_t &= -M_t^{-1}B_t^T (Q_t + \bar{V}_{t+1}^{-1})^{-1}A_t
                  
        state propogation:
            x_{t+1} = A_t x_t + a_t + B_t u_t'''

        t = self.t #current timestep 

        #indices
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        t2 = (self.t +2)*self.n
        tun = self.t*self.un
        tu1 = (self.t +1)*self.un

        # Backward message distribution of the next timestep
        vn = self.v[:,t+1:t+2]
        Vn = self.V[:,t1:t2]
        Vinvn = self.Vinv[:,t1:t2]

        # system matrices of the current timestep
        Ac = self.A[:,tn:t1]
        Bc = self.B[:,tun:tu1]
        ac = self.a[:,t:t+1]

        # task/cost matrices for the next timestep
        rn = self.r[:,t+1:t+2]
        Rn = self.R[:,t1:t2]

        # process noise covariance
        Q = self.Q

        #control matrix
        H = self.H

        # conditional control distribution variables
        Vbar  = Vinvn + Rn
        vbar  = np.dot(Vinvn, vn) + rn
        Vstar = np.linalg.inv(Q + np.linalg.inv(Vbar))
        M     = np.dot(np.dot(Bc.transpose(),Vstar),Bc) + H

        # optimal feedback controller for the current timestep
        o1  = np.dot(Bc.transpose(), np.dot(Vstar, np.dot(np.linalg.inv(Vbar),vbar)))
        o2  = np.dot(Bc.transpose(), np.dot(Vstar, ac))
        self.ot   = np.dot(np.linalg.inv(M),(o1-o2))

        self.Ot   = -np.dot(np.linalg.inv(M),np.dot(Bc.transpose(),np.dot(Vstar, Ac)))

        if self.k == 0:
            self.o[:,t:t+1]   = self.ot
            #print(tn,t1)
            self.O[:,tn:t1] = self.Ot


        else:
            self.o[:,t:t+1] = (1-self.alpha)* self.o[:,t:t+1] + self.alpha* self.ot
            self.O[:,tn:t1] = (1-self.alpha)*self.O[:,tn:t1] + self.alpha*self.Ot

        self.u[:,t:t+1] = self.o[:,t:t+1]  + np.dot(self.O[:,tn:t1], self.Xu[:,t:t+1])
    
    
    # update using the current estimate
    def update(self, xhat):
        self.get_system_dynamics(xhat) # update system dynamics
        self.compute_task_message(xhat) # update task messages
        self.compute_belief() # compute belief

    def beliefconvergence(self, xhat):
        t = self.t #current timestep
        conditionmet = False #belief convergence variable

        #belief convergence condition
        if self.k >0:
            if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                return
            else:
                while not conditionmet:
                    xhat = (1-self.alpha)* xhat+ self.alpha*self.bmean
                    self.update(xhat)

                    if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                        self.belief[:,t:t+1] = self.bmean
                        conditionmet = True
                                


    # AICO algorithm
    def AICO(self):
        T = self.T #Total timesteps of the trajectory
        self.Xold  = np.zeros((self.n, self.T +1)) #trajectory in the previous iteration
        error = np.full(self.X.shape, 1e-2) #error in the state trajectory in the consecutive iterations
        #for k in range(0,self.K):
        k = 0 # iterator
        # run through iterations until convergence
        while abs(np.sum(error)) > self.trajectorytolerance and k < 1000: #trajectory convergence condition

            print('k =',k )
            self.k = k # current iteration

            if k>0:
                #update trajectory in previous iteration 
                self.Xold[:,0:-1] = self.X [:,0:-1]   

            # forward pass        
            for t in range(1, T+1):
                self.t = t #current timestep
                self.compute_forward_message() #compute forward message distribution

                # current state estimate
                if k == 0:
                    xhat = self.s[:,t:t+1]

                else:
                    xhat = (1-self.alpha)* self.X[:,t:t+1] + self.alpha*self.belief[:,t:t+1]


                self.update(xhat) #update using current state estimate
                
                self.beliefconvergence(xhat)#belief convergence
             
                # update self
                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean
            

            # backward pass
            for t in range(T, 0, -1):
                           
                self.t = t #current timestep

               
                if t  == T:  # At the final timestep
                    # The forward message is equal to the backward message
                    self.v[:,t:t+1] = self.belief[:,t:t+1]
                    self.V[:,(self.t )*self.n:(self.t +1 )*self.n] = self.S[:,(self.t )*self.n:(self.t +1 )*self.n]
                    self.Vinv[:,(self.t)*self.n:(self.t +1)*self.n] = np.linalg.inv(self.S[:,(self.t )*self.n:(self.t +1 )*self.n])
                    
            
                else:
                    self.compute_backward_message() #compute the backward message

                # state estimate at the current timestep    
                xhat = (1-self.alpha)* self.X[:,t:t+1] + self.alpha*self.belief[:,t:t+1]
                
                '''if t<T:
                    self.compute_feedback_controller()'''
                
                self.update(xhat) # update using current state estimate
                
                self.beliefconvergence(xhat) #belief convergence
                
                #update self
                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean   

            # compute the trajectory using inferred controls
            '''for t in range(0,T):
                self.t = t
                tn = self.t*self.n
                t1 = (self.t +1)*self.n
                t2 = (self.t +2)*self.n
                tun = self.t*self.un
                tu1 = (self.t +1)*self.un
                Ac = self.A[:,tn:t1]
                Bc = self.B[:,tun:tu1]
                ac = self.a[:,t:t+1]
                self.compute_feedback_controller()

                self.Xu[:,t+1:t+2] = np.dot(Ac,self.Xu[:,t:t+1]) + ac + np.dot(Bc, self.omega[:,t:t+1])'''

            for t in range(0,T):
                self.t = t
                tn = self.t*self.n
                t1 = (self.t +1)*self.n
                t2 = (self.t +2)*self.n
                tun = self.t*self.un
                tu1 = (self.t +1)*self.un
                Ac = self.A[0:3,tn:tn+3]
                Bc = self.B[0:3,tun:tu1]
                ac = self.a[0:3,t:t+1]

                X = list(itertools.chain.from_iterable(self.X[:,t:t+1]))
                Xo = list(itertools.chain.from_iterable(self.Xomega[:,t:t+1]))
                X1 = Xo[0] + self.h*self.vel*cos(Xo[2]) + self.h*self.mu
                X2 = Xo[1] + self.h*self.vel*sin(Xo[2]) + self.h*self.gamma
                X3  = Xo[2] + self.h*X[4]

                #self.Xomega[:,t+1:t+2] = np.dot(Ac,self.Xomega[:,t:t+1]) + ac + np.dot(Bc, self.X[4:5,t:t+1])
                self.Xomega[:,t+1:t+2] = np.array([[X1], [X2], [X3]])
                
            # check difference in the trajectory in consecutive iterations
            if k ==0:
                diff = self.X - self.Xold
               
            else:
                diff = self.X - self.Xold

            # sum of differences at each timestep    
            error = np.sum(diff[:,0:-1], axis= 1)    
            print('diff=', diff) #print diff to screen
            k = k+1 #increment iterator
        
        return self.X[0:-1]   
            
    def plot(self):
            if self.model == 'unicycle':
                T = self.T
                vxactual = []
                vyactual = []
                vactual  = []
                tactualplot = []
                for l in range(1,T+1):
                    tactualplot.append(l)
                    vxactual.append((self.X[0,l:l+1] - self.X[0,l-1:l])/self.h)
                    vyactual.append((self.X[1,l:l+1] - self.X[1,l-1:l])/self.h)
                    vactual.append(sqrt(vxactual[l-1]**2 + vyactual[l-1]**2) )
                plt.figure()
                plt.plot(tactualplot, vxactual, label= 'vxcalc', marker= '.', color= 'red', linewidth= '2')
                plt.plot(tactualplot,vyactual, label= 'vycalc ', marker= '.', color= 'green', linewidth= '2')
                plt.plot(tactualplot, vactual, label=  'vcalc' , marker= '.', color= 'blue', linewidth= '2')
                plt.xlabel('timestep')
                plt.ylabel('v')
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v = self.vel, T= self.T, h= self.h))
                plt.legend()
                #plt.yticks([-12,-10,-5,0,5,10,12])
                plt.show()
                plt.figure()
                plt.plot(tactualplot, self.X[0,1:T+1], label= 'x', marker= '.', color= 'red', linewidth= '2')
                plt.plot(tactualplot,self.X[1,1:T+1], label= 'y', marker= '.', color= 'green', linewidth= '2')
                #plt.plot(tactualplot, self.X[2,1:T+1], label=  '$\theta$' , marker= '.', color= 'blue', linewidth= '2')
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v= self.vel, T= self.T, h= self.h))
                plt.legend()
                plt.show()
                plt.figure()
                plt.plot(tactualplot, self.X[2,1:T+1], label=  '$\\theta$' , marker= '.', color= 'blue', linewidth= '2')
                plt.plot(tactualplot, self.Xomega[2,1:T+1], label=  '$\\theta_\omega$' , marker= '.', color= 'red', linewidth= '2')
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v= self.vel, T= self.T, h= self.h))
                plt.legend()
                plt.show()
                plt.figure()
                plt.plot(tactualplot[0:-2], self.X[4,1:T-1], label= '$\omega$', marker= '.', color= 'orange', linewidth= '2')
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec, limit= [{limit0}, {limit1}]'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v = self.vel, T= self.T, h= self.h, limit0= self.omega_limit[0], limit1= self.omega_limit[1]))
                plt.legend()
                plt.show()
                plt.figure()
                plt.plot(self.X[0,:], self.X[1,:], label = 'xy-trajectory', color= 'green', linewidth = '2')
                plt.plot(self.Xomega[0,:], self.Xomega[1,:], label = 'xy-trajectory using $\omega$', color= 'blue', linewidth = '2')
                plt.legend()
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v= self.vel, T= self.T, h= self.h))
                plt.show()
                plt.plot(self.X[0,:]- self.Xomega[0,:], label = 'errorx', color= 'darkcyan', linewidth = '2')                 
                plt.plot(self.X[1,:]-self.Xomega[1,:], label = 'errory', color= 'red', linewidth = '2')
                plt.legend()
                plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= self.x0_tolist[0], x02= self.x0_tolist[1], x03= self.x0_tolist[2], x04= self.x0_tolist[3], x05= self.x0_tolist[4], xg1= self.xT[0], xg2= self.xT[1], v= self.vel, T= self.T, h= self.h))
                plt.show()
    
    
       


# main

xo = np.array([[-100],[20],[3.14],[100],[0]])
xf = [-10,100]
dx = 0.0
dy = 0.0
sol1 = AICO_solver('unicycle', xo, xf, dx, dy)
print('running AICO')
X_withoutdist = sol1.AICO()
sol1.plot()
dx = 10.0
dy = 7.5
sol2 = AICO_solver('unicycle', xo, xf, dx, dy)
print('running AICO')
X_withdist = sol2.AICO()
sol2.plot()
plt.figure()
plt.plot(X_withoutdist[0,:], X_withoutdist[1,:], label = 'xy-trajectory without disturbance', color= 'green', linewidth = '2')
plt.plot(self.Xomega[0,:], self.Xomega[1,:], label = 'xy-trajectory with disturbance $\mu = 10.0$, $\gamma = 7.5$', color= 'blue', linewidth = '2')
plt.legend()
plt.title('x0 = [{x01}, {x02}, {x03}, {x04}, {x05}] m, xg = [{xg1}, {xg2}] m,\n velocity = {v} m/s, Timesteps = {T}, timestep= {h} sec'.format(x01= sol.x0_tolist[0], x02= sol.x0_tolist[1], x03= sol.x0_tolist[2], x04= sol.x0_tolist[3], x05= sol.x0_tolist[4], xg1= sol.xT[0], xg2= sol.xT[1], v= sol.vel, T= sol.T, h= sol.h))
plt.show()
#np.savetxt('data_AICO_unicycle_withdisturbance_2_1.5.csv', data, delimiter= ",")
