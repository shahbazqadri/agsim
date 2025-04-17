import sys
import copy
#import rospy
import numpy as np
from math import sin, cos
import itertools
#import matplotlib.pyplot as plt
'''import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sin, cos, sqrt, pi, log
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list'''

class AICO_solver(object):
    def __init__(self, model):
        super(AICO_solver, self).__init__()

        '''# initialization of moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('custom_trajectory',anonymous=True)
        joint_state_topic = ['joint_states:=/robot/joint_states']
        # Instantiate RobotCommander Object as an outer level interface to the robot
        robot = moveit_commander.RobotCommander()
        # Instantiate PlanningSceneInterface object as an interface to the environment(world)
        scene = moveit_commander.PlanningSceneInterface()
        # Instantiate MoveGroupCommander object as an interface to the planning group
        #group_name = "arm"
        group_name = groupname
        group = moveit_commander.MoveGroupCommander(group_name, ns="")
        # Publisher to publish the trajectories for visualization in RViz
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        # Planning frame
        planning_frame = group.get_planning_frame()
        # Planning group names
        group_names = robot.get_group_names()
        self.robot = robot
        self.group = group
        self.scene = scene
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.group_names = group_names
        
        # Initializing start position
        joint_goal = group.get_current_joint_values()
        start = [0.25, -0.25, 3.1415, 0.9494, 0, 2.3561, 0, -1.5, 0]
        for i in range(0, len(joint_goal)):
            joint_goal[i] = start[i+(len(start) - len(joint_goal))]
        
        # go can be called with joint values, poses or withour any arguments if the pose or joint target has already been set for the group
        group.go(joint_goal, wait=True)
        # To ensure there is no residual movement
        group.stop()'''
        self.model = model
        self.T = 1000
        self.t = 0
        self.n = 3
        self.un = 1 #3
        self.K = 200
        self.x0 = np.array([[-100],[20],[3.14]])
        self.xT =[-90, 10]
        self.h  = 0.001
        
        self.H = np.eye(self.un)
        self.alpha = 0.9
        self.tolerance = 1e-5

        # mean and covariances of the messages in the previous timestep  
        self.s = np.zeros((self.n, self.T +1))
        self.S = np.zeros((self.n, self.n * (self.T+1)))
        self.Sinv = np.zeros((self.n, self.n * (self.T+1)))


        self.Vinv = np.zeros((self.n, self.n * (self.T+1)))
        self.V = np.zeros((self.n, self.n * (self.T+1)))
        self.v = np.zeros((self.n, self.T +1))

        self.o = np.zeros((self.un, self.T +1))
        self.O = np.zeros((self.un, self.n * (self.T+1)))
        self.u   = np.zeros((self.un, self.T +1))


        self.belief    = np.zeros((self.n, self.T +1))
        #self.belief1    = np.zeros((self.n, self.T +1))
        self.beliefcov = np.zeros((self.n, self.n * (self.T+1)))
        self.X      = np.zeros((self.n, self.T +1))
        self.data      = np.zeros((self.K*self.n, self.T +1))
        self.X[:,0:1] = self.x0

        self.Xu      = np.zeros((self.n, self.T +1))
        self.datau      = np.zeros((self.K*self.n, self.T +1))
        self.Xu[:,0:1] = self.x0
    
        

        self.s[:, 0:1] = self.x0
        self.S[:,0:self.n] = 1e-10*np.eye(self.n)
        self.Sinv[:,0:self.n] = 1e10*np.eye(self.n)

        

        self.belief[:, 0:1] = self.x0
        #self.belief1[:, 0:1] = self.x0
        self.beliefcov[:,0:self.n] = 1e-10*np.eye(self.n)




        self.R = np.zeros((self.n, self.n * (self.T+1)))
        self.r = np.zeros((self.n, self.T +1))

        self.R[:,0:self.n] = np.eye(self.n)

        self.A = np.zeros((self.n, self.n * (self.T+1)))
        self.B = np.zeros((self.n, self.un * (self.T+1)))
        self.a = np.zeros((self.n, self.T +1))
        self.vel = 10.0
        if self.model == 'linear':
            self.Q = 0.0001*np.eye(self.n)
            self.A[:,0:self.n] = np.eye(self.n) #+ self.h*np.array([[1,0,-self.h*10*sin(self.x0[2])],[0,1,self.h*10*cos(self.x0[2])],[0,0,1]])
            self.B[:,0:self.un] = np.eye(self.un)
            self.a[:, 0:1] = np.zeros((self.n,1))
        elif self.model == 'unicycle':
            self.Q = 0.0001*np.eye(self.n)*(self.h)

            Jx= np.array([[0,0,-self.vel*sin(self.x0[2])],[0,0,self.vel*cos(self.x0[2])],[0,0,0]])
            self.A[:,0:self.n] = np.eye(self.n) +  self.h*Jx
            self.a[:, 0:1] = self.h * np.array([[self.vel*cos(self.x0[2])],[self.vel*sin(self.x0[2])],[0]]) - self.h*np.dot(Jx,self.x0)
            self.B[:,0:self.un] = self.h * np.array([[0],[0],[1]])

    # get cost matrices R, r
    def compute_task_message(self,xhat):
        t = self.t
        tn = self.t*self.n
        t1 = (self.t+1)*self.n
        prec = [1e-4, 1e5]
        vel = 10.0
        # Linear case
        if self.model == 'linear':
            self.R[:,tn:t1] = np.eye(self.n)
            self.r[:, t:t+1] = np.zeros((self.n,1))
        # Unicycle model
        elif self.model == 'unicycle':
             xhat_tolist = list(itertools.chain.from_iterable(xhat))
             J =np.array([[1,0,0],[0,1,0]]) #np.array([[1,0,0],[0,1,0], [0,0,1]])
             y = np.array([[self.xT[0]],[self.xT[1]]])#np.array([[self.xT[0]],[self.xT[1]], [self.xT[2]]])
             phi = np.array([[xhat_tolist[0]],[xhat_tolist[1]]])#np.array([[xhat_tolist[0]],[xhat_tolist[1]],[xhat_tolist[2]]]) 
             if t < self.T:
                 self.R[:,tn:t1] = prec[0]*np.dot(J.transpose(),J)
                 #import pdb; pdb.set_trace()
                 self.r[:, t:t+1]=  prec[0]*np.dot(J.transpose(),(y - phi + np.dot(J, xhat)))
             else:
                 self.R[:,tn:t1] =   prec[1]*np.dot(J.transpose(),J)
                 self.r[:, t:t+1]=   prec[1]*np.dot(J.transpose(),(y - phi + np.dot(J, xhat)))
        else:
            pass


        
       
    # get system matrices A, B, a
    def get_system_dynamics(self,xhat):
        t = self.t
        tn = self.t*self.n
        tun = self.t*self.un
        t1 = (self.t+1)*self.n
        tu1= (self.t+1)*self.un
        
        # Linear case
        if self.model == 'linear':
            self.A[:,tn:t1] = np.eye(self.n)
            self.a[:, t:t+1] = np.zeros((self.n,1))
            self.B[:,tun:tu1] = np.eye(self.n)
        # Unicycle model
        elif self.model == 'unicycle':
            vel = 10.0
            mu = 2.0 #3.0 #0.2
            gamma = 1.5 #5.0 #0.1
            Jx= np.array([[0,0,-vel*sin(xhat[2])],[0,0,vel*cos(xhat[2])],[0,0,0]])
            self.A[:,tn:t1] = np.eye(self.n) +  self.h*Jx
            self.a[:, t:t+1] = self.h * np.array([[vel*cos(xhat[2])+mu],[vel*sin(xhat[2])+gamma],[0]]) - self.h*np.dot(Jx,xhat)
            self.B[:,tun:tu1] = self.h * np.array([[0],[0],[1]])
        else:
            pass
            
        
    # Forward message
    def compute_forward_message(self):

        t = self.t
        tn = self.t*self.n
        tun = self.t*self.un
        t1 = (self.t -1)*self.n
        t2 = (self.t +1)*self.n
        tu1 = (self.t -1)*self.un
        tu2 = (self.t +1)*self.un


        sp = self.s[:,t-1:t]
        Sp = self.S[:,t1:tn]
        Sinvp = self.Sinv[:,t1:tn]

        Ap = self.A[:,t1:tn]
        Bp = self.B[:,tu1:tun]
        ap = self.a[:,t:t+1]

        rp = self.r[:,t-1:t]
        Rp = self.R[:,t1:tn]
       

        Q = self.Q
        H = self.H


        Sbar = np.linalg.inv(Sinvp+Rp)
        W    = Q + np.dot(np.dot(Bp,np.linalg.inv(H)),Bp.transpose())

        s = ap + np.dot(Ap, np.dot(Sbar,(np.dot(Sinvp,sp)+rp)))

        S =  W + np.dot(np.dot(Ap, np.linalg.inv(Sinvp+Rp)), Ap.transpose())

        print 
        self.s[:,t:t+1] = s
        self.S[:,tn:t2] = S
        self.Sinv[:,tn:t2] = np.linalg.inv(S)


    # Backward message
    def compute_backward_message(self):

        t = self.t
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        t2 = (self.t +2)*self.n
        tun = self.t*self.un
        tu1 = (self.t +1)*self.un

        vn = self.v[:,t+1:t+2]
        Vn = self.V[:,t1:t2]
        Vinvn = self.Vinv[:,t1:t2]

        Ac = self.A[:,tn:t1]
        Bc = self.B[:,tun:tu1]
        ac = self.a[:,t:t+1]

        rn = self.r[:,t+1:t+2]
        Rn = self.R[:,t1:t2]

        Q = self.Q
        H = self.H

        #print  vn


        Vbar = np.linalg.inv(Vinvn +Rn)
        #print Vbar
        W    = Q + np.dot(np.dot(Bc,np.linalg.inv(H)),Bc.transpose())
        #print W

        v = -np.dot(np.linalg.inv(Ac), ac) + np.dot(np.linalg.inv(Ac),np.dot(Vbar,(np.dot(Vinvn,vn)+rn)))
        #print v
        V = np.dot(np.linalg.inv(Ac), np.dot(W+Vbar, np.linalg.inv(Ac.transpose())))
        #print V


        self.v[:,t:t+1] = v 
        self.V[:,tn:t1] = V
        self.Vinv[:,tn:t1] = np.linalg.inv(V)

    # Belief computation
    def compute_belief(self):
        
        t = self.t
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        
        Sinv = np.linalg.inv(self.S[:,tn:t1])
        Vinv = self.Vinv[:,tn:t1]
        R    = self.R[:,tn:t1]
        s    = self.s[:,t:t+1]
        v    = self.v[:,t:t+1]
        r    = self.r[:,t:t+1]
        
        self.bcov  = Sinv + Vinv + R
        
        self.bmean = np.dot(np.linalg.inv(self.bcov),(np.dot(Sinv,s) + np.dot(Vinv,v) + r))
        self.beliefcov[:,tn:t1] = self.bcov
        #import pdb; pdb.set_trace()
        
    def compute_feedback_controller(self):

        t = self.t
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        t2 = (self.t +2)*self.n
        tun = self.t*self.un
        tu1 = (self.t +1)*self.un

        vn = self.v[:,t+1:t+2]
        Vn = self.V[:,t1:t2]
        Vinvn = self.Vinv[:,t1:t2]

        Ac = self.A[:,tn:t1]
        Bc = self.B[:,tun:tu1]
        ac = self.a[:,t:t+1]

        rn = self.r[:,t+1:t+2]
        Rn = self.R[:,t1:t2]

        Q = self.Q
        H = self.H

        Vbar  = Vinvn + Rn
        vbar  = np.dot(Vinvn, vn) + rn
        Vstar = np.linalg.inv(Q + np.linalg.inv(Vbar))
        M     = np.dot(np.dot(Bc.transpose(),Vstar),Bc) + H

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



    # run through iterations
    def AICO(self):
        T = self.T
        for k in range(0,self.K):
            print('k =',k )
            self.k = k
            #print 'forward pass'
            
            for t in range(1, T+1):
                #print 't =', t
                self.t = t
                self.compute_forward_message()
                if k == 0:
                    xhat = self.s[:,t:t+1]
                    #print 'xhat =', xhat
                else:
                    xhat = (1-self.alpha)* self.X[:,t:t+1] + self.alpha*self.belief[:,t:t+1]
                    #print 'xhat =', xhat

                self.get_system_dynamics(xhat)
                self.compute_task_message(xhat)
                self.compute_belief()
                conditionmet = False
                if k >0:
                    if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                        continue
                    else:
                        while not conditionmet:
                            #b = np.array(np.random.multivariate_normal(list(itertools.chain.from_iterable(self.bmean)), self.bcov)).reshape(self.n,1)
                            xhat = (1-self.alpha)* xhat+ self.alpha*self.bmean
                            self.get_system_dynamics(xhat)
                            self.compute_task_message(xhat)
                            self.compute_belief()
                            if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                                self.belief[:,t:t+1] = self.bmean
                                #print 'b=',b
                                #import pdb; pdb.set_trace()
                                conditionmet = True
                                

                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean
            

            #print 'backward pass'
            for t in range(T, 0, -1):
                #print 't=',t                
                self.t = t

                if t  == T:
                    #self.v[:,t:t+1] = self.belief[:,t:t+1]
                    self.v[:,t:t+1] = self.belief[:,t:t+1]
                    self.V[:,(self.t )*self.n:(self.t +1 )*self.n] = self.S[:,(self.t )*self.n:(self.t +1 )*self.n]
                    self.Vinv[:,(self.t)*self.n:(self.t +1)*self.n] = np.linalg.inv(self.S[:,(self.t )*self.n:(self.t +1 )*self.n])
                    #xhat = self.v[:,t:t+1]
               
                else:
                    self.compute_backward_message()
                    
                xhat = (1-self.alpha)* self.X[:,t:t+1] + self.alpha*self.belief[:,t:t+1]
                #print 'xhat =',xhat

                self.get_system_dynamics(xhat)
                self.compute_task_message(xhat)
                self.compute_belief()
                #conditionmet = False

                '''while not conditionmet:
                    b = np.array(np.random.multivariate_normal(list(itertools.chain.from_iterable(self.bmean)), self.bcov)).reshape(self.n,1)
                    if abs(max((xhat-b)**2)) < self.tolerance:
                        self.belief[:,t:t+1] = b
                        conditionmet = True
                        #print 'b =', b
                        #import pdb; pdb.set_trace()
                        self.X[:,t:t+1] = xhat'''

                if k >0:
                    if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                        continue
                    else:
                        while not conditionmet:
                            #b = np.array(np.random.multivariate_normal(list(itertools.chain.from_iterable(self.bmean)), self.bcov)).reshape(self.n,1)
                            xhat = (1-self.alpha)* xhat+ self.alpha*self.bmean
                            self.get_system_dynamics(xhat)
                            self.compute_task_message(xhat)
                            self.compute_belief()
                            if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                                self.belief[:,t:t+1] = self.bmean
                                #print 'b=',b
                                #import pdb; pdb.set_trace()
                                conditionmet = True
                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean   

            print('X=', self.X[:,0:-2])
            #print 'b=', self.belief
            #print 'b1=', self.belief1
            for t in range(0,T):
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

                self.Xu[:,t+1:t+2] = np.dot(Ac,self.Xu[:,t:t+1]) + ac + np.dot(Bc, self.u[:,t:t+1])
                
            self.data[k*self.n:(k+1)*self.n,:] = self.X
            #self.datau[k*self.n:(k+1)*self.n,:] = self.Xu
            self.datau[k*self.n:(k+1)*self.n,:] = self. u
            #self.data[k*self.n:(k+1)*self.n,:] = self.belief
        #return self.data, self.beliefco
        return self.data, self.datau

sol = AICO_solver('unicycle')
print('running AICO')
data, datau = sol.AICO()
np.savetxt('data_AICO_unicycle.csv', data, delimiter= ",")
np.savetxt('data_AICO_control_unicycle.csv', data, delimiter= ",")
