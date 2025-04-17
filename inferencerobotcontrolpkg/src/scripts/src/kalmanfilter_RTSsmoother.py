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

class kalmansmoother(object):
    def __init__(self, model):
        super(kalmansmoother, self).__init__()

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
        self.K = 100
        self.x0 = np.array([[-100],[20],[3.14]])
        self.xT =[-90, 10]
        self.h  = 0.001
        
        self.H = np.eye(self.un)
        self.alpha = 0.9
        self.tolerance = 1e-5

        # mean and covariances of the messages in the previous timestep  
        self.mu = np.zeros((self.n, self.T +1))
        self.V = np.zeros((self.n, self.n * (self.T+1)))
        self.P = np.zeros((self.n, self.n * (self.T+1)))

        self.muhat = np.zeros((self.n, self.T +1))
        self.Vhat = np.zeros((self.n, self.n * (self.T+1)))
        

        self.mu0 = self.x0
        self.V0 = 1e-10*np.eye(self.n)

        self.A = np.zeros((self.n, self.n * (self.T+1)))
        self.B = np.zeros((self.n, self.un * (self.T+1)))
        self.a = np.zeros((self.n, self.T +1))
        self.vel = 10.0

        self.R = np.zeros((self.n, self.n * (self.T+1)))
        self.r = np.zeros((self.n, self.T +1))

        self.R[:,0:self.n] = np.eye(self.n)

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

        self.X      = np.zeros((self.n, self.T +1))
        self.data   = np.zeros((self.K*self.n, self.T +1)) 

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
                 #self.R[:,tn:t1] = prec[0]*np.dot(J.transpose(),J)
                 self.R[:,tn:t1] = prec[0]*np.dot(J.transpose(),J)
                 #import pdb; pdb.set_trace()
                 #self.r[:, t:t+1]=  prec[0]*np.dot(J.transpose(),(y - phi + np.dot(J, xhat)))
                 self.r[:, t:t+1]= prec[0]*np.dot(J.transpose(),(y - phi + np.dot(J, xhat)))
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
            mu = 0.0 #3.0 #0.2
            gamma = 0.0 #5.0 #0.1
            Jx= np.array([[0,0,-vel*sin(xhat[2])],[0,0,vel*cos(xhat[2])],[0,0,0]])
            self.A[:,tn:t1] = np.eye(self.n) +  self.h*Jx
            self.a[:, t:t+1] = self.h * np.array([[vel*cos(xhat[2])+mu],[vel*sin(xhat[2])+gamma],[0]]) - self.h*np.dot(Jx,xhat)
            self.B[:,tun:tu1] = self.h * np.array([[0],[0],[1]])
        else:
            pass
            
        
    #kalman filter algorithm
    def kalmanfilter(self):
        t = self.t
        tn = self.t*self.n
        t1 = (self.t -1)*self.n
        t2 = (self.t +1)*self.n
        tun = self.t*self.un
        tu1 = (self.t -1)*self.un
        tu2 = (self.t +1)*self.un
        H = np.eye(self.n)
        
        if t>0: 
            if self.model == 'linear':
                xhat = np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]) + self.a[:,t-1:t]
           
            elif self.model == 'unicycle':
                if self.k > 0:
                    #xhat = (1-self.alpha)*np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]) + self.a[:,t-1:t] + self.alpha*self.muhat[:,t:t+1]
                    xhat = np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]) + self.a[:,t-1:t]
                else:
                    xhat = np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]) + self.a[:,t-1:t]

            else:
                pass

      

            #print('xhat=',xhat)
            #print(xhat.shape)
            self.compute_task_message(xhat)
    
        #print(self.R[:,tn:t2])
        #R =  np.linalg.inv(self.R[:,tn:t2])
        r = self.r[:,t:t+1]
        #z = np.dot(R,r)
        z = np.zeros((self.n,1))

        if t == 0:
            #k1 = np.linalg.inv(np.dot(np.dot(H,self.V0),H.transpose()) + R) 
            k1 = np.dot(np.dot(np.linalg.inv(np.dot(np.dot(H,self.V0),H.transpose())),np.linalg.inv(np.linalg.inv(np.dot(np.dot(H,self.V0),H.transpose()))+self.R[:,tn:t2])),self.R[:,tn:t2])
            K = np.dot(np.dot(self.V0,H.transpose()),k1)
            mu = self.mu0 + np.dot(K,(z- np.dot(H,self.mu0)))
            V  = np.dot((np.eye(self.n) - np.dot(K,H)),self.V0)
            P  = np.dot(np.dot(self.A[:,tn:t2],V),self.A[:,tn:t2].transpose()) +self.Q + np.dot(np.dot(self.B[:,tun:tu2],np.linalg.inv(self.H)),self.B[:,tun:tu2].transpose())

        else:
            #k1 = np.linalg.inv(np.dot(np.dot(H,self.P[:,t1:tn]),H.transpose()) + R)
            k1 = np.dot(np.dot(np.linalg.inv(np.dot(np.dot(H,self.P[:,t1:tn]),H.transpose())),np.linalg.inv(np.linalg.inv(np.dot(np.dot(H,self.P[:,t1:tn]),H.transpose()))+self.R[:,tn:t2])),self.R[:,tn:t2])
            K = np.dot(np.dot(self.P[:,t1:tn],H.transpose()),k1)
            mu = np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]) + np.dot(K,(z - np.dot(H,np.dot(self.A[:,t1:tn],self.mu[:,t-1:t]+ self.a[:,t-1:t]))))
            Vx = np.eye(self.n) - np.dot(K,H)
            V  = np.dot(Vx,self.P[:,t1:tn])
            P  = np.dot(np.dot(self.A[:,t1:tn],V),self.A[:,t1:tn].transpose()) + self.Q + np.dot(np.dot(self.B[:,tu1:tun],np.linalg.inv(self.H)),self.B[:,tu1:tun].transpose())

        self.mu[:,t:t+1] = mu
        self.V[:,tn:t2] = V
        self.P[:,tn:t2] = P
    
    #Rauch-Tung-Striebel(RTS) smoother
    def RTSsmoother(self):
        t = self.t
        tn = self.t*self.n
        t1 = (self.t +1)*self.n
        t2 = (self.t +2)*self.n


        C = np.dot(np.dot(self.V[:,tn:t1],self.A[:,tn:t1].transpose()),np.linalg.inv(self.P[:,tn:t1]))
        self.muhat[:,t:t+1] = self.mu[:,t:t+1] + np.dot(C, (self.muhat[:,t+1:t+2] - np.dot(self.A[:,tn:t1],self.mu[:,t:t+1])))
        self.Vhat[:,tn:t1]  = self.V[:,tn:t1] + np.dot(np.dot(C, (self.Vhat[:,t1:t2] - self.P[:,tn:t1])),C.transpose())
        #print('muhat',self.muhat[:,t:t+1])
        #print('Vhat', self.Vhat[:,tn:t1])



        
    # run through iterations
    def KFRTS(self):
        T = self.T  

        for k in range(0,self.K):  
            self.k = k        
            #print 'forward pass'
            for t in range(0, T+1):
                #print('t=', t)
                #print 't =', t
                self.t = t
                self.kalmanfilter()
                
                xhat = self.mu[:,t:t+1]

                self.get_system_dynamics(xhat)
                self.compute_task_message(xhat)
            
            #print(self.mu[:,T:T+1])

            #print 'backward pass'
            for t in range(T, -1, -1):
                #print('t=',t)                
                self.t = t

                if t == T:
                    self.muhat[:,t:t+1] = self.mu[:,t:t+1]
                    self.Vhat[:,t*self. n : (t+1)*self.n] = self.V[:,t*self.n:(t+1)*self.n]
                else:
                    self.RTSsmoother()
                    
                
                #print 'xhat =',xhat

                self.get_system_dynamics(xhat)
                #self.compute_task_message(xhat)
                

        print('X=', self.muhat)
        #print 'b=', self.belief
        #print 'b1=', self.belief1


        self.data[k*self.n:(k+1)*self.n,:] = self.muhat
        #self.data[k*self.n:(k+1)*self.n,:] = self.belief
    #return self.data, self.beliefco
        return self.data

sol = kalmansmoother('unicycle')
print('running smoother algorithm')
data = sol.KFRTS()
np.savetxt('data_smoother_unicycle_mu.csv', data, delimiter= ",")