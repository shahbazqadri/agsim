#!/usr/bin/env python
#imports
import sys
import copy
import rospy
import numpy as np
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
import moveit_commander
import tf
import tf.transformations
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import sin, cos, sqrt, pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import itertools

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('custom_trajectory',anonymous=True)
traj_publisher = rospy.Publisher('/robot/wavetank_arm_gantry_controller/command', JointTrajectory, queue_size = 10)
joint_state_topic = ['joint_states:=/robot/joint_states']
# Instantiate RobotCommander Object as an outer level interface to the robot
robot = moveit_commander.RobotCommander()

# Instantiate PlanningSceneInterface object as an interface to the environment(world)
scene = moveit_commander.PlanningSceneInterface()

# Instantiate MoveGroupCommander object as an interface to the planning group
#group_name = "arm"
group_name = 'arm_gantry'
group = moveit_commander.MoveGroupCommander(group_name, ns="")

# Basic information
# Name of reference frame

planning_frame = group.get_pose_reference_frame()
print "Reference frame: %s" % planning_frame
 # Name of End-effector link 
    #eef_link = group.get_end_effector_link()
    #print "End effector: %s" % eef_link

# List of groups
group_names = robot.get_group_names()
print "Robot Groups:", group_names

# Current Robot State for debugging purposes
print "Printing robot state"
print robot.get_current_state
print ""

#transform = tf.TransformListener()

(status, tree) = kdl_parser.treeFromFile('src/robot/urdf/xy-stage_arm.urdf')
#print status
chain = tree.getChain("world", "xystagelink_t")
num_joints = chain.getNrOfJoints()
#print num_joints
fk_pos = kdl.ChainFkSolverPos_recursive(chain)
ee_pose = kdl.Frame()
theta = kdl.JntArray(num_joints)


def get_jacobian(joint_goal):
   
    # go can be called with joint values, poses or withour any arguments if the pose or joint target has already been set for the group
    group.go(joint_goal, wait=True)

    # To ensure there is no residual movement
    group.stop()

    #group.set_pose_reference_frame('xystagexgantry')
    J = group.get_jacobian_matrix(joint_goal)

    jac = np.array(J[0:3,:])

    #print jac.shape
    #print group.get_current_pose()
    return jac

def get_cartesian_state(joint_goal):


    pos = group.get_current_pose().pose
    #rpy = group.get_current_rpy()

    #y = np.array([[pos.position.x],[pos.position.y],[pos.position.z],[rpy[0]],[rpy[1]],[rpy[2]]]) 
    y = np.array([[pos.position.x],[pos.position.y],[pos.position.z]])
    #print 'y=',y

    return y

def kdl_to_mat(data):
    mat = np.mat(np.zeros((data.rows(), data.columns())))
    for i in range(data.rows()):
        for j in range(data.columns()):
            mat[i,j] = data[i,j]

    return mat


def task_matrices(jntval):
    for i in range(0, len(jntval)):
        theta[i] = jntval[i]
    #print theta
    fk_pos.JntToCart(theta, ee_pose)
    y = np.array([[ee_pose.p[0]], [ee_pose.p[1]], [ee_pose.p[2]]])
    #print ee_pose.p
    #print ee_pose.M
    jacobian_solver = kdl.ChainJntToJacSolver(chain)
    J = kdl.Jacobian(num_joints)
    jacobian_solver.JntToJac(theta,J)
    J = kdl_to_mat(J)

    return J[0:3,:] , y





def constrain(joint_value, lower, upper):

    for i in range(0, joint_value.shape[0]):

        if joint_value[i] < lower[i]:
            joint_value[i] = lower[i]
        elif joint_value[i] > upper[i]:
            joint_value[i] = upper[i]
        else:
            pass

    joint_goal = list(itertools.chain.from_iterable(joint_value))
    group.go(joint_goal, wait=True)

    # To ensure there is no residual movement
    group.stop()
    xhat = np.array(group.get_current_joint_values()).reshape(9,1)

    return xhat


#joint_goal = [1.5,0.1,0.5,0.2,0.7,0.2,0,0.21,0]
#get_jacobian(joint_goal, 'arm_gantry')

class AICO_solver(object):
    def __init__(self, model, qo, qf):
        super(AICO_solver, self).__init__()

        self.model = model
        self.T = 3#Total number of Timesteps
        self.t = 0 
        self.n   = 9 #dimension of state
        self.un  = 9#dimension of the input
        self.x0  = qo #np.array([[-100],[20],[3.14],[self.vel],[0]]) #initial state
        
        if self.model == 'nonlinear':
            
            q0_tolist = list(itertools.chain.from_iterable(qo))
            group.go(q0_tolist, wait=True)
            # To ensure there is no residual movement
            group.stop()
            print 'qf=', qf
            '''j = get_jacobian(q0_tolist)
            print 'jac x0= ', j
            xo = get_cartesian_state(q0_tolist)'''
            j, xo = task_matrices(q0_tolist)
            qf_tolist = list(itertools.chain.from_iterable(qf))
            group.go(qf_tolist, wait=True)
            # To ensure there is no residual movement
            group.stop()
            '''xf = get_cartesian_state(qf_tolist)
            j = get_jacobian(qf_tolist)
            #(trans,rot) = transform.lookupTransform('world', 'xgantry_baselink',  rospy.Time(0))'''
            j, xf = task_matrices(qf_tolist)
            print 'xf=', xf
            print group.get_active_joints()
            print 'jac_xf=', j
            #print 'trans=', trans
            #print 'rot=', rot
            self.xo  = xo
            self.xT  = xf #[0,0] #goal state

        elif self.model == 'linear':
            self.xT = qf
        
        else:
            pass

        self.h  = 0.1 # stepsize
        self.trajectorytolerance = 1e-1 #trajectory tolerance   
        self.alpha = 0.9 #convergence rate
        self.tolerance =  1e-2 #1e-5 #belief tolerance
        self.lower = np.array([[0.25], [-1.5],[-3.1415], [-1.9198], [-2.9670], [-2.3561], [-3.1415],[-1.9198], [-3.1415]])
        self.upper = np.array([[3],[0.75], [3.1415], [1.9198], [2.9760], [2.3561], [3.1415],[1.9198],[3.1415]])

        # mean and covariance of the forward message
        self.s = np.zeros((self.n, self.T +1))
        self.S = np.zeros((self.n, self.n * (self.T+1)))
        self.Sinv = np.zeros((self.n, self.n * (self.T+1)))
        ## Initialization
        self.s[:, 0:1] = self.x0
        self.S[:,0:self.n] = 1e-10*np.eye(self.n)
        self.Sinv[:,0:self.n] = 1e10*np.eye(self.n)

        # mean and covariance of the backward message
        self.Vinv = np.zeros((self.n, self.n * (self.T+1)))
        self.V = np.zeros((self.n, self.n * (self.T+1)))
        self.v = np.zeros((self.n, self.T +1))

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


        # System matrices in the form x_{t+1} = A_t * x_t + a_t + B_t * u_t
        self.A = np.zeros((self.n, self.n * (self.T+1)))
        self.B = np.zeros((self.n, self.un * (self.T+1)))
        self.a = np.zeros((self.n, self.T +1))

        ## Initialization
        self.Q = 1e-10*np.eye(self.n)*(self.h) # process noise covariance

        self.A[:,0:self.n] = np.eye(self.n) 
        self.B[:,0:self.un] =   self.h*np.eye(self.n)
        self.H  = np.eye(self.un)

    def reset_to_start(self):

        x0_tolist = list(itertools.chain.from_iterable(self.x0))
        group.go(x0_tolist, wait=True)

        # To ensure there is no residual movement
        group.stop()      

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
       
        #precisions for task variables
        prec1 = [1e-2, 1e5]  # for task space position 

        xhat_tolist = list(itertools.chain.from_iterable(xhat))  # convert xhat to list

        if self.model == 'nonlinear':
            # Jacobian of the map from state-space to task-space 
            #J1 = get_jacobian(xhat_tolist)


            # Desired Task space variables
            #y1 = self.xo + self.t*(self.xT - self.xo)/self.T
            y1 = self.xT


            # Map from state-space to task-space y = phi(xhat)
            #phi1 = get_cartesian_state(xhat_tolist)
            J1, phi1 = task_matrices(xhat_tolist)

            print 'y1=', y1
            print 'phi1=', phi1

        
        elif self.model == 'linear':
            # Jacobian of the map from state-space to task-space 
            J1 = np.eye(self.n)

            # Desired Task space variables
            y1 = self.xT


            # Map from state-space to task-space y = phi(xhat)
            phi1 = xhat
            print 'phi1=', phi1

        else:
            pass

        
        # Task message updates
        if t < self.T:
            self.R[:,tn:t1] = prec1[0]*np.dot(J1.transpose(),J1) 

            self.r[:, t:t+1]=  prec1[0]*np.dot(J1.transpose(),(y1 - phi1 + np.dot(J1, xhat))) 
        else:
            self.R[:,tn:t1] =   prec1[1]*np.dot(J1.transpose(),J1) 
            self.r[:, t:t+1]=   prec1[1]*np.dot(J1.transpose(),(y1 - phi1 + np.dot(J1, xhat))) 

   
    # Get system dynamics
    def get_system_dynamics(self,xhat):
    
        t = self.t # current timestep
        #indices
        tn = self.t*self.n
        tun = self.t*self.un
        t1 = (self.t+1)*self.n
        tu1= (self.t+1)*self.un
        
        
        self.A[:,tn:t1] = np.eye(self.n) 
        self.B[:,tun:tu1] = self.h*np.eye(self.un)
            
        
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

        '''s = ap + np.dot(Ap, np.dot(Sbar,(np.dot(Sinvp,sp)+rp)))

        S =  W + np.dot(np.dot(Ap, np.linalg.inv(Sinvp+Rp)), Ap.transpose())'''
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

        '''v = -np.dot(np.linalg.inv(Ac), ac) + np.dot(np.linalg.inv(Ac),np.dot(Vbar,(np.dot(Vinvn,vn)+rn))) 
        V = np.dot(np.linalg.inv(Ac), np.dot(W+Vbar, np.linalg.inv(Ac.transpose())))'''
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
        #import pdb; pdb.set_trace()\


    
    
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
            count = 0
            if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                return
            else:
                while not conditionmet:
                    '''count +=1
                    if count <30:'''
                    xhat = (1-self.alpha)* xhat+ self.alpha*self.bmean
                    xhat = constrain(xhat,self.lower, self.upper)
                    self.update(xhat)

                    if abs(max((xhat-self.bmean)**2)) < self.tolerance:
                        self.belief[:,t:t+1] = self.bmean
                        conditionmet = True

                    else:
                        diff = xhat - self.bmean
                        self.belief[:,t:t+1] = constrain(xhat + 0.05/np.linalg.norm(diff), self.lower, self.upper)
                        conditionmet = True
                    
                                


    # AICO algorithm
    def AICO(self):
        print('running AICO')
        T = self.T #Total timesteps of the trajectory
        self.Xold  = np.zeros((self.n, self.T +1)) #trajectory in the previous iteration
        diff= np.full(self.X.shape, 1) #error in the state trajectory in the consecutive iterations
        #for k in range(0,self.K):
        k = 0 # iterator
        # run through iterations until convergence
        #while abs(np.max(error)) > self.trajectorytolerance and k < 15: #trajectory convergence condition
        while abs(np.max(diff[:,0:-1])) > self.trajectorytolerance and k < 100:
            print('k =',k )
            self.k = k # current iteration
            self.reset_to_start()
            if k>0:
                #update trajectory in previous iteration 
                self.Xold[:,0:-1] = self.X[:,0:-1]   
                self.reset_to_start()

            # forward pass        
            for t in range(1, T+1):
                self.t = t #current timestep
                self.compute_forward_message() #compute forward message distribution

                # current state estimate
                if k == 0:
                    xhat = self.s[:,t:t+1]

                else:
                    xhat = (1-self.alpha)* self.X[:,t:t+1] + self.alpha*self.belief[:,t:t+1]

                xhat = constrain(xhat, self.lower, self.upper)
                print 'tf=', t
                #print 'xhat=', xhat
                #self.get_system_dynamics(xhat)
                
                self.update(xhat) #update using current state estimate
                
                self.beliefconvergence(xhat)#belief convergence
             
                # update self
                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean
                
                '''bmean_tolist = list(itertools.chain.from_iterable(self.bmean))
                group.go(bmean_tolist, wait=True)

                # To ensure there is no residual movement
                group.stop()'''
            

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
                xhat = constrain(xhat, self.lower, self.upper)
                print 'tb=', t
                #print 'xhat=', xhat
                
                self.update(xhat) # update using current state estimate
                
                self.beliefconvergence(xhat) #belief convergence
                
                #update self
                self.X[:,t:t+1] = xhat
                self.belief[:,t:t+1] = self.bmean
                 
                   
            
            # check difference in the trajectory in consecutive iterations
            if k ==0:
                diff = self.X - self.Xold
               
            else:
                diff = self.X - self.Xold

            # sum of differences at each timestep    
            error = np.sum(diff[:,0:-1], axis= 1)    
            #print('diff=', diff) #print diff to screen
            k = k+1 #increment iterator
        
        return self.X[0:-1]



# main

qo = np.array([[0.25],[-0.25],[3.1415],[0.9494],[0],[2.3],[0],[-1.5],[0]])
#qf = np.array(group.get_random_joint_values()).reshape(9,1)
qf = np.array([[3.0],[-0.25],[3.1415],[0.9494],[0],[2.3],[0],[-1.5],[0]])


sol1 = AICO_solver('nonlinear',qo, qf)
X = sol1.AICO()
for i in range(0, X.shape[1]+1):
    traj = JointTrajectory()

    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = ''
    traj.joint_names = ["xgantry_baselink", "ytrackplate_xgantry", "joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"]

    point = JointTrajectoryPoint()

    point.positions = list(itertools.chain.from_iterable(X[:,i:i+1]))
    point.velocities = []
    point.accelerations = []
    point.effort = []
    point.time_from_start = rospy.Duration(5)

    traj.points.append(point)

    traj_publisher.publish(traj)
    rospy.loginfo(traj)

print get_cartesian_state(list(itertools.chain.from_iterable(X[:,-1:])))
