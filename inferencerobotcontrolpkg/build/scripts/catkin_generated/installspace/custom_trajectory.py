#!/usr/bin/env python2
#imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sin, cos, sqrt, pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import csv
from points_on_circle import points_on_circle

# Code based on the tutorial on movegroup python interface 
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py




class custom_trajectory(object):
    def __init__(self,groupname):
        super(custom_trajectory,self).__init__()

        
        # initialization of moveit_commander and rospy node

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

        # Basic information
        # Name of reference frame
        planning_frame = group.get_planning_frame()
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

        self.robot = robot 
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        #self.eef_link = eef_link
        self.group_names = group_names  

    def go_to_joint_state(self):

        group = self.group

        print self.group
        # initializing to start position
        joint_goal = group.get_current_joint_values()
        print joint_goal
        start = [0.25, -0.25, 3.1415, 0.9494, 0, 2.3561, 0, -1.5, 0]
        for i in range(0, len(joint_goal)):
            joint_goal[i] = start[i+(len(start)-len(joint_goal))]
        '''joint_goal[0] = 0.25 #xgantry_baselink
        joint_goal[1] = -0.25 #ytrackplate_xgantry

        joint_goal[2] = 3.1415 #joint_s
        joint_goal[3] = 0.9494 #joint_l
        joint_goal[4] = 0    #joint_e
        joint_goal[5] = 2.3561 #joint_u
        joint_goal[6] = 0    #joint_r
        joint_goal[7] = -1.5 #joint_b
        joint_goal[8] = 0    #joint_t'''
        # go can be called with joint values, poses or withour any arguments if the pose or joint target has already been set for the group
        group.go(joint_goal, wait=True)

        # To ensure there is no residual movement
        group.stop()

    def plan_cartesian_path(self, waypoints):
        group = self.group
           
        #waypoints = sinusoidaltrajectory()

        (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0)

        return plan,fraction
    

    def sinusoidaltrajectory(self, scale=1):
        group = self.group
        waypoints = []
        wpose = group.get_current_pose().pose
        for i in range(0,300):
            wpose.position.y -= 0.01*scale
            wpose.position.x = -2.09 + sin((4)*wpose.position.y)
            print(wpose)
            waypoints.append(copy.deepcopy(wpose))
        
        return waypoints

    def circulartrajectory(self):
        
        group = self.group
        waypoints = []
        wpose = group.get_current_pose().pose

        with open('/home/wmf-admin/Desktop/robotcontrolpkg/src/scripts/src/waypoints.csv','r') as f:

            #Instantiate csv writer
            reader = csv.reader(f, delimiter= ',')

            for row in reader:
                wpose.position.x = float(row[0])
                wpose.position.y = float(row[1])
                waypoints.append(copy.deepcopy(wpose))
                
       
        
        
        return waypoints



    def display_trajectory(self,plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher


        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self,plan):
        group = self.group

        group.execute(plan,wait=True)


def main():
        try:
            print "Press Enter to setup moveit commander (press ctrl+D to exit)"
            raw_input()
            ag_trajectory = custom_trajectory("arm_gantry")
            a_trajectory  = custom_trajectory("arm")

            for i in range(0,3):
                print "Press Enter to go to start position"
                raw_input()
                ag_trajectory.go_to_joint_state()

                print "Press Enter to execute sinusoidal trajectory"
                raw_input()
                waypoints = ag_trajectory.sinusoidaltrajectory()
                '''print "Press Enter to display cartesian path"
                raw_input()'''
                cartesian_plan,fraction = ag_trajectory.plan_cartesian_path(waypoints)

                '''print "Press Enter to display saved trajectory"
                raw_input()'''
                ag_trajectory.display_trajectory(cartesian_plan)

                '''print "Press Enter to execute saved trajectory"
                raw_input()'''
                
                ag_trajectory.go_to_joint_state()
                ag_trajectory.execute_plan(cartesian_plan)
            
            print "Done"


            print "Press Enter to go to start position"
            raw_input()
            ag_trajectory.go_to_joint_state()

            print "Press Enter to execute circular trajectory"
            raw_input()
            waypoints = a_trajectory.circulartrajectory()
            '''print "Press Enter to display cartesian path"
            raw_input()'''
            cartesian_plan,fraction = a_trajectory.plan_cartesian_path(waypoints)

            '''print "Press Enter to display saved trajectory"
            raw_input()'''
            a_trajectory.display_trajectory(cartesian_plan)

            '''print "Press Enter to execute saved trajectory"
            raw_input()'''
            for i in range(0,3):
                a_trajectory.go_to_joint_state()
                a_trajectory.execute_plan(cartesian_plan)




        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

if __name__ == '__main__':
    main()



