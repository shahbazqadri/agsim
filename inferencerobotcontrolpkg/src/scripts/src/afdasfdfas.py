#!/usr/bin/env python
#imports
import sys
import copy
import rospy
import numpy as np
import itertools
import matplotlib.pyplot as plt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sin, cos, sqrt, pi, log
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class AICO_solver(object):
    def __init__(self):#, groupname):
        super(AICO_solver, self).__init__()

        self. A = np.zeros((4,4))

    def test(self):
            A = self.A
            a = A[:,2:3]

            print a

            b = np.array([[1],[2],[3],[4]])

            self.A[:,2:3] = b

    def test1(self):
            
            np.savetxt('test1.csv', self.A, delimiter= ",")

def main():

    aico = AICO_solver()

    aico.test()
    aico.test1()

if  __name__ == '__main__':
    main()
            