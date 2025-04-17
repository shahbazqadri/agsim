#!/usr/bin/env python
import numpy
import csv
from math import pi, sin, cos

#center of circle [x,y]
h = -2.14
k = -0.98

#radius of circle
r = 0.2

#circular trajectory waypoint generator
def points_on_circle(npoints, h, k, r):
    #empty list
    l = []
    for i in range(0,npoints):
        angle = i*2*pi/npoints
        x = h + r*cos(angle)
        y = k + r*sin(angle)
        l.append([x,y])

    return l
l = points_on_circle(100, h, k, r)

# open file in write mode
with open('/home/wmf-admin/Desktop/robotcontrolpkg/src/scripts/src/waypoints.csv','w') as f:

    #Instantiate csv writer
    writer = csv.writer(f)

    writer.writerows(l)

