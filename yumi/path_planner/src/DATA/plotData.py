#!/usr/bin/env python3
import rospy
import tf
import time
import numpy as np
from std_msgs.msg import Bool
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D, proj3d
import matplotlib.pyplot as plt
import matplotlib as mpl
import ast
 
#Data from the left arm
RealL = open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataReal/Left/Lrun5L.txt","r")
SimL = open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataRealSim/Left/Lsim5L.txt","r")
 
#Data from the right arm
RealR = open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataReal/Left/Rrun5L.txt","r")
SimR = open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataRealSim/Left/Rsim5L.txt","r")
 
dataStrL = RealL.read()
dataStrSimL = SimL.read()
 
dataStrR = RealR.read()
dataStrSimR = SimR.read()
 
#Convert data string to array for left arm
dataL = np.array([[float(j) for j in i.split('\t')]for i in dataStrL.splitlines()])
dataSimL = np.array([[float(j) for j in i.split('\t')]for i in dataStrSimL.splitlines()]) 
 
#Convert data string to array for right arm
dataR = np.array([[float(j) for j in i.split('\t')]for i in dataStrR.splitlines()])
dataSimR = np.array([[float(j) for j in i.split('\t')]for i in dataStrSimR.splitlines()])
 
 
 
mpl.rcParams['legend.fontsize'] = 10
 
fig = plt.figure()
fig.suptitle('Picking from left side')
 
 
#Plott for YuMis right arm
ax = fig.add_subplot(1, 2, 1, projection='3d')
ax.text2D(0.05, 0.95, "YuMis right arm", transform=ax.transAxes)
 
xyzR = np.array(dataR)
xR = xyzR[:,0]
yR = xyzR[:,1]
zR = xyzR[:,2]
 
simR = np.array(dataSimR)
xSimR = simR[:,0]
ySimR = simR[:,1]
zSimR = simR[:,2]
 
 
ax.plot3D(xR, yR, zR, 'red', label= 'The grippers actual movement')
ax.plot3D(xSimR, ySimR, zSimR, 'blue' ,label= 'The grippers simulated movement')
ax.scatter(xR[0], yR[0], zR[0], 'green')
 
#ax.set_xlim([0.3, 0.5])
#ax.set_ylim([-40, 0])
#ax.set_zlim([0, 0.25])
 
ax.axes.set_xlim3d(left=0.3, right=0.5)
ax.axes.set_ylim3d(bottom=-0.5, top=0) 
ax.axes.set_zlim3d(bottom=0, top=0.25)
 
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
 
ax.legend()
 
#Plott for YuMis left arm
 
ax = fig.add_subplot(1, 2, 2, projection='3d')
ax.text2D(0.05, 0.95, "YuMis left arm", transform=ax.transAxes)
 
 
xyzL = np.array(dataL)
xL = xyzL[:,0]
yL = xyzL[:,1]
zL = xyzL[:,2]
 
simL = np.array(dataSimL)
xSimL = simL[:,0]
ySimL = simL[:,1]
zSimL = simL[:,2]
 
ax.plot3D(xL, yL, zL, 'red')
ax.plot3D(xSimL, ySimL, zSimL, 'blue')
ax.scatter(xL[0], yL[0], zL[0], 'green')
#ax.scatter(xSimL[0], ySimL[0], zSimL[0], 'green')
 
ax.axes.set_xlim3d(left=0.5, right=0.3)
ax.axes.set_ylim3d(bottom=0.5, top=0) 
ax.axes.set_zlim3d(bottom=0, top=0.25)
 
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
 
 
plt.show()
 
 
 
 
 
 
