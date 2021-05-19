#!/usr/bin/env python3
 
import rospy
import tf
import time
import numpy as np
from std_msgs.msg import Bool
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
 
 
stop = False
start = False
#dataL = np.empty
dataLr = [[0]*3]*1
dataL = [0, 0, 0]
 
print(dataLr)
print(dataL)
 
def callback_stop(bool):
    global stop
    stop = bool.data
 
def callback_start(bool):
    print('start aktiverades')
    tfListener = tf.TransformListener()  
    time.sleep(1)
 
    i = 0
 
    while (not stop): 
        (posRight, orientationRight) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (posLeft, orientationLeft) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
    
        if i == 0:
            global dataL
            
            dataL = [[ posLeft[0], posLeft[1], posLeft[2] ]]
            i = i + 1
 
        else:
            dataL.append(posLeft)
            time.sleep(0.1)
 
    
        if(stop):
            print('Data gathered')
            break
 
    if stop:
        fig = plt.figure()
        ax = plt.axes(projection="3d")
 
        #refV = 
        #refVX = ref[:,0]
        #refVY = ref[:,1]
        #refVZ = ref[:,2]
 
 
        xyzL = np.array(dataL)
        x = xyzL[:,0]
        y = xyzL[:,1]
        z = xyzL[:,2]
 
        #ax.plot3D(refVX, refVY, refVZ, 'blue')
        ax.plot3D(x, y, z, 'red')
        ax.scatter(x[0], y[0], z[0], 'green')
 
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
 
        plt.show()
        
 
 
# starting ROS node and subscribers
rospy.init_node('gather_data', anonymous=True) 
sub_start = rospy.Subscriber('/start', Bool, callback_start)
sub_stop = rospy.Subscriber('/stop', Bool, callback_stop)
 
print('Hejsan hoppsan pippi')
 
rospy.spin()
