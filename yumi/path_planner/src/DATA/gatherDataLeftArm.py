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
dataStr = " "
 
def callback_stop(bool):
    global stop
    stop = bool.data
 
def callback_start(bool):
    print('Gathering data for left arm')
    tfListener = tf.TransformListener()  
    time.sleep(1)
 
    i = 0
 
    while (not stop): 
        #(posRight, orientationRight) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (posLeft, orientationLeft) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
    
        if i == 0:
            global dataStr
            dataStr =  str(posLeft[0]) + "\t" +  str(posLeft[1]) + "\t" + str(posLeft[2]) + "\n"
            i = i + 1
 
        else:
            dataStr = dataStr +  str(posLeft[0]) + "\t" +  str(posLeft[1]) + "\t" + str(posLeft[2]) + "\n"
            time.sleep(0.1)
 
    
        if(stop):
            #Kör översta för att spara simulrad data och undre för att spara verklig data.
            #OBS! Tänk på att döpa om fillnamnen!
            #file2write=open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataYuMiOnly/Left/Lrun2Lx30y20.txt",'a')
            #file2write=open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataYuMiOnlySim/Right/Lsim6Rx50y0.txt",'a')
            #file2write=open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataReal/Lrun5L.txt",'a')
            #file2write=open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataRealSim/Left/Lsim5L.txt",'a')
            file2write=open("/home/pontus/catkin_ws/src/yumi_dlo_thesis/path_planner/src/dataYuMiMarkerSim/Right/Lsim4R.txt",'a')

            file2write.write(dataStr)
            file2write.close()

            print('Data gathered')
            #print(dataStr)
            break
"""
    if stop:
        fig = plt.figure()
        ax = plt.axes(projection="3d")
 
        #refV = 
        #refVX = ref[:,0]
        #refVY = ref[:,1]
        #refVZ = ref[:,2]
 
        ax.text2D(0.05, 0.95, "YuMis left arm", transform=ax.transAxes)

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
        
"""
 
# starting ROS node and subscribers
rospy.init_node('gather_data', anonymous=True) 
sub_start = rospy.Subscriber('/start', Bool, callback_start)
sub_stop = rospy.Subscriber('/stop', Bool, callback_stop)
 
print('Ready to collect data')
 
rospy.spin()

