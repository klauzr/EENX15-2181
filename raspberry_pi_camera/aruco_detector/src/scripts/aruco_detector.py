#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import time
from std_msgs.msg import Float64MultiArray



# Creates a node and a publisher
# The subscriber is created further down
rospy.init_node("detector_node", anonymous=True)
print("Node started")
pub = rospy.Publisher('/coord_topic', Float64MultiArray, queue_size=10)
msg = Float64MultiArray()


# Calibration and distortion matrix
calibMatrix = np.array([[491.34272566,   0,         321.31198884],
 [  0,         489.2337054,  245.14729927],
 [  0,           0,           1,        ]])
distMatrix = np.array([[ 0.18215647, -0.49787075,  0.00181935, -0.00299326,  0.36826704]])

# Size of the aruco marker (in cm?)
marker_size = 10

# The aruco library used when detecting markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# 
detector_parameters = aruco.DetectorParameters_create()
detector_parameters.doCornerRefinement = True
detector_parameters.cornerRefinementMinAccuracy = 0.01

# For capturing video 
video_stream = cv2.VideoCapture(0)

# Set video stream resolution, e.g. 640x480, 480x360
# Higher resolution makes the video slower 
video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Tolerance used when checking if AGV has stopped (in meters)
tol = 0.005 

agv_stopped = False
id1_old_x = None
id1_old_y = None
coordinates = 4*[None]

'''
coordinates = 6*[None]
coordinates[0] = 1
coordinates[1] = 1
coordinates[2] = 0.9
coordinates[3] = 1.25
coordinates[4] = 1
coordinates[5] = 2

msg.data = coordinates
while True: 
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)
    print('sent msg')
    print(type(coordinates[3]))

    if rospy.is_shutdown():
         print('shut down')        
         break
'''    

def callback(status):
    #agv_stopped = status.data
    print('Received status from AGV')


sub = rospy.Subscriber('/AGV_status', Float64MultiArray, callback)

while True:
    # Saves images from the video stream and converts it into a grayscale image
    # ret is currently unused (check if removable)
    ret, frame = video_stream.read()
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get the corner positions of each (visible) marker in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=detector_parameters)

    if (ids is not None): 

         for i in range(0, len(ids)):
              # Calculates the markers positions and places each position in an array
              result = aruco.estimatePoseSingleMarkers(corners[i], marker_size, calibMatrix, distMatrix)
              rvec, tvec = result[0][0,0,:], result[1][0,0,:]
              #print("Marker found, printing coordinates for id = " + str(ids[i]))
              #print("x-position = ", round(tvec[0], 1), ", y-position = ", round(tvec[1], 1), ", z-position = ", round(tvec[2], 1))

              
              aruco.drawDetectedMarkers(frame, corners)
              #aruco.drawAxis(frame, calibMatrix, distMatrix, rvec, tvec, 10)


              x_coord = tvec[0]/100
              y_coord = tvec[1]/100

              if (ids[i] == 0):
                   coordinates[0] = x_coord
                   coordinates[1] = y_coord      

              # ANDRADE HAR TILL != 0 ISTALLET FOR == 1 
              elif (ids[i] != 0):
                   coordinates[2] = x_coord
                   coordinates[3] = y_coord  
                   id1_current_x = x_coord
                   id1_current_y = y_coord        
                   
                   #kommentera ut sen
                   if((id1_old_x == None) or (abs(id1_current_x - id1_old_x) > tol and abs(id1_current_y - id1_old_y) > tol)):
                        print("AGV moving")                        
                        start_time = time.time()
                        agv_stopped = False      
                   elif((abs(id1_current_x - id1_old_x) < tol and abs(id1_current_y - id1_old_y) < tol) and (time.time() - start_time) > 3): 
                        agv_stopped = True                   
                        msg.data = coordinates

                   id1_old_x = x_coord
                   id1_old_y = y_coord     
                   #

              if (agv_stopped and (not None in coordinates)):
                   #msg.data = coordinates  #remove this if old code is restored
                   pub.publish(msg)
                   agv_stopped = False
                   start_time = 0               #kan vara fel, alt. time.time()
                   print("published")
                   print("Marker found, printing coordinates for id = " + str(ids[i]))
                   print("x-position = ", x_coord, ", y-position = ", y_coord)
    else:
         print("No marker found")
 
    #Opens a window that shows the image stored in the variable frame
    cv2.imshow("Video stream", frame)
    #This is used to refresh the image displayed by cv2.imshow (maybe)
    cv2.waitKey(1)
           
    if rospy.is_shutdown():
         print('Marker detector has been shut down.')        
         break





if __name__ =="__main__":
   
    rospy.spin()
