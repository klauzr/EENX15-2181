#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import math
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool

# Constants defining the robot arms range from the innermost joint
# Based on values from datasheet, a margin of 5 cm has been added
# Upper bound = 0.681 - 0.06 - 0.05 = 0.571
# Lower bound = 0.274 - 0.06 + 0.05 = 0.264
UPPER_BOUND = 0.576
LOWER_BOUND = 0.259

# Define the innermost joints offsets from Yumis origin
LEFT_ARM_OFFSET_X = 0.06
LEFT_ARM_OFFSET_Y = 0.075
RIGHT_ARM_OFFSET_X = 0.06
RIGHT_ARM_OFFSET_Y = -0.075  

# Define the fixed markers offset from the origin
FIXED_MARKER_OFFSET_X = np.float64(0.532)
FIXED_MARKER_OFFSET_Y = np.float64(0.0)

# Define the AGV markers offset from the object
AGV_MARKER_OFFSET_Y = 0 #np.float64(0.15)

# a
sequence_running = False
agv_stopped = False

# Callback run when coordinates are received
def callback(msg):
   global sequence_running
   global agv_stopped
   
   if(not sequence_running and agv_stopped):
      sequence_running = True
      print('Received coordinates, calculating positions.')
      print('Fixed marker: ' + str(msg.data[0]) + ', ' + str(msg.data[1]))
      print('AGV: ' + str(msg.data[2]) + ', ' + str(msg.data[3]))

      # Saves the x and y position of the fixed marker
      fixed_marker_x = np.float64(msg.data[0])
      fixed_marker_y = np.float64(msg.data[1]) 
      
      # Saves the x and y position of the AGVs marker
      agv_marker_x = np.float64(msg.data[2])
      agv_marker_y = np.float64(msg.data[3])

      # Calculates the distance between the markers
      diff_x = agv_marker_x - fixed_marker_x
      diff_y = agv_marker_y - fixed_marker_y

      # Multiply by -1 to set the camera direction to the same direction as Yumi
      diff_x = (-1)*diff_x
      diff_y = (-1)*diff_y

      print('Diffx = ' + str(diff_x))
      print('Diffy = ' + str(diff_y))
      # Calculates the target position
      # Yumi and the camera have different coordinate frames, the calculations below takes this into consideration
      pick_pos_x = FIXED_MARKER_OFFSET_X + diff_y
      pick_pos_y = FIXED_MARKER_OFFSET_Y + diff_x - AGV_MARKER_OFFSET_Y

      print(pick_pos_x)
      #print(type(diff_x))
      #print(type(pick_pos_y))
      
      # Picks up with the arm closest to the object
      if(pick_pos_y >= 0):
         pickup_left(pick_pos_x, pick_pos_y)
      elif(pick_pos_y < 0):
         pickup_right(pick_pos_x, pick_pos_y)
   
  

def pickup_left(pick_pos_x, pick_pos_y):
   # Calculate the objects distance from the innermost joint
   # Offsets are removed from the x and y values to account for the innermost joints distance from the origin, the values are based on datasheets
   object_distance = math.sqrt(math.pow((pick_pos_x - LEFT_ARM_OFFSET_X), 2) + math.pow((pick_pos_y - LEFT_ARM_OFFSET_Y), 2))

   global sequence_running

   # The if-statement prevents yumi from reaching too far 
   # The first condition may need change or be removed, 0.2 is a random value
   if(pick_pos_x >= -0.2 and object_distance >= LOWER_BOUND and object_distance <= UPPER_BOUND):
       msg = Bool()
       msg.data = True
       pub_start.publish(msg)

       
       pickup1_left(pick_pos_x, pick_pos_y, 0.0125)
       rospy.sleep(18)
       pickup2_left(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(6)
       pickup3_left(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(15)
       pickup4_left(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(9)
       pickup5_left(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(6)
       pickup6_left(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(30)

       msg.data = True
       rospy.sleep(5)
       pub_stop.publish(msg)
       sequence_running = False


       print('Sequence done.')
   else:
       print('Object cannot be reached with left arm.')
       sequence_running = False
       return   
   
   
def pickup_right(pick_pos_x, pick_pos_y):
   # Calculate the objects distance from the innermost joint
   # Offsets are removed from the x and y values to account for the innermost joints distance from the origin, values based on datasheets
   object_distance = math.sqrt(math.pow((pick_pos_x - RIGHT_ARM_OFFSET_X), 2) + math.pow((pick_pos_y - RIGHT_ARM_OFFSET_Y), 2))
   
   global sequence_running

   # The if-statement prevents yumi from reaching too far 
   # The first condition may need change or be removed, 0.2 is a random value
   if(pick_pos_x >= -0.2 and object_distance >= LOWER_BOUND and object_distance <= UPPER_BOUND):
       msg = Bool()
       msg.data = True
       pub_start.publish(msg) 
       
       pickup1_right(pick_pos_x, pick_pos_y, 0.0125)
       rospy.sleep(18)
       pickup2_right(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(6)
       pickup3_right(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(15)
       pickup4_right(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(9)
       pickup5_right(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(6)
       pickup6_right(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(30)
       

       msg.data = True
       rospy.sleep(5)
       pub_stop.publish(msg)
       sequence_running = False

       print('Sequence done.')
   else:
       print('Object cannot be reached with right arm.') 
       sequence_running = False
       return     
   

def pickup1_left(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   # Move to starting position
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
 
   # Move left arm above object 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [x, y, 0.05]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move arm down to object
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [x, y, 0.01]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('1: Picking up at x = ' + str(x) + ', y = ' + str(y))
 

def pickup2_left(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual' 

   # Close gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [x, y, 0.01]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('2: Close gripper.')
 
   
 
def pickup3_left(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'

   # Move arm up from table
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [x, y, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 4.0
 
   trajectory = [trajectoryPoint]
 
   #Move arms towards exchange location, turn both grippers 90 degrees inwards
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.05, 0.25]
   trajectoryPoint.positionLeft = [0.35, 0.05, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   #trajectory = [trajectoryPoint]
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('3: Moving towards exchange location.')
 
def pickup4_left(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')

   # Move both grippers closer to each other
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.005, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.005, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('4: Moving arms to exchange location.')

 
def pickup5_left(x, y, z):
 
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   # Close right claw
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.01, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.01, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 3.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 

   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   #Open left claw
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.01, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.01, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 3.0
 
   #trajectory = [trajectoryPoint]
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('5. Moving cube to the other gripper.')

 
def pickup6_left(x,y,z):
 
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual' 
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   #Move arms away from each other
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.1, 0.25]
   trajectoryPoint.positionLeft = [0.35, 0.1, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 
   # Move right arm above drop location, move left arm to starting position 
   leftRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.1]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
 
   trajectory.append(trajectoryPoint)
 
   # Move right arm down to drop location
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.01]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
 
   trajectory.append(trajectoryPoint)
 
   # Open right gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.01]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move right arm up from work table
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move right arm to starting position
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('6. Putting down cube and moving back to starting position.')

def pickup1_right(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   # Move to starting position
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
 
   # Move right arm above object
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [x, y, 0.05]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move arm down to object
   trajectoryPoint = Trajectory_point()
   #trajectoryPoint.positionRight = [x, y, -0.01]
   trajectoryPoint.positionRight = [x, y, 0.01]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('1: Picking up at x = ' + str(x) + ', y = ' + str(y))

#gor for andra armen
def pickup2_right(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual' 
   
   # Close gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [x, y, 0.01]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('1: Picking up at x = ' + str(x) + ', y = ' + str(y))
   
 
def pickup3_right(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'

   # Move arm up from table
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [x, y, z+0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 4.0
 
   trajectory = [trajectoryPoint]
 
   #Move arms towards exchange location, turn both grippers 90 degrees inwards
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.05, 0.25]
   trajectoryPoint.positionLeft = [0.35, 0.05, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   #trajectory = [trajectoryPoint]
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('2: Moving towards exchange location.')
 
def pickup4_right(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')

   # Move both grippers closer to each other
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.005, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.005, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('3: Moving arms to exchange location.')

 
def pickup5_right(x, y, z):
 
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   # Close left claw
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.01, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.01, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 3.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 

   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   #Open right claw
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, 0.01, 0.25]
   trajectoryPoint.positionLeft = [0.35, -0.01, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 3.0
 
   #trajectory = [trajectoryPoint]
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('4. Moving cube to the other gripper.')

 
def pickup6_right(x,y,z):
 
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual' 
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
 
   #Move arms away from each other
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.1, 0.25]
   trajectoryPoint.positionLeft = [0.35, 0.1, 0.25]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 
   # Move left arm above drop location, move right arm to starting position
   leftRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.5, 0.2, z+0.1]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
 
   trajectory.append(trajectoryPoint)
 
   # Move left arm down to drop location
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.5, 0.2, 0.005]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
 
   trajectory.append(trajectoryPoint)
 
   # Open left gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.5, 0.2, 0.005]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move left arm up from work table
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.5, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   # Move left arm to starting position
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
 
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('5. Putting down cube and moving back to starting position.')



# Use this method to return Yumis arms to a chosen starting position
def reset_position():
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.15, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 4.0
 
   trajectory = [trajectoryPoint]

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
   
   rospy.sleep(4)
   print("Resetting position.")


# Use this method to move the left gripper to a chosen position
# The center of the fixed marker should then be placed below the grippers tip
# Write the chosen x and y positions in FIXED_OFFSET_X and FIXED_OFFSET_Y
def calibrate_marker():
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   
   reset_position()
   
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.15, 0.2]
   trajectoryPoint.positionLeft = [0.527, 0.005, 0.025]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0

   trajectory = [trajectoryPoint]

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
     
   print("Calibrating.")

def callback1(status):
    i =2 #agv_stopped = status.data
       #print('Received status from AGV_status')

def callback2(status):
   #print('Received status from /hrp/hrp1/AGV_status')
   if(status.data == float(1)):
      #print('Ready to pick')
      global agv_stopped
      agv_stopped = True
    

# Starts a node, publisher and subscriber
rospy.init_node('yumi_pickup', anonymous=True)
pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)

pub_start = rospy.Publisher('/start', Bool, queue_size=1)
pub_stop = rospy.Publisher('/stop', Bool, queue_size=1)

sub = rospy.Subscriber('/coord_topic', Float64MultiArray, callback,queue_size=2)
agv_sub = rospy.Subscriber('AGV_status', Float64, callback1)
agv_hrp_sub = rospy.Subscriber('/hrp/hrp1/AGV_status', Float64, callback2)

rospy.sleep(0.1)
print('Node has been started.')  
 
if __name__ == '__main__':


   '''
   FIXED_OFFSET_X = 0.525
   FIXED_OFFSET_Y = 0.01


   fixed_marker_x = 1
   fixed_marker_y = 1
   #fixed_marker_x = msg[0]
   #fixed_marker_y = msg[1]    


   agv_pos_x = 0.9
   agv_pos_y = 1.25
   #agv_pos_x = msg[2]
   #agv_pos_y = msg[3]

   diff_x = agv_pos_x - fixed_marker_x
   diff_y = agv_pos_y - fixed_marker_y

   pick_pos_x = FIXED_OFFSET_X + diff_x
   pick_pos_y = FIXED_OFFSET_Y + diff_y

   pickup1(pick_pos_x, pick_pos_y, 0.01)
   rospy.sleep(24)
   pickup2(pick_pos_x, pick_pos_y, 0.01)
   rospy.sleep(25)
   pickup3(pick_pos_x, pick_pos_y, 0.01)
   rospy.sleep(11)
   pickup4(pick_pos_x, pick_pos_y, 0.01)
   rospy.sleep(11)
   pickup5(pick_pos_x, pick_pos_y, 0.01)
      

   
   pickup1(0.3, 0.4, 0.01)
   rospy.sleep(24)
   pickup2(0.3, 0.4, 0.01)
   rospy.sleep(25)
   pickup3(0.3, 0.4, 0.01)
   rospy.sleep(11)
   pickup4(0.3, 0.4, 0.01)
   rospy.sleep(11)
   pickup5(0.3, 0.4, 0.01)
   '''   
   #calibrate_marker()
   #reset_position()
   
   pickup_right(0.40,-0.30)


   rospy.spin()
