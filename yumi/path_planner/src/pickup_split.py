#!/usr/bin/env python3
 
import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


#publishtest = rospy.Publisher("test_birdge", Float64MultiArray, queue_size=10)

 
 
def callback(msg):
   print('Received coordinates, calculating positions.')
   print('Fixed marker: ' + str(msg.data[0]) + ', ' + str(msg.data[1]))
   print('AGV: ' + str(msg.data[2]) + ', ' + str(msg.data[3]))

   FIXED_OFFSET_X = np.float64(0.525)
   FIXED_OFFSET_Y = np.float64(0.01)

   fixed_marker_x = np.float64(msg.data[0])
   fixed_marker_y = np.float64(msg.data[1])	

   agv_pos_x = np.float64(msg.data[2])
   agv_pos_y = np.float64(msg.data[3])

   diff_x = agv_pos_x - fixed_marker_x 
   diff_y = agv_pos_y - fixed_marker_y

#Multiplied by -1 to set the camera d
# irection to yumi direction
   diff_x = (-1)*diff_x
   diff_y = (-1)*diff_y
#yumis x-led aer kamerans y-led och vice versa
   pick_pos_x = FIXED_OFFSET_X + diff_y
   pick_pos_y = FIXED_OFFSET_Y + diff_x

   #print(type(diff_x))
   #print(type(pick_pos_y))
#The if-statement prevents yumi from reaching too far   
   if(pick_pos_x >= 0 and pick_pos_x <= 0.6 and abs(pick_pos_y - 0.1) <= 0.6):
       pickup1(pick_pos_x, pick_pos_y, 0.0125)
       rospy.sleep(24)
       pickup2(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(15)
       pickup3(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(9)
       pickup4(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(6)
       pickup5(pick_pos_x, pick_pos_y, 0.01)
       rospy.sleep(30)
       print('Sequence done.')
   else:
       print('Object too far away.')


def pickup1(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   #msg.forceControl = 0
   #msg.maxForce = 0
 
   #Move to start position
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
 
   #Move left arm above object and open left claw
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [x, y, z+0.1]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   #Move arm down to object
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [x, y, z+0.017]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   #Close gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [x, y, z+0.017]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('1: Moving to pick up location at x = ' + str(x) + ', y = ' + str(y))
   
 
def pickup2(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'

       #Move arm upp from table
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [x, y, z+0.1]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 4.0
 
   trajectory = [trajectoryPoint]
 
   #Move arms to echange location, turn the left-claw 90 degrees in the z-axis and open right claw
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
  
   # För att ymi inte ska krocka IRL så är denna ändrad
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.355, -0.1, z+0.15]
   trajectoryPoint.positionLeft = [0.35, 0.1, z+0.1375]
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
 
   print('2: Moving towards exchange location.')
 
def pickup3(x,y,z):
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   #msg.forceControl = 0
   #msg.maxForce = 0
  
  
   #Move arms together
 
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
  
   trajectoryPoint = Trajectory_point()
 
   # För att ymi inte ska krocka IRL så är denna ändrad
   trajectoryPoint.positionRight = [0.355, 0.005, z+0.15]
   trajectoryPoint.positionLeft = [0.35, -0.005, z+0.1375]
 
 
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
   
 
   print('3: Moving arms to exchange location.')

 
def pickup4(x, y, z):
  
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   #msg.forceControl = 0
   #msg.maxForce = 0
  
   #Close right claw
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
  
   # För att ymi inte ska krocka IRL så är denna ändrad
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.355, 0.01, z+0.15]
   trajectoryPoint.positionLeft = [0.35, -0.01, z+0.1375]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 3.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 
   #Open left claw
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
  
   # För att ymi inte ska krocka IRL så är denna ändrad
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.355, 0.01, z+0.15]
   trajectoryPoint.positionLeft = [0.35, -0.01, z+0.1375]
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
 
   print('4. Moving cube to the other gripper.')

 
def pickup5(x,y,z):
 
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'

  
  
   #Move arms away from each other
   leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
  
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.355, -0.1, z+0.15]
   trajectoryPoint.positionLeft = [0.35, 0.1, z+0.1375]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [20,20]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory = [trajectoryPoint]
   #trajectory.append(trajectoryPoint)
 
   #Move right arm to above worktable, left arm to startpos and close left claw
   leftRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
  
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, z+0.1]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
  
   trajectory.append(trajectoryPoint)
 
   #Move right arm down to worktable
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.02]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
 
   trajectory.append(trajectoryPoint)
 
   #Open right gripper
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.02]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   #Move right arm up from object
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.5, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [20,20]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   #Move right arm to start position and close right gripper
   rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')
  
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
   trajectoryPoint.orientationLeft = leftRot
   trajectoryPoint.orientationRight = rightRot
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 6.0
 
   trajectory.append(trajectoryPoint)
 
   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
 
   print('5. Putting down cube and moving back to starting position.')

def reset_position():
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
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
     
   print("Resetting position.")

def calibrate_marker():
   msg = Trajectory_msg()
   msg.header.stamp = rospy.Time.now()
   msg.mode = 'individual'
   
   reset_position()
   
   trajectoryPoint = Trajectory_point()
   trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
   trajectoryPoint.positionLeft = [0.525, 0.01, 0.025]
   trajectoryPoint.orientationLeft = [1,0,0,0]
   trajectoryPoint.orientationRight = [1,0,0,0]
   trajectoryPoint.gripperLeft = [0,0]
   trajectoryPoint.gripperRight = [0,0]
   trajectoryPoint.pointTime = 10.0

   trajectory = [trajectoryPoint]

   msg.trajectory = trajectory
   pub.publish(msg)
   rospy.sleep(0.1)
   pub.publish(msg)
     
   print("Calibrating.")

def callback1(status):
    #agv_stopped = status.data
       print('Received status from AGV_status')

def callback2(status):
    #agv_stopped = status.data
       print('Received status from /hrp/hrp1/AGV_status')

# starting ROS node and subscribers
rospy.init_node('trajectory_test', anonymous=True)
pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
print('starting subscribers')
sub = rospy.Subscriber('/coord_topic', Float64MultiArray, callback)
sub_status = rospy.Subscriber('AGV_status', Float64, callback1)
sub1 = rospy.Subscriber('/hrp/hrp1/AGV_status', Float64, callback2)
rospy.sleep(0.1)

 
if __name__ == '__main__':

   #msg = Float64MultiArray()
   #list = [float(1)]
   #msg.data = list
   #publishtest.publish(msg)
   #print("testade hejsan go")
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
      
   calibrate_marker()
   
   
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
   rospy.spin()

