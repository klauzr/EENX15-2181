#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np


# starting ROS node and subscribers
rospy.init_node('trajectory_test', anonymous=True) 
pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
rospy.sleep(0.1)


def pickup(x,y,z):
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
    trajectoryPoint.positionLeft = [x, y, z+0.03]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [20,20]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 6.0

    trajectory.append(trajectoryPoint)

    #Close gripper
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
    trajectoryPoint.positionLeft = [x, y, z+0.03]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 6.0

    trajectory.append(trajectoryPoint)

    
    #Move arm upp from table
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
    trajectoryPoint.positionLeft = [x, y, z+0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 4.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)


    """
    leftRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 180*np.pi/180, 'rzyx')

    #Test to se which axis-rotations we originaly have
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
    trajectoryPoint.positionLeft = [x, y, z+0.1]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 8.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)

    """

    #Move arms to echange location, turn the left-claw 90 degrees in the z-axis and open right claw
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    # För att ymi inte ska krocka IRL så är denna ändrad
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.05, z+0.15]
    trajectoryPoint.positionLeft = [0.34, 0.05, z+0.15]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [20,20]
    trajectoryPoint.pointTime = 6.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)
    
    #delaykod start
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    # För att ymi inte ska krocka IRL så är denna ändrad
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.05, z+0.15]
    trajectoryPoint.positionLeft = [0.34, 0.05, z+0.15]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [20,20]
    trajectoryPoint.pointTime = 6.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)
    #delay slut

    #Move arms together
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    trajectoryPoint = Trajectory_point()

    # För att ymi inte ska krocka IRL så är denna ändrad
    trajectoryPoint.positionRight = [0.35, 0.03, z+0.15]
    trajectoryPoint.positionLeft = [0.34, -0.03, z+0.15]


    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [20,20]
    trajectoryPoint.pointTime = 6.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)
    

    #Close right claw
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    # För att ymi inte ska krocka IRL så är denna ändrad
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, 0.03, z+0.15]
    trajectoryPoint.positionLeft = [0.34, -0.03, z+0.15]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 3.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)

    #Open left claw
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    # För att ymi inte ska krocka IRL så är denna ändrad
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, 0.03, z+0.15]
    trajectoryPoint.positionLeft = [0.34, -0.03, z+0.15]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [20,20]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 3.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)

    #Move arms away from each other
    leftRot = tf.transformations.quaternion_from_euler(90*np.pi/180, 90*np.pi/180, 180*np.pi/180, 'rzyx')
    rightRot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0*np.pi/180, 270*np.pi/180, 'rzyx')
    
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, z+0.15]
    trajectoryPoint.positionLeft = [0.34, 0.1, z+0.15]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [20,20]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 6.0

    #trajectory = [trajectoryPoint]
    trajectory.append(trajectoryPoint)

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
    trajectoryPoint.positionRight = [0.5, -0.2, 0.03]
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
    trajectoryPoint.orientationLeft = leftRot
    trajectoryPoint.orientationRight = rightRot
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 6.0

   
    trajectory.append(trajectoryPoint)

    #Open right gripper
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.5, -0.2, 0.05]
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

    print('Moving to start position...')
    rospy.sleep(3)


if __name__ == '__main__':
    
    pickup(0.3, 0.4, 0.01)
    rospy.spin()
