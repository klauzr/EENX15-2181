#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray


def callback(data_array):
    print('Marker 0: ' + 'x = ' + str(data_array.data[0]) + ', y = ' + str(data_array.data[1]))
    print('Marker 1: ' + 'x = ' + str(data_array.data[2]) + ', y = ' + str(data_array.data[3]))
    print('\nDiffx = ' + str(data_array.data[0] - data_array.data[2]))
    print('\nDiffy = ' + str(data_array.data[1] - data_array.data[3]))

rospy.init_node("coord_collector", anonymous=True)
sub = rospy.Subscriber('/coord_topic', Float64MultiArray, callback)
rospy.sleep(0.1)

if __name__ =="__main__":
    rospy.spin()
