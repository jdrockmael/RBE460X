#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
def callback(msg):
    # # values at 0 degree
    # print msg.ranges[0]
    # # values at 90 degree
    # print msg.ranges[360]
    # # values at 180 degree
    # print msg.ranges[719]
    rospy.loginfo(msg.ranges[0])
    time.sleep(2)
rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()