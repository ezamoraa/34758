#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

def callback(msg):
    rospy.loginfo(msg.data)

def QR_listener():
    rospy.init_node('QR_status_listener', anonymous=True)
    rospy.Subscriber('visp_auto_tracker/status', Int8, callback, queue_size=1000)
    rospy.spin()


if __name__ == '__main__':
    QR_listener()