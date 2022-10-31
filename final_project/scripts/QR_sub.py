#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

def callback(msg):
    rospy.loginfo(msg.data)

def QR_listener():
    rospy.init_node('QR_status_listener', anonymous=True)
    msg = rospy.Subscriber('visp_auto_tracker/status', Int8, queue_size=1000)
    callback(msg.data)
    
    rospy.spin()

def main():
    QR_listener()

if __name__ == '__main__':
    main()