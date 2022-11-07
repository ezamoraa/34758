import rospy
import numpy as np
from std_msgs.msg import Int8, String
from final_project.msg import qr 
from std_msgs.msg import Int8, String
import actionlib
import math
import geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

def rotate():
    global cmd_vel_pub, state
    camera_angle = 0.523 # 30 deg
    twist = Twist()
        
    twist.angular.z = camera_angle
    cmd_vel_pub.publish(twist)

def stop():
    global cmd_vel_pub, state
    twist = Twist()
    #print(twist)
    #twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

def qr_status_callback(msg):
    global status_qr
    #print(msg)
    status_qr = msg.data
  
def qr_msg_callback(msg):
    global qr_message
    qr_message = msg.data

def qr_tf_cov_callback(msg):
    global ob_postion_relative
    ob_postion_relative = msg.pose

def parse_message():
    global word
    # Read message
    content = str(qr_message) #.copy()
    
    values = []
    for iter in range(5):
        idx_start = content.find('=')+1
        idx_end = content.find('\n')-1
        value = content[idx_start:idx_end]
        content = content[idx_end+2:]
        values.append(value)

    value = content[2:]
    values.append(value)
        
    id = int(values[4])
		qr_msg.x_current = float(values[0])
		qr_msg.y_current = float(values[1])
		qr_msg.x_next = float(values[2])
		qr_msg.y_next = float(values[3])
		qr_msg.id = id
		qr_msg.letter = values[5]

    return qr_msg