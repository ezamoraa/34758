#!/usr/bin/env python

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist

waypoints = [  
    [(-7.16, -3.47, 0.0), (0.0, 0.0, -0.6, 0.78)],
    [(-5.91, 1.3, 0.0), (0.0, 0.0, 0.69, 0.71)],
    [(5.36, 2.14, 0.0), (0.0, 0.0, -0.02, 0.99)],
    [(7, -2.88, 0.0), (0.0, 0.0, -0.66, 0.74)]
]

def goal_pose(pose):
    goal_pose_orient = MoveBaseGoal()

    goal_pose_orient.target_pose.header.frame_id = 'map'
    # Goal position
    gp = goal_pose_orient.target_pose.pose.position
    gp.x, gp.y, gp.z = pose[0]

    # Goal orientation
    ot = goal_pose_orient.target_pose.pose.orientation
    ot.x, ot.y, ot.z, ot.w = pose[1]

    return goal_pose_orient

        
def check_status(n=1):
    for _ in range(n):
        msg = rospy.wait_for_message('visp_auto_tracker/code_message', String, 1)
        if msg == None:
            return False
        elif msg.data == '':
            return False
    return True


def publish_move(msg):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while pub.get_num_connections() < 1:
        # wait for a connection to publisher
        pass
    pub.publish(msg)


def scan_for_qr():
    move = Twist()
    angular_vel = 0.2 #radians/sec
    encoder = 0
    while encoder < 6.15:
        rospy.loginfo('scanning')
        is_qr_stable = check_status(5)
        if is_qr_stable == True:
            print("qr is stable")
            move.angular.z = 0
            publish_move(move)
            return True
        else:
            print("qr is not stable")
            move.angular.z = angular_vel
            publish_move(move)
            rospy.sleep(1.)
            move.angular.z = 0
            publish_move(move)
            rospy.sleep(2.)

        encoder += angular_vel

    return False


def main():
    rospy.init_node('QR_status_listener')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        found_qr = False
        while not client.get_goal_status_text() == "Goal reached." and found_qr == False:
            found_qr = check_status()
            if found_qr == True:
                rospy.loginfo("Found QR")
                client.cancel_all_goals()
                found_qr = scan_for_qr()
                if found_qr == True:
                    break
                else:
                    client.send_goal(goal)

        if found_qr == True:
            break
        
        print("Next waypoint")


        
if __name__ == '__main__':
    main()