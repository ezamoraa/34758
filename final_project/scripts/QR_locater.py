#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8



class GoToWaypoint(object):

    def __init__(self, pose):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.pose = pose
        self.goal_pose_orient = MoveBaseGoal()
        
        self.goal_pose_orient.target_pose.header.frame_id = 'map'
        
        self.gp = self.goal_pose_orient.target_pose.pose.position
        self.gp.x, self.gp.y, self.gp.z = self.pose[0]

        self.orient = self.goal_pose_orient.target_pose.pose.orientation
        self.orient.x, self.orient.y, self.orient.z, self.orient.w = self.pose[1]

        self.QR_status = False


    def callback(self, msg):
        if msg.data == 3:
            self.QR_status = not self.QR_status
            print("Found QR code")
        

    def QR_listerner(self):
        rospy.Subscriber('visp_auto_tracker/status', Int8, self.callback, queue_size=10)


    def main(self):
        self.client.send_goal(self.goal_pose_orient)
        
        while not self.client.get_goal_status_text() == "Goal reached." and not rospy.is_shutdown():
            self.QR_listerner()
            if self.QR_status == True:
                break

            rospy.sleep(0.2)

        return self.QR_status


if __name__ == '__main__':

    rospy.init_node('QR_status_listener')
    waypoints = [  
        [(0.59, 0.7, 0.0), (0.0, 0.0, -0.01, -0.99)],
        [(-5.1, -2.03, 0.0), (0.0, 0.0, -0.9, 0.428)]
    ]

    for waypoint in waypoints:
        go_to_waypoint = GoToWaypoint(waypoint)
        go_to_waypoint.main()
        
        if go_to_waypoint.main() == True:
            break
        else:
            print("Goal reached")
            print("Next waypoint")
        

        rospy.sleep(3)