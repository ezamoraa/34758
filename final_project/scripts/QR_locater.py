#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String



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


    def check_status(self,n=1):
        
        for _ in range(n):
            msg = rospy.wait_for_message('visp_auto_tracker/status', Int8, 3)

            if msg == None:
                return False
            elif msg.data not in [3,4]:
                return False

        return True, msg


    def execute2(self):
        self.client.send_goal(self.goal_pose_orient)

        while not rospy.is_shutdown():
            
            status = self.check_status()
            if status == True:
                self.client.cancel_goal()
                print(status[1])
                return True


            elif self.client.get_goal_status_text() == "Goal reached.":
                return False

        return False  

    def execute(self):


        self.client.send_goal(self.goal_pose_orient)

        while not rospy.is_shutdown():
            if self.check_status() == True:
                self.client.cancel_goal()

                if self.check_status(5) == True:

                    return True
                else:
                    #Rotate slowly
                    pass

            elif self.client.get_goal_status_text() == "Goal reached.":
                return False

        return False  






def main():
    rospy.init_node('QR_status_listener')

    waypoints = [  
        [(0.59, 0.7, 0.0), (0.0, 0.0, -0.01, -0.99)],
        [(-5.1, -2.03, 0.0), (0.0, 0.0, -0.9, 0.428)]
    ]

    for waypoint in waypoints:
        go_to_waypoint = GoToWaypoint(waypoint)
        found_QR = go_to_waypoint.execute2()

        if found_QR == True:
            break
        
        print("Goal reached")
        print("Next waypoint")
        

        rospy.sleep(3)
    

if __name__ == '__main__':
    main()