#!/usr/bin/env python
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
 
 
waypoints = [  
    [(0.59, 0.7, 0.0), (0.0, 0.0, -0.01, -0.99)],
    [(-5.1, -2.03, 0.0), (0.0, 0.0, -0.9, 0.428)]
]
 
 
def goal_pose(pose):  
    goal_pose_orient = MoveBaseGoal()
    
    
    goal_pose_orient.target_pose.header.frame_id = 'map' 

    # Goal pose
    gp = goal_pose_orient.target_pose.pose.position
    gp.x, gp.y, gp.z = pose[0]

    # Goal orientation
    orient = goal_pose_orient.target_pose.pose.orientation
    orient.x, orient.y, orient.z, orient.w = pose[1]
 
    return goal_pose_orient
 
 
if __name__ == '__main__':
    rospy.init_node('patrol')
 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
   
    while True:
        for pose in waypoints:   
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            rospy.sleep(3)