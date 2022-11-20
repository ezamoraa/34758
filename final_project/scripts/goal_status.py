#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


waypoints = [  
    [(-5.586520, -0.486526, 0.0), (0.0, 0.0, 0.0, -1.1)],
    # [(-5.007070, -3.531660, 0.0), (0.000014, 0.000010, -0.000152, 0.78)],
    [(-7.16, -3.47, 0.0), (0.0, 0.0, -0.6, 0.78)]
    # [(7, -2.88, 0.0), (0.0, 0.0, -0.66, 0.74)]
]

def goal_pose(pose):
    goal_pose_orient = MoveBaseGoal()

    goal_pose_orient.target_pose.header.frame_id = 'map'
    goal_pose_orient.target_pose.header.stamp = rospy.Time(0)
    
    # Goal position
    gp = goal_pose_orient.target_pose.pose.position
    gp.x, gp.y, gp.z = pose[0]

    # Goal orientation
    ot = goal_pose_orient.target_pose.pose.orientation
    ot.x, ot.y, ot.z, ot.w = pose[1]

    return goal_pose_orient



def main():
    rospy.init_node('QR_status_listener')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    def plan_finished():
        state = client.get_state()
        return state in [GoalStatus.ABORTED, GoalStatus.SUCCEEDED]

    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        while not plan_finished() and not rospy.is_shutdown():
            print("Running")

        print("Next waypoint")

        
if __name__ == '__main__':
    main()