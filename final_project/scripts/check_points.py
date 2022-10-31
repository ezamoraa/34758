#!/usr/bin/env python
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8

#asdasdas
 
 
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
 

def callback(msg):

    if msg.data == 3:
        QR_status = not(QR_status)
        print("Found QR code")


    return QR_status

def QR_listener():
    rospy.init_node('QR_status_listener', anonymous=True)
    msg = rospy.Subscriber('visp_auto_tracker/status', Int8, callback, queue_size=1000)


 
if __name__ == '__main__':
    rospy.init_node('patrol')
 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    for pose in waypoints:
        
        goal = goal_pose(pose)
        client.send_goal(goal)

        QR_status = False
        while not client.get_goal_status_text() == "Goal reached." and not rospy.is_shutdown():
            QR_listener()
            
            rospy.sleep(0.2)
        
        print("Goal reached")
        print("Next waypoint")
        

        rospy.sleep(3)