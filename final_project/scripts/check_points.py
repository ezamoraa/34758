#!/usr/bin/env python
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

 
 
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
 

def callback(msg, QR_status):
    if msg.data == 3 or msg.data == 4:
        QR_status = not QR_status
        print("Status in callback " + str(QR_status))
        #print("Found QR code")
        # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # while pub.get_num_connections() < 1:
        #     print("Waiting for publisher connection")
        # move = Twist()
        # move.linear.x = 0
        # move.linear.y = 0
        # move.angular.z = 0
        # pub.publish(move)
        
    return QR_status

def QR_listener(QR_status):
    msg_y = rospy.Subscriber('visp_auto_tracker/status', Int8, callback, QR_status, queue_size=1)
    print(msg_y)
    return QR_status

def main():
    rospy.init_node('QR_finder', anonymous=True)
 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    for pose in waypoints:
        
        goal = goal_pose(pose)
        client.send_goal(goal)
        QR_status = False
        while not client.get_goal_status_text() == "Goal reached." and not rospy.is_shutdown():
            QR_listener(QR_status)
            print(QR_status)
            rospy.sleep(0.2)
        
        if QR_status == True:
            print("Stop robot")
            exit

        print("Goal reached")
        print("Next waypoint")
        

        rospy.sleep(3)


if __name__ == '__main__':
    main()