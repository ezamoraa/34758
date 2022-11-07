#!/usr/bin/env python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist


def get_move_goal(pose):
    goal_pose_orient = MoveBaseGoal()

    goal_pose_orient.target_pose.header.frame_id = 'map'
    # Goal position
    gp = goal_pose_orient.target_pose.pose.position
    gp.x, gp.y, gp.z = pose[0]

    # Goal orientation
    ot = goal_pose_orient.target_pose.pose.orientation
    ot.x, ot.y, ot.z, ot.w = pose[1]

    return goal_pose_orient


def publish_move(msg):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while pub.get_num_connections() < 1:
        # wait for a connection to publisher
        pass
    pub.publish(msg)


class QRSecretSolver:
    def __init__(self):
        self.init_waypoints = [
            [(-7.16, -3.47, 0.0), (0.0, 0.0, -0.6, 0.78)],
            [(-5.91, 1.3, 0.0), (0.0, 0.0, 0.69, 0.71)],
            [(5.36, 2.14, 0.0), (0.0, 0.0, -0.02, 0.99)],
            [(7, -2.88, 0.0), (0.0, 0.0, -0.66, 0.74)]
        ]

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.qr_info = {}


    def move_to_goal(self, pose):
        goal = get_move_goal(pose)
        self.move_client.send_goal(goal)


    def visp_detect_qr(times=1):
        for _ in range(times):
            msg = rospy.wait_for_message('visp_auto_tracker/code_message', String, 1)
            if msg == None or msg.data == '':
                return None
        return msg


    def scan_for_qr(self):
        move = Twist()
        angular_vel = 0.2 #radians/sec
        encoder = 0
        while encoder < 6.15:
            rospy.loginfo('scanning')
            is_qr_stable = self.visp_detect_qr(times=5)
            if is_qr_stable:
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


    def wander_init_waypoints(self, qr_detect_cb):
        move_goal_reached = (
            lambda : self.move_client.get_goal_status_text() == "Goal reached."
        )
        for pose in self.init_waypoints:
            print("Navigate to waypoint pose: {}".format(pose))
            # Navigate to waypoint goal
            self.move_to_goal(pose)

            # While the robot is navigating towards the goal (async)
            while not move_goal_reached():
                # Try to detect QR with camera
                qr_msg = visp_detect_qr()
                if qr_msg is not None:
                    # A valid message means we detected a QR at least once
                    stop = qr_detect_cb(qr_msg)
                    if stop:
                        # Callback instructed us to stop wandering
                        break
        else:
            raise Exception("Did not find the initial QRs while wandering!")


    def find_initial_qrs(self):
        # TODO: Find the first two QRs (at least) and solve the hidden frame
        # For these first two QRs we need to properly stop the robot to estimate
        # the world pose from the camera information

        # Callback invoked when the wander init routine detects a QR
        def initial_qr_detect_cb():
            rospy.loginfo("Detected QR")

            # Cancel any navigation goal
            self.move_client.cancel_all_goals()
            # Scan area to find and lock QR
            found_qr = self.scan_for_qr()
            if found_qr:
                # TODO: Find QR world pose
                # If this is the second QR we should return true
                # to stop wandering
            else:
                # TODO: Do we want to continue to current wander waypoint?
                # client.send_goal(goal)
                pass
            return False

        self.wander_init_waypoints(initial_qr_detect_cb)


    def find_remaining_qrs(self):
        # TODO: Find the rest of the QRs from the QRs info after
        # solving the hidden frame from the first two QRs
        pass


    def build_secret_message(self):
        # TODO: Use qr_info dictionary to build the message
        secret_msg = ""
        return secret_msg


    def find_secret_message(self):
        self.find_initial_qrs()
        self.find_remaining_qrs()

        return self.build_secret_message()
