#!/usr/bin/env python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
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


def visp_detect_qr(times=1, timeout=2):
    for _ in range(times):
        msg = rospy.wait_for_message('visp_auto_tracker/code_message', String, timeout)
        if msg == None or msg.data == '':
            return None
    return msg


class QRSecretSolver:
    def __init__(self, init_waypoints):
        self.init_waypoints = init_waypoints

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.qr_info = {}


    def move_to_goal(self, pose):
        goal = get_move_goal(pose)
        self.move_client.send_goal(goal)


    def scan_for_qr(self):
        move = Twist()
        angular_vel = 0.2 #radians/sec
        encoder = 0

        while encoder < 6.15:
            rospy.loginfo('scanning')
            is_qr_stable = visp_detect_qr(times=5)
            if is_qr_stable:
                rospy.logdebug("qr is stable")
                return True

            rospy.logdebug("qr is not stable")
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
            rospy.loginfo("Navigate to waypoint pose: {}".format(pose))
            # Navigate to waypoint goal
            self.move_to_goal(pose)

            # While the robot is navigating towards the goal (async)
            while not move_goal_reached():
                # Try to detect QR with camera
                qr_msg = visp_detect_qr()
                if qr_msg is not None:
                    # A valid message means we detected a QR at least once
                    self.move_client.cancel_all_goals()

                    rospy.loginfo("Detected QR: msg={}".format(qr_msg))
                    stop = qr_detect_cb(qr_msg)
                    if stop:
                        # Callback instructed us to stop wandering
                        return
                    # Resume last navigation goal
                    self.move_to_goal(pose)

        raise Exception("Did not find the initial QRs while wandering!")


    def find_initial_qrs(self):
        # Find the first two QRs (at least) and solve the hidden frame
        # For these first two QRs we need to properly stop the robot to
        # estimate the world pose from the camera information

        # Callback invoked when the wander init routine detects a QR
        def initial_qr_detect_cb(qr_msg):
            stop = False

            # Scan area to find and lock QR
            found_qr = self.scan_for_qr()
            if found_qr:
                # TODO: Find QR world pose
                # If this is the second QR we should return true
                # to stop wandering (do it right away for now)
                stop = True

            return stop

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
