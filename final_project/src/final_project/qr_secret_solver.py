#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import numpy as np
import math
import tf


# Total number of QR codes to find
NUM_QR_IDS = 5


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


def visp_detect_qr(times=1, timeout=3, filter_dict=None):
    for _ in range(times):
        msg = rospy.wait_for_message('visp_auto_tracker/code_message', String, timeout)
        if msg is None or msg.data == '':
            return None

        if filter_dict and (QRInfo(msg.data).qr_id in filter_dict):
            return None

    return msg


class QRInfo:
    def __init__(self, qr_msg):
        qr_msg = qr_msg.split("\r\n")

        self.current_pos = tuple(float(x.split('=')[-1]) for x in qr_msg[:2])
        self.next_pos = tuple(float(x.split('=')[-1]) for x in qr_msg[2:4])
        self.secret_letter = qr_msg[5][-1]
        self.qr_id = int(qr_msg[4][-1])
        self.world_pose = None


class QRSecretSolver:
    def __init__(self, init_waypoints):
        self.init_waypoints = init_waypoints

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.qr_info_initial = {}
        self.qr_info_all = {}

        self.tl = tf.TransformListener()

        self.world_frame = "map"
        self.camera_frame = "camera_optical_link"

    def add_qr_info_all(self, qr_msg):
        qr_info = QRInfo(qr_msg.data)
        self.qr_info_all[qr_info.qr_id] = qr_info
        return qr_info

    def move_to_goal(self, pose):
        goal = get_move_goal(pose)
        self.move_client.send_goal(goal)

    def scan_for_qr(self, detect_times=5, qr_detect_filter_dict=None):
        move = Twist()
        angular_vel = 0.2  # radians/sec
        encoder = 0

        while encoder < 6.15:
            rospy.loginfo('scanning')
            qr_msg = visp_detect_qr(times=detect_times,
                                    filter_dict=qr_detect_filter_dict)
            if qr_msg:
                rospy.logdebug("qr is stable")
                return qr_msg

            rospy.logdebug("qr is not stable")
            move.angular.z = angular_vel
            publish_move(move)
            rospy.sleep(1.)
            move.angular.z = 0
            publish_move(move)
            rospy.sleep(2.)

            encoder += angular_vel

        return None

    def wander_waypoints(self, waypoints,
                         qr_detect_cb=lambda x: False,
                         qr_detect_filter_dict=None,
                         waypoint_cb=lambda: False):
        if not waypoints:
            return

        move_goal_reached = (
            lambda: self.move_client.get_goal_status_text() == "Goal reached."
        )
        for pose in waypoints:
            rospy.loginfo("Navigate to waypoint pose: {}".format(pose))
            # Navigate to waypoint goal
            self.move_to_goal(pose)

            # While the robot is navigating towards the goal (async)
            while not move_goal_reached():
                # Try to detect any QR code with the camera
                qr_msg = visp_detect_qr(filter_dict=qr_detect_filter_dict)
                if qr_msg is not None:
                    # A valid message means we detected a QR at least once
                    self.move_client.cancel_all_goals()

                    rospy.loginfo("Detected QR: msg={}".format(qr_msg))
                    stop = qr_detect_cb(qr_msg)
                    if stop:
                        return
                    # Resume last navigation goal
                    self.move_to_goal(pose)

            # We reached the waypoint
            stop = waypoint_cb()
            if stop:
                return

        raise Exception("Did not find the initial QRs while wandering!")

    def get_qr_world_pose_from_camera(self, timeout=3):
        t = rospy.Time()
        msg = rospy.wait_for_message('visp_auto_tracker/object_position', PoseStamped, timeout)
        msg.header.frame_id = self.camera_frame
        msg.header.stamp = t

        self.tl.waitForTransform(self.camera_frame, self.world_frame, t, rospy.Duration(timeout))
        wmsg = self.tl.transformPose(self.world_frame, msg)

        return wmsg.pose

    def find_hidden_frame(self):
        # TODO: Calculate the hidden frame
        pass

    def get_qr_world_pos_from_hidden(self, qr_hidden_pos):
        # TODO: Transform point from hidden to world frame
        world_pos = None
        return world_pos

    def get_waypoints_around_qr_world_pos(self, qr_world_pos, num_waypoints=5, radius=1):
        angles = np.linspace(0, 2 * math.pi, num_waypoints)

        waypoints = []
        for angle in angles:
            x = qr_world_pos.x + radius * math.cos(angle)
            y = qr_world_pos.y + radius * math.sin(angle)
            waypoint = [(x, y, 0), (0, 0, 0, 0)]
            waypoints.append(waypoint)

        return waypoints

    def find_initial_qrs(self):
        # Find the first two QRs (at least). For these QRs we need to
        # properly stop the robot to estimate the world pose from the
        # camera

        def initial_qr_detect_cb(detect_qr_msg):
            # Callback invoked when we detect a QR while wandering

            # We add the QR we detect in case it is different
            # from the one we will scan
            self.add_qr_info_all(detect_qr_msg)

            # Scan area rotating robot to find and lock QR
            scan_qr_msg = self.scan_for_qr(qr_detect_filter_dict=self.qr_info_initial)
            if scan_qr_msg:
                scan_qr_info = self.add_qr_info_all(scan_qr_msg)

                # Get QR code world pose from camera
                scan_qr_info.world_pose = self.get_qr_world_pose_from_camera()

                self.qr_info_initial[scan_qr_info.qr_id] = scan_qr_info

                # If this is the second QR we should stop wandering
                stop = (len(self.qr_info_initial) == 2)
                return stop

            return False

        # Wander around specific waypoints in the map
        self.wander_waypoints(self.init_waypoints,
                              qr_detect_cb=initial_qr_detect_cb,
                              qr_detect_filter_dict=self.qr_info_initial)

    def while_remaining_qrs_to_find(self):
        # While we have remaining QRs yield the next to find
        while len(self.qr_info_all) < NUM_QR_IDS:
            # Get remaining QR IDs
            current_qr_ids = set(self.qr_info_all.keys())
            remaining_qr_ids = list(set(range(1, NUM_QR_IDS+1))-current_qr_ids)

            rospy.loginfo("Remaining QR IDs: {}".format(remaining_qr_ids))

            # Get QR IDs that we can find
            qrs_to_find = {}
            for qr_id in remaining_qr_ids:
                prev_qr_id = (qr_id - 1 + NUM_QR_IDS) % NUM_QR_IDS
                prev_qr_info = self.qr_info_all.get(prev_qr_id)
                if prev_qr_info is not None:
                    qr_hidden_pos = prev_qr_info.next_pos
                    qrs_to_find[qr_id] = self.get_qr_world_pos_from_hidden(qr_hidden_pos)

            rospy.loginfo("Remaining QRs to find: {}".format(qrs_to_find))
            yield qrs_to_find

    def find_remaining_qrs(self):
        # Find the rest of the QRs from the QRs info after
        # solving the hidden frame from the first two QRs
        for qrs_to_find in self.while_remaining_qrs_to_find():
            for qr_id, qr_world_pos in qrs_to_find.items():
                # We might have found this QR by chance while wandering. If so, skip it
                if qr_id in self.qr_info_all:
                    continue

                # Note that these callbacks will change with every qr_id
                def remaining_qr_detect_cb(detect_qr_msg):
                    # Callback invoked when we detect a QR while wandering
                    # We add any new QR, even if it is not the one we are hunting
                    self.add_qr_info_all(detect_qr_msg)

                    if QRInfo(detect_qr_msg.data).qr_id == qr_id:
                        # This is the QR we are trying to find so stop wandering
                        return True

                    return False

                def remaining_qr_waypoint_cb():
                    # Scan area rotating robot to find specific QR
                    scan_qr_retries = NUM_QR_IDS-2  # Number of retries to find specific QR in full scan

                    for _ in range(scan_qr_retries):
                        scan_qr_msg = self.scan_for_qr(qr_detect_filter_dict=self.qr_info_all)
                        if scan_qr_msg:
                            scan_qr_info = self.add_qr_info_all(scan_qr_msg)
                            if scan_qr_info.qr_id == qr_id:
                                # This is the QR we are trying to find so stop wandering
                                return True
                        else:
                            # We did not find the specific QR after a full scan
                            break

                    return False

                # Wander around waypoints near the QR code
                waypoints = self.get_waypoints_around_qr_world_pos(qr_world_pos)
                self.wander_waypoints(waypoints,
                                      qr_detect_cb=remaining_qr_detect_cb,
                                      qr_detect_filter_dict=self.qr_info_all,
                                      waypoint_cb=remaining_qr_waypoint_cb)

    def build_secret_message(self):
        secret_msg = "".join(
            info.secret_letter for _, info in sorted(self.qr_info_all.items())
        )
        return secret_msg

    def find_secret_message(self):
        self.find_initial_qrs()
        self.find_hidden_frame()
        self.find_remaining_qrs()

        return self.build_secret_message()
