#!/usr/bin/env python

import rospy

import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

TRACK_MODEL = 3

def handle_visp_pose(msg):
    status = rospy.wait_for_message("/visp_auto_tracker/status", Int8, 10)
    if status.data is not TRACK_MODEL:
        return

    p = msg.pose
    br = tf.TransformBroadcaster()
    position = tuple([getattr(p.position, i) for i in ["x", "y", "z"]])
    orientation = tuple([getattr(p.orientation, i) for i in ["x", "y", "z", "w"]])

    br.sendTransform(position,
                     orientation,
                     rospy.Time.now(),
                     "qr_code",
                     "camera_optical_link")

def main():
    rospy.init_node('qr_tf_broadcaster')

    rospy.Subscriber('/visp_auto_tracker/object_position',
                     PoseStamped,
                     handle_visp_pose)
    rospy.spin()


if __name__ == '__main__':
    main()
