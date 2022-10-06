#!/usr/bin/env python 
import rospy
from gazebo_msgs.msg import ModelStates

def get_poses():
    rospy.init_node('gaz_pose_listener')

    model_states = rospy.wait_for_message("gazebo/model_states", ModelStates, 10)

    index_names = [
        (i, name) for i, name in enumerate(model_states.name)
        if "cube" in name
        ]

    cube_poses = [model_states.pose[i] for i,_ in index_names]
    cube_names = [model_states.name[i] for i,_ in index_names]
    return cube_poses, cube_names


poses, names = get_poses()
rospy.loginfo(names)
