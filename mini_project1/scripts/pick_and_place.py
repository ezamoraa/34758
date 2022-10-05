#!/usr/bin/env python

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

from gazebo_msgs.msg import ModelStates
from mini_project1 import move


def get_cubes_pose(model_states, cube_name_prefix="cube"):
  # Get model_states indexes for all the cubes
  cube_i = [
      i for i, name in enumerate(model_states.name)
      if cube_name_prefix in name
  ]
  assert len(cube_i) > 0, "Failed to get cubes pose from Gazebo"

  # Generate a dictionary with the cube poses (key=name, value=pose)
  cubes_pose = {
    model_states.name[i] : model_states.pose[i]
    for i in cube_i
  }
  return cubes_pose

def get_bucket_pose(model_states, bucket_name="bucket"):
  for name, pose in zip(model_states.name, model_states.pose):
    if name == bucket_name:
      return pose
  else:
    raise AssertionError("Failed to get bucket pose from Gazebo")


def main():
  # log_lvl=rospy.INFO
  log_lvl=rospy.DEBUG

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_and_place', anonymous=True, log_level=log_lvl)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")

  rospy.logdebug(robot.get_current_state())

  move.setup_group(group)

  model_states = rospy.wait_for_message("gazebo/model_states", ModelStates, 10)
  cubes_pose = get_cubes_pose(model_states)
  bucket_pose = get_bucket_pose(model_states)

  cube_names = list(cubes_pose.keys())

  # Pick and place all cubes found
  for cube_name in cube_names:
    rospy.logdebug("PICK AND PLACE CUBE [{}]".format(cube_name))
    move.move_pick_and_place_cube(robot, group, cubes_pose[cube_name], bucket_pose)

    # Update poses just in case something changed
    model_states = rospy.wait_for_message("gazebo/model_states", ModelStates, 10)
    cubes_pose = get_cubes_pose(model_states)
    bucket_pose = get_bucket_pose(model_states)

  moveit_commander.roscpp_shutdown()

  # print("============ STOPPING")
  # R = rospy.Rate(10)
  # while not rospy.is_shutdown():
  #   R.sleep()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
