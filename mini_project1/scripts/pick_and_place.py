#!/usr/bin/env python

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

from mini_project1 import move

def main():
  log_lvl=rospy.INFO
  # log_lvl=rospy.DEBUG

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('pick_and_place', anonymous=True, log_level=log_lvl)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")

  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  rospy.logdebug(robot.get_current_state())

  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)


  pose_goal = group.get_current_pose().pose
  pose_goal.position.x = 0.4
  pose_goal.position.y = 0
  pose_goal.position.z = 0.95
  print(pose_goal)

  ## second movement
  pose_goal2 = group.get_current_pose().pose
  pose_goal2.position.x = 0.3
  pose_goal2.position.y = -0.2
  pose_goal2.position.z = 0.95
  print(pose_goal2)

  move.move_pickup_drop_cube(robot, group, pose_goal, pose_goal2)

  moveit_commander.roscpp_shutdown()

  print("============ STOPPING")
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()


if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
