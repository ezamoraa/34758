#!/usr/bin/env python
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
 
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def move_gripper(action, wait_after_s=3):

  assert action in ["open", "close"], "Invalid gripper action"
 
  joint_pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  currentJointState.header.stamp = rospy.get_rostime()

  action_pos = {
    "open": 0.005,
    "close": 0.7,
  }
  currentJointState.position = tuple(
    list(currentJointState.position[:6]) + 3 * [action_pos[action]]
  )

  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    joint_pub.publish(currentJointState)
    rate.sleep()

  rospy.sleep(wait_after_s)


def move_group_cartesian_path(robot, group, poses):

  waypoints = copy.deepcopy(poses)
  #createcartesian  plan
  (plan, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)

  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(2.)

  ## Moving to a pose goal
  group.execute(plan,wait=True)

  #rospy.sleep(4.)


def move_pickup_drop_cube(robot, group, pose_cube, pose_bucket, h_cube=0.1, h_bucket=0.05):
  
  h_limb = 0.05   #Adding extra height due to the limb height (fingers)

  rospy.logdebug("initial pos")

  # Move robot above the cube and opens the gripper
  pose_above_cube = copy.deepcopy(pose_cube)
  pose_above_cube.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_above_cube.position.z += (h_cube + h_limb)
  
  rospy.logdebug("move above cube")
  move_group_cartesian_path(robot, group, [pose_above_cube])

  rospy.logdebug("open gripper")
  move_gripper("open")

  # Move robot to cube and graps it
  pose_at_cube = copy.deepcopy(pose_above_cube)
  pose_at_cube.position.z -= (h_cube + h_limb)

  rospy.logdebug("move to cube")
  move_group_cartesian_path(robot, group, [pose_at_cube])

  rospy.logdebug("Close gripper")
  move_gripper("close")

  # Move robot to the bucket 
  pose_above_bucket = copy.deepcopy(pose_bucket)
  pose_above_bucket.orientation = pose_above_cube.orientation
  pose_above_bucket.position.z += (h_bucket + h_limb)

  rospy.logdebug("move to bucket")
  move_group_cartesian_path(robot, group, [pose_above_cube, pose_above_bucket])

  rospy.logdebug("open gripper")
  move_gripper("open")


def main():
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")

  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print("============ Printing robot state")
  print(robot.get_current_state())
  print("============")

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

  pose_goal2 = group.get_current_pose().pose
  pose_goal2.position.x = 0.3
  pose_goal2.position.y = -0.2
  pose_goal2.position.z = 0.95
  print(pose_goal2)

  move_pickup_drop_cube(robot, group, pose_goal, pose_goal2)

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
