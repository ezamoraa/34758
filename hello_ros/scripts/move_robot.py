#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
 
from std_msgs.msg import String
from sensor_msgs.msg import JointState



def set_gripper_state(close_not_open):
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  currentJointState.header.stamp = rospy.get_rostime()
  pos = 0.7 if close_not_open else 0.005
  currentJointState.position = tuple(list(currentJointState.position[:6]) + 3 * [pos])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    rate.sleep()
  rospy.sleep(3.)


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


def pickup_drop_cube(robot, group, pose_cube, pose_bucket, h_cube=0.1, h_bucket=0.05):
  
  h_limb = 0.05   #Adding extra height due to the limb height (fingers)

  print("initial pos")

  # Move robot above the cube and opens the gripper
  pose_above_cube = copy.deepcopy(pose_cube)
  pose_above_cube.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
  pose_above_cube.position.z += (h_cube + h_limb)
  
  print("move above cube")
  move_group_cartesian_path(robot, group, [pose_above_cube])
  print("open gripper")
  set_gripper_state(False)

  # Move robot to cube and graps it
  pose_at_cube = copy.deepcopy(pose_above_cube)
  pose_at_cube.position.z -= (h_cube + h_limb)

  print("move to cube")
  move_group_cartesian_path(robot, group, [pose_at_cube])
  print("Close gripper")
  set_gripper_state(True)

  # Move robot to the bucket 
  pose_above_bucket = copy.deepcopy(pose_bucket)
  pose_above_bucket.orientation = pose_above_cube.orientation
  pose_above_bucket.position.z += (h_bucket + h_limb)

  print("move to bucket")
  move_group_cartesian_path(robot, group, [pose_above_cube, pose_above_bucket])
  print("open gripper")
  set_gripper_state(False)







 
def main():
  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
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
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
 
  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)

  
  
  
  
  pose_goal = group.get_current_pose().pose
  #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  0.  , 0.))
  pose_goal.position.x = 0.4
  pose_goal.position.y = 0
  pose_goal.position.z = 0.95
  print pose_goal

  ## second movement
  pose_goal2 = group.get_current_pose().pose
  #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  0.  , 0.))
  pose_goal2.position.x = 0.3
  pose_goal2.position.y = -0.2
  pose_goal2.position.z = 0.95
  print pose_goal2


  pickup_drop_cube(robot, group, pose_goal, pose_goal2)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()



if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass