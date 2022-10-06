#!/usr/bin/env python
import os
from symbol import argument
import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
# from gazebo_msgs import ModelState
from geometry_msgs.msg import *

def spawn_cube_bucket():

    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/%s/catkin_ws/src/hello_ros/urdf/cube.urdf" % os.getlogin(), "r") as f:
        product_xml = f.read()

    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))

    num_of_cubes = random.randint(2,6)

    for num in xrange(0,num_of_cubes):
        bin_y   =   random.uniform(0,0.5)
        bin_x   =   random.uniform(0,0.5)
        item_name   =   "cube{}".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")

    with open("/home/%s/catkin_ws/src/hello_ros/urdf/bucket.urdf" % os.getlogin(), "r") as f:
        product_xml = f.read()

    item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=0.78),   orient)
    print("Spawning model:%s", "bucket")
    spawn_model("bucket", product_xml, "", item_pose, "world")




def delete_cube():
    print("Waiting for gazebo services...")
    rospy.init_node("delete_products")
    rospy.wait_for_service("gazebo/delete_model")
    print("Got it.")

    cubes = ["cube0", "cube2", "cube3"]
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    for cube in cubes:
        delete_model(cube)


if __name__=='__main__':
  try:
    # spawn_cube_bucket() 
    spawn_cube_bucket()
  except rospy.ROSInterruptException:
    pass