#!/usr/bin/env python

import rospy, rospkg, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from argparse import ArgumentParser
import math


def parse_args(argv=None):
    parser = ArgumentParser()

    parser.add_argument(
        "-d", "--delete", dest="delete", action='store_true',
        help='Delete the models instead of spawning them'
    )

    args = parser.parse_args(argv)
    return args


def main():
    args = parse_args(rospy.myargv()[1:])
    spawn_not_delete = not args.delete

    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))

    def get_model_xml(model_pkg_path, pkg_name="mini_project1"):
        pkg_path = rospkg.RosPack().get_path(pkg_name)
        model_abs_path = "{}/{}".format(pkg_path, model_pkg_path)
        with open(model_abs_path, "r") as f:
            return f.read()

    def spawn_item(item_name, item_pose, model_pkg_path):
        print("Spawning model: ", item_name)
        model_xml = get_model_xml(model_pkg_path)
        spawn_model(item_name, model_xml, "", item_pose, "world")

    # Manipulate cubes
    num_of_cubes = random.randint(2,6) if spawn_not_delete else 6
    for num in range(num_of_cubes):
        item_name = "cube{}".format(num)
        if spawn_not_delete:
            x0 = 0.35
            y0 = 0.2
            circle_r = 0.20
            alpha_x = 2 * math.pi * random.random()
            alpha_y = 2 * math.pi * random.random()
            r = circle_r * math.sqrt(random.random())
            pose_x = r * math.cos(alpha_x) + x0
            pose_y = r * math.cos(alpha_y) + y0
            pose_z = 0.9
            item_pose = Pose(Point(x=pose_x, y=pose_y, z=pose_z), orient)
            spawn_item(item_name, item_pose, "urdf/cube.urdf")
        else:
            delete_model(item_name)

    # Manipulate bucket
    item_name = "bucket"
    if spawn_not_delete:
        item_pose = Pose(Point(x=0.53, y=-0.23,  z=0.78), orient)
        spawn_item(item_name, item_pose, "urdf/bucket.urdf")
    else:
        delete_model(item_name)

if __name__ == '__main__':
    main()
