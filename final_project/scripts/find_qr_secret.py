#!/usr/bin/env python

import rospy
from final_project.qr_secret_solver import QRSecretSolver


def main():
    log_lvl=rospy.INFO
    # log_lvl=rospy.DEBUG
    rospy.init_node('find_qr_secret', log_level=log_lvl)

    init_waypoints = [
        [(-7.16, -3.47, 0.0), (0.0, 0.0, -0.6, 0.78)],
        [(-5.91, 1.3, 0.0), (0.0, 0.0, 0.69, 0.71)],
        [(5.36, 2.14, 0.0), (0.0, 0.0, -0.02, 0.99)],
        [(7, -2.88, 0.0), (0.0, 0.0, -0.66, 0.74)]
    ]
    solver = QRSecretSolver(init_waypoints)

    msg = solver.find_secret_message()
    rospy.loginfo("Secret message found: {}".format(msg))


if __name__ == '__main__':
    main()
