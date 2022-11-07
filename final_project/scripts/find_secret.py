#!/usr/bin/env python

import rospy
from final_project import QRSecretSolver


def main():
    # log_lvl=rospy.INFO
    log_lvl=rospy.DEBUG

    rospy.init_node('find_qr_secret', log_level=log_lvl)

    solver = QRSecretSolver()
    msg = solver.find_secret_message()
    print("Secret message found!: {}".format(msg))


if __name__ == '__main__':
    main()
