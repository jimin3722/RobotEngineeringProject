#!/usr/bin/env python3

import os
import rospy
import numpy as np
import time
from re_project.msg import *
from inverse_kinematics import inverse_kinematics



def main():
    
    rospy.init_node('angle_pub_node')
    pub_angle = rospy.Publisher('set_position', SyncSetPosition, queue_size = 1)

    while(True):

        inv = inverse_kinematics(l1 = 11.75, l2 = 15, l3 = 15, l4 = 7)
        
        data = SyncSetPosition()

        data.ax_id = [1,2,3,4]

        position = list(map(int, input("숫자들을 입력하세요: ").split()))

        ax_position = np.array(inv.calc_inv_to_ground(position[0], position[1], position[2]))

        data.ax_position = ax_position

        pub_angle.publish(data)

        a = input("u wanna terminate [y/n] : ")

        if a == "y":
            break


if __name__ == '__main__':
    main()