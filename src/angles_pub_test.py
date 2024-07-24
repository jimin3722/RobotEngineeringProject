#!/usr/bin/env python3

import os
import rospy
import numpy as np
import time
from re_project.msg import *
from inverse_kinematics import inverse_kinematics

l1 = 8.8 #11.65
l2 = 13
l3 = 13
l4 = 9.7 #5.5#6.4+0.5#11.1 #12.3
l4 = 5

def main():
    
    rospy.init_node('angle_pub_node')
    pub_angle = rospy.Publisher('set_position', SyncSetPosition, queue_size = 1)

    while(True):

        inv = inverse_kinematics(l1, l2, l3, l4, a = 3.3)

        # offset = [-0.35 ,-0.6, -0.4]
        #position  = [10, -6, 0.0]
        position  = [4, 4.5, 0.0]
        #position  = [8.0, -1.0, 0.0]
        
        ik = inv.calc_inv_to_ground
        ik = inv.calc_inv_to_ground_ver2
        
        data = SyncSetPosition()

        data.ax_id = [1,2,3,4]

        offset = list(map(float, input("offset을 입력하세요: ").split()))

        ax_position = np.array(ik(position[0]+offset[0], position[1]+offset[1], position[2]+offset[2]))

        data.ax_position = ax_position

        pub_angle.publish(data)

        a = input("u wanna terminate [y/n] : ")

        if a == "n":
            
            data = SyncSetPosition()
            data.ax_id = [1,2,3,4]
            data.ax_position = [0,0,0,0]

            pub_angle.publish(data)

        if a == "y":
            break


if __name__ == '__main__':
    main()