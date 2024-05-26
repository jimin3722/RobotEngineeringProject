#!/usr/bin/env python3

import os
import rospy
import numpy as np
import time
from re_project.msg import *
from inverse_kinematics import inverse_kinematics


AX_DXL_ID = [1, 2, 3, 4]
XL_DXL_ID = [5]

l1 = 11.75
l2 = 15
l3 = 15
l4 = 7

class mission_pen:

    def __init__(self):
        self.pub_angle = rospy.Publisher('set_position', SyncSetPosition, queue_size = 1)
        self.sub_Present_Pose = rospy.Subscriber('present_position', SyncSetPosition, self.PoseCallback, queue_size=1) 

        self.res_line = 50
        self.res_circle = 50

    def PoseCallback(self, msg):
        self.present_pose = msg.ax_position 

    def make_circle(self, center_point, r):

        circle = []
        
        # 원의 중심점 좌표
        x, y, z = center_point
        
        angles = np.linspace(0, 2 * np.pi, self.res_circle + 1)[:-1]
        
        # 각 부분의 좌표 계산
        for angle in angles:
            x_coord = x + r * np.cos(angle)
            y_coord = y + r * np.sin(angle)
            circle.append([x_coord, y_coord, z])
        
        return circle

    def make_line(self, first_position, second_position):
        
        step_sizes = [abs(second_position[i] - first_position[i]) / self.res_line for i in range(3)]

        positions = [first_position]
        for i in range(self.res_line):
            position = [first_position[0] + (i if first_position[0]<second_position[0] else -i) * step_sizes[0],
                        first_position[1] + (i if first_position[1]<second_position[1] else -i) * step_sizes[1],
                        first_position[2] + (i if first_position[2]<second_position[2] else -i) * step_sizes[2]]
            positions.append(position)
        
        return positions
    
    def pub_angles(self, angles):
        data = SyncSetPosition()
        data.ax_id = AX_DXL_ID
        data.ax_position = angles
        self.pub_angle.publish(data)
        
    
def main():

    first_flag = True

    mode = "line"
    mode = "circle"

    rospy.init_node('mission_pen')
    rate = rospy.Rate(5)

    inv = inverse_kinematics(l1, l2, l3, l4)    

    mp = mission_pen()

    mp.pub_angles([0,0,0,0])

    rospy.sleep(3)

    if mode == "circle":
        positions = mp.make_circle([3,4,0], 2)
    if mode == "line":
        positions = mp.make_line([1,2,0], [4,5,0])
    
    for pos in positions:
        
        angles = np.array(inv.calc_inv_to_ground(pos[0], pos[1], pos[2]))

        if first_flag:
            
            mp.pub_angles(angles)

            first_flag = False

            continue

        mp.pub_angles(angles)

        rate.sleep()
        
        
if __name__ == "__main__":
    main()