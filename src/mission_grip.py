#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Point
from manipulator.msg import *
from math import *
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
import inverse_kinematics


AX_DXL_ID = [1, 2, 3, 4]
XL_DXL_ID = [5]

Aa = 6.5 + 3.0
Xx = 45 * pi/180
Bb = 4.0 + 12.0
Yy = 35 * pi/180
Hh = 7.0

l1 = 11.75
l2 = 13
l3 = 13
l4 = 7

cube_size = 2.5
cube_loc = [-cube_size * np.sin(Yy), cube_size * np.cos(Yy), 0]
cube_h = [0, 0, cube_size] 

gripping_position = [99999, 99999, 99999]
ungripping_position = [-99999, -99999, -99999]
gripping_theta = 256
gripping_time = 30  # topic 30번 발행
    
class ThetaPub:
    def __init__(self):
        self.ThetaPublisher = rospy.Publisher('set_position', SyncSetPosition, queue_size=1)
        self.PresentPoseSub = rospy.Subscriber('present_position', SyncSetPosition, self.PoseCallback, queue_size=1) 
        
        self.PoseInfo = SyncSetPosition()
        self.flag = False
        
    def PoseCallback(self, msg):
        self.flag = True
        self.PoseInfo = msg
        
        
def linspace(target_position, first_position):

    step_sizes = [abs(target_position[i] - first_position[i]) / 300 for i in range(3)]

    positions = [first_position]
    for i in range(300):
        position = [first_position[0] + (i if first_position[0]<target_position[0] else -i) * step_sizes[0],
                    first_position[1] + (i if first_position[1]<target_position[1] else -i) * step_sizes[1],
                    first_position[2] + (i if first_position[2]<target_position[2] else -i) * step_sizes[2]]
        positions.append(position)

    positions.append(target_position)
    return positions


def JointSpace(theta_, traj_):
    
    Inverse = inverse_kinematics.inverse_kinematics(l1, l2, l3, l4)
    t = [0, 0, 0, 0, 0]
    grip_flag = False
    
    for i in traj_:
        if i != gripping_position and i != ungripping_position:
            t = Inverse.calc_inv_to_ground(i)
            if grip_flag:
                t.append(gripping_theta)
            else:
                t.append(0)
            theta_.append(t)
        else:
            if i == gripping_position:
                grip_flag = True
                t[4] = gripping_theta
            else:
                grip_flag = False
                t[4] = 0
                
            for j in range(gripping_time):
                theta_.append(t)
    
    return theta_
    
    
def main():
    rospy.init_node('mission_grip')
    rate = rospy.Rate(5)
    
    theta_pub = ThetaPub()
    Theta = SyncSetPosition()
    
    
    Theta.ax_id = AX_DXL_ID
    Theta.xl_id = XL_DXL_ID

    # initialpoint -> pickpoint -> grip -> initialpoint -> placepoint -> ungrip -> ...
    PickPoint = [Aa*sin(Xx), Aa*cos(Xx), Hh] # 블럭 집는 위치
    PlacePoint = [Bb*sin(Yy), Bb*cos(Yy), Hh] # 블럭 놓는 위치
    InitialPoint = (PickPoint + PlacePoint)/2 # 가운데 적당한 높이 -> 쌓여있는 블럭에 닿지 않는
    
    theta = []
    
    
    for i in range(4):
        traj = np.concatenate((linspace(InitialPoint, PickPoint), gripping_position, linspace(PickPoint, InitialPoint),
                               linspace(InitialPoint, PlacePoint), ungripping_position, linspace(PlacePoint, InitialPoint)))
        PlacePoint += cube_loc
    
    theta = JointSpace(theta, traj)      
        
    PlacePoint = [Bb*sin(Yy), Bb*cos(Yy), Hh]
    PlacePoint = PlacePoint + cube_loc / 2 + cube_h
    InitialPoint += cube_h
    
    for i in range(3):
        traj = np.concatenate((linspace(InitialPoint, PickPoint), gripping_position, linspace(PickPoint, InitialPoint),
                               linspace(InitialPoint, PlacePoint), ungripping_position, linspace(PlacePoint, InitialPoint)))
        PlacePoint += cube_loc
    
    theta = JointSpace(theta, traj)
    
    PlacePoint = [Bb*sin(Yy), Bb*cos(Yy), Hh]
    PlacePoint = PlacePoint + cube_loc + cube_h * 2
    InitialPoint += cube_h * 2   
        
    for i in range(2):
        traj = np.concatenate((linspace(InitialPoint, PickPoint), gripping_position, linspace(PickPoint, InitialPoint),
                               linspace(InitialPoint, PlacePoint), ungripping_position, linspace(PlacePoint, InitialPoint)))
        PlacePoint += cube_loc
        
    theta = JointSpace(theta, traj)
    
    PlacePoint = [Bb*sin(Yy), Bb*cos(Yy), Hh]
    PlacePoint = PlacePoint + cube_loc * 3 / 2 + cube_h * 3
    InitialPoint += cube_h * 3
    
    traj = np.concatenate((linspace(InitialPoint, PickPoint), gripping_position, linspace(PickPoint, InitialPoint),
                           linspace(InitialPoint, PlacePoint), ungripping_position, linspace(PlacePoint, InitialPoint)))
    
    theta = JointSpace(theta, traj)
    
    time = 0
    
    while not rospy.is_shutdown():
        
        Theta = SyncSetPosition()
        
        Theta.ax_id = AX_DXL_ID
        Theta.xl_id = XL_DXL_ID
        
        Theta.ax_position.append(theta[time][0])
        Theta.ax_position.append(theta[time][1])
        Theta.ax_position.append(theta[time][2])
        Theta.ax_position.append(theta[time][3])
        Theta.xl_position.append(theta[time][4])
        
        theta_pub.ThetaPublisher.publish(Theta)
        time += 1
        
        rate.sleep()
        
        
if __name__ == "__main__":
    main()