#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import Point
from manipulator.msg import *
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
import inverse_kinematics


AX_DXL_ID = [1, 2, 3, 4]
XL_DXL_ID = [5]
L1 = 0
THETA1 = 0
L2 = 0
THETA2 = 0

l1 = 11.75
l2 = 15
l3 = 15
l4 = 7
    
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
    
    
def main():
    rospy.init_node('mission_grip')
    rate = rospy.Rate(5)
    
    theta_pub = ThetaPub()
    Theta = SyncSetPosition()
    Inverse = inverse_kinematics.inverse_kinematics(l1, l2, l3, l4)
    
    Theta.ax_id = AX_DXL_ID
    Theta.xl_id = XL_DXL_ID
    
    # initialpoint -> pickpoint -> grip -> initialpoint -> placepoint -> ungrip -> ...
    InitialPoint = [] # 가운데 적당한 높이 -> 쌓여있는 블럭에 닿지 않는
    PickPoint = [] # 블럭 집는 위치
    PlacePoint = [] # 블럭 놓는 위치
    
    GripFlag = False
    
    while not rospy.is_shutdown():
        
        Theta = SyncSetPosition()
        
        Theta.ax_id = AX_DXL_ID
        Theta.xl_id = XL_DXL_ID
        
        Theta.ax_position.append()
        Theta.ax_position.append()
        Theta.ax_position.append()
        Theta.ax_position.append()
        Theta.xl_position.append()
        
        theta_pub.ThetaPublisher.publish(Theta)
        
        rate.sleep()
        
        
if __name__ == "__main__":
    main()