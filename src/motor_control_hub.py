#!/usr/bin/env python3
# -- coding: utf-8 --

import os, sys
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk import *
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.port_handler import PortHandler
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.packet_handler import PacketHandler
from DynamixelSDK.ros.dynamixel_sdk.src.dynamixel_sdk.robotis_def import *
from re_project.msg import *
import math
import sys, tty, termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

####mode 설정!!!####

#MODE = "grip"
MODE = "pen"

###################

#*************************AX-12A(PROTOCOL_VERSION 1.0)*****************************#

# Control table address
AX_ADDR_TORQUE_ENABLE          = 24 # 토크 활성화(1)/비활성화(0)
AX_ADDR_CW_COMPLIANCE_MARGIN   = 26 # 시계방향 : Goal Position을 도달했다고 판단하는 Margin값, 예를 들어 Goal Position이 30이고 Margin값이 2라면 28~32에 도달하면 goal position에 도달한것으로 판단함
AX_ADDR_CCW_COMPLIANCE_MARGIN  = 27 # 반시계 방향 : ```
AX_ADDR_CW_COMPLIANCE_SLOPE    = 28 # 시계방향 : 가속/김속하는 시간
AX_ADDR_CCW_COMPLIANCE_SLOPE   = 29 # 반시계방향 : ```
AX_ADDR_GOAL_POSITION          = 30 # 목표 각도
AX_ADDR_MOVING_SPEED           = 32 # 목표 속도
AX_ADDR_PRESENT_POSITION       = 36 # 현재 각도
AX_ADDR_PRESENT_SPEED          = 38 # 현재 속도
AX_ADDR_PRESENT_LOAD           = 40
AX_ADDR_MOVING                 = 46
AX_ADDR_PUNCH                  = 48 # 모터에 가하는 최소 전류 -> 다르게 생각하면 최소 속도라고 할 수 있을 듯

AX_PROTOCOL_VERSION = 1.0

AX_DXL_ID = [1, 2, 3, 4] # 모터 ID 0부터 순서대로 설정

BAUDRATE = 1000000

AX_TORQUE_ENABLE = 1
AX_TORQUE_DISABLE = 0

AX_CW_COMPLIANCE_MARGIN = 1 #실제로 설정하려는 값
AX_CCW_COMPLIANCE_MARGIN = 1
AX_CW_COMPLIANCE_SLOPE = 128
AX_CCW_COMPLIANCE_SLOPE = 128

DEVICENAME = '/dev/ttyUSB0'


port_handler = PortHandler(DEVICENAME)
ax_packet_handler = PacketHandler(AX_PROTOCOL_VERSION)

#**********************************************************************************#


#**********************XL330-M288-T(PROTOCOL_VERSION 2.0)**************************#

# Control table address

XL_ADDR_TORQUE_ENABLE           = 64
XL_ADDR_VELOCITY_I_GAIN         = 76
XL_ADDR_VELOCITY_P_GAIN         = 78
XL_ADDR_POTISION_D_GAIN         = 80
XL_ADDR_POSITION_I_GAIN         = 82
XL_ADDR_POSITION_P_GAIN         = 84
XL_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XL_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XL_ADDR_PROFILE_ACCELERATION    = 108
XL_ADDR_PROFILE_VELOCITY        = 112
XL_ADDR_GOAL_POSITION           = 116
XL_ADDR_MOVING                  = 122
XL_ADDR_MOVING_STATUS           = 123
XL_ADDR_PRESENT_POSITION        = 132

XL_PROTOCOL_VERSION = 2.0

XL_DXL_ID = [5] # 그리퍼 모터 ID 5로 고정


XL_TORQUE_ENABLE = 1
XL_TORQUE_DISABLE = 0

xl_packet_handler = PacketHandler(XL_PROTOCOL_VERSION)


MOTOR_VELOCITY = [100, 100, 100, 100, 100] # 모터 ID 별 속도


#**********************************************************************************#


def ax_rad_to_position(rad):
    y_axis_position = 511
    add_position = round(rad*(180/math.pi)/(300/1024))
    position = y_axis_position+add_position
    if(position > 1023):
        print("overposition: ",position)
        position = 1023
    elif(position < 0) :
        position = 0
    return position


def xl_rad_to_position(rad):
    y_axis_position = 2047
    add_position = round(rad*(180/math.pi)/(360/4096))
    position = y_axis_position+add_position
    if(position > 4095):
        position = 4095
    elif(position < 0) :
        position = 0
    return position
    

class MotorControlHub:

    def __init__(self):

        self.set_pos = SyncSetPosition()
        self.set_ax_speed = AXSyncSetMovingSpeed()
        
        self.target_position_flag = False
        
        self.target_first_link_flag = False
        
        self.set_pos.ax_id = AX_DXL_ID
        self.set_pos.xl_id = XL_DXL_ID

        self.set_pos.ax_position = [0, 0, 0, 0]
        self.set_pos.xl_position = [2048]

        self.set_ax_speed.id = AX_DXL_ID
        self.set_ax_speed.speed = [40]
        
        rospy.Subscriber('set_position', SyncSetPosition, self.set_goal_pos_callback, queue_size=1)
        
        self.pos_pub = rospy.Publisher('present_position', SyncSetPosition, queue_size=1)
        self.ax_speed_pub = rospy.Publisher('present_ax_speed', AXSyncSetMovingSpeed, queue_size=1)


    def set_goal_pos_callback(self,data):
        self.set_pos = data

    def set_goal_pos(self,data:SyncSetPosition):
        for idx in range(len(data.ax_id)): # id radian position
            ax_pos = ax_rad_to_position(data.ax_position[idx])
            print("Set Goal AX_Position of ID %s = %s, %s" % (data.ax_id[idx], data.ax_position[idx], ax_pos))
            ax_packet_handler.write2ByteTxRx(port_handler,data.ax_id[idx], AX_ADDR_GOAL_POSITION, ax_pos)

        if MODE == "grip":
            
            for idx in range(len(data.xl_id)): # id radian position
                xl_pos = xl_rad_to_position(data.xl_position[idx])
                print("Set Goal XL_Position of ID %s = %s, %s" % (data.xl_id[idx], data.xl_position[idx], xl_pos))
                xl_packet_handler.write4ByteTxRx(port_handler,data.xl_id[idx], XL_ADDR_GOAL_POSITION, xl_pos)

    def present_position_callback(self):
        present_position = SyncSetPosition()
        present_position.ax_id = AX_DXL_ID
        present_position.xl_id = XL_DXL_ID
        present_position.ax_position = []
        present_position.xl_position = []

        for id in AX_DXL_ID:
            dxl_present_position, dxl_comm_result, dxl_error = ax_packet_handler.read2ByteTxRx(port_handler, id, AX_ADDR_PRESENT_POSITION)
            present_position.ax_position.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return
            
        for id in XL_DXL_ID:
            dxl_present_position, dxl_comm_result, dxl_error = xl_packet_handler.read4ByteTxRx(port_handler, id, XL_ADDR_PRESENT_POSITION)
            present_position.xl_position.append(dxl_present_position)
            if(dxl_comm_result != COMM_SUCCESS) :
                return
            if(dxl_error != 0) :
                return
            
        self.pos_pub.publish(present_position)


def main():


    rospy.init_node('motor_control_hub')
 

    try:
        port_handler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    try:
        port_handler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    for id in AX_DXL_ID :
        dxl_comm_result, dxl_error = ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % ax_packet_handler.getTxRxResult(dxl_comm_result))
            print("Press any key to terminate...")
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % ax_packet_handler.getRxPacketError(dxl_error))
            print("Press any key to terminate...")
            getch()
            quit()
        else:
            print(f"DYNAMIXEL(ID : {id}) has been successfully connected")
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_MARGIN, AX_CW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_MARGIN, AX_CCW_COMPLIANCE_MARGIN) #초기 margin 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CW_COMPLIANCE_SLOPE, AX_CW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write1ByteTxRx(port_handler, id, AX_ADDR_CCW_COMPLIANCE_SLOPE, AX_CCW_COMPLIANCE_SLOPE) #초기 slope 설정
            ax_packet_handler.write2ByteTxRx(port_handler, id, AX_ADDR_MOVING_SPEED, MOTOR_VELOCITY[id]) #초기 속도 설정

    if MODE == "grip":

        for id in XL_DXL_ID:
            dxl_comm_result, dxl_error = xl_packet_handler.write1ByteTxRx(port_handler, id, XL_ADDR_TORQUE_ENABLE, XL_TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % xl_packet_handler.getTxRxResult(dxl_comm_result))
                print("Press any key to terminate...")
                getch()
                quit()
            elif dxl_error != 0:
                print("%s" % xl_packet_handler.getRxPacketError(dxl_error))
                print("Press any key to terminate...")
                getch()
                quit()
            else:
                xl_packet_handler.write4ByteTxRx(port_handler, id, XL_ADDR_PROFILE_VELOCITY, MOTOR_VELOCITY[id])
            print(f"DYNAMIXEL(ID : {id}) has been successfully connected")

    
    print("Ready to get & set Position.")

    ############################################################################################################
    #  여기까지는 dynamixel 기본 설정
    ############################################################################################################

    data_hub = MotorControlHub()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        data_hub.set_goal_pos(data_hub.set_pos)

        data_hub.present_position_callback()

        rate.sleep()


if __name__ == '__main__':
    main()