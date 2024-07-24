#!/usr/bin/env python3

import os
import rospy
import numpy as np
import time
from re_project.msg import *
from inverse_kinematics import inverse_kinematics
import matplotlib.pyplot as plt

AX_DXL_ID = [1, 2, 3, 4]
XL_DXL_ID = [5]

l1 = 8.8 #11.65
l2 = 13
l3 = 13
l4 = 5 #9.7 #5.5#6.4+0.5#11.1 #12.3


def plot(positions):
    # x, y 데이터 분리
    x_data = [point[0] for point in positions]
    y_data = [point[1] for point in positions]

    # Plot 생성
    plt.plot(x_data, y_data, 'o-') # 'o-'는 데이터 포인트를 원으로 표시하고 선으로 연결한다는 의미입니다.

    # 제목 및 축 라벨 설정
    plt.title('Plot Example')
    plt.xlabel('X axis')
    plt.ylabel('Y axis')

    # Plot 표시
    plt.show()


class mission_pen:

    def __init__(self):
        self.pub_angle = rospy.Publisher('set_position', SyncSetPosition, queue_size = 1)
        self.sub_Present_Pose = rospy.Subscriber('present_position', SyncSetPosition, self.PoseCallback, queue_size=1) 

        self.res_line = 300
        self.res_circle = 500


    def PoseCallback(self, msg):
        self.present_pose = msg.ax_position 


    def make_circle(self, center_point, r):

        circle = []
        
        # 원의 중심점 좌표
        x, y, z = center_point
        
        angles = np.linspace(0.5 * np.pi, 2.5 * np.pi, self.res_circle + 1)[:-1]
        
        # 각 부분의 좌표 계산
        for angle in angles:
            x_coord = x + r * np.cos(angle)
            y_coord = y + r * np.sin(angle)
            circle.append([x_coord, y_coord, z])
        
        return circle
    

    def make_ellips(self, center_point, r):
        
        # 원의 중심점 좌표
        x, y, z = center_point

        ellips = []

        # 타원의 중심
        center_x = x
        center_y = y

        # 장축과 단축의 길이
        a = r+0.1  # 장축 반경
        b = r-0.1  # 단축 반경

        # 타원의 각도 (중점에서의 벡터와 평행)
        theta = np.arctan2(center_y + 0, center_x + 6)  # 벡터 (1, 0)과 평행함

        # 타원의 점들 생성
        ts = np.linspace(-0.0 * np.pi, 2 * np.pi, self.res_circle)
        ts = np.linspace(2.0 * np.pi, -0.1 * np.pi, self.res_circle)


        # 각 부분의 좌표 계산
        for t in ts:
            x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
            y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
            ellips.append([x, y, z])
        
        return ellips

   
    def make_ellips2(self, center_point, r):
        
        # 원의 중심점 좌표
        x, y, z = center_point

        ellips1 = []
        ellips2 = []

        # 타원의 중심
        center_x = x
        center_y = y

        # 장축과 단축의 길이
        a = r+0.2  # 장축 반경
        b = r+0.4  # 단축 반경

        # 타원의 각도 (중점에서의 벡터와 평행)
        theta = np.arctan2(center_y + 0, center_x + 6)  # 벡터 (1, 0)과 평행함

        # 타원의 점들 생성
        ts = np.linspace(0.0 * np.pi, 1.05 * np.pi, self.res_circle)

        # 각 부분의 좌표 계산
        for t in ts:
            x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
            y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
            ellips1.append([x, y, z])

        # 타원의 점들 생성
        ts = np.linspace(2.05 * np.pi, 1.0 * np.pi, self.res_circle)

        # 각 부분의 좌표 계산
        for t in ts:
            x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
            y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
            ellips2.append([x, y, z])
        
        return ellips1, ellips2


    def make_line(self, first_position, second_position):
        
        step_sizes = [abs(second_position[i] - first_position[i]) / self.res_line for i in range(3)]

        positions = []

        residual_term = 0#round(self.res_circle/500)

        for i in range(residual_term):
            position = [first_position[0] - (i if first_position[0]<second_position[0] else -i) * step_sizes[0],
                        first_position[1] - (i if first_position[1]<second_position[1] else -i) * step_sizes[1],
                        first_position[2] - (i if first_position[2]<second_position[2] else -i) * step_sizes[2]]
            positions.append(position)
        
        positions.reverse()

        for i in range(4,self.res_line+residual_term):
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

    rospy.init_node('mission_pen')

    ###### ik 설정 #######

    inv = inverse_kinematics(l1, l2, l3, l4, a = 3.3)    
    
    #ik = inv.calc_inv_to_ground
    ik = inv.calc_inv_to_ground_ver2

    mp = mission_pen()


    ###### 미션 설정 #######

    line = True
    #line = False
    
    circle_1 = True
    #circle_1 = False
    
    circle_2 = True
    circle_2 = False


    ###### circle 미션 인자 설정 #######

    circle_rate = 25

    r = 3.95

    circle_offset = [-0.45, -0.5, -0.65]
    circle_pose = [8.0, -1.0, 0.0]

    # ver1
    # circle_offset = [-0.0, -0.15, -0.1]
    # circle_pose = [8.65, -2.2, 0]


    ###### line 미션 인자 설정 #######

    line_rate = 30#50
    
    line_dot1_offset = [-0.3, -0.5, -0.7] #[-0.2 ,0, -0.7] #[-0.9, -0.5, -0.5]
    line_dot1_pose = [10, -6, 0.0]

    line_dot2_offset = [-0.55, -0.4, -0.5] #[-0.2 ,0, -0.7] #[-1, -0.05, -0.5]
    line_dot2_pose = [4, 4.5, 0.0]


    ########## 미션 시작!!!!!! ##########

    ellips_positions1, ellips_positions2 = mp.make_ellips2([circle_pose[0] + circle_offset[0], circle_pose[1] + circle_offset[1], circle_pose[2] + circle_offset[2]], r)

    ellips_positions = mp.make_ellips([circle_pose[0] + circle_offset[0], circle_pose[1] + circle_offset[1], circle_pose[2] + circle_offset[2]], r)

    line_positions = mp.make_line([line_dot1_pose[0] + line_dot1_offset[0], line_dot1_pose[1] + line_dot1_offset[1], line_dot1_pose[2] + line_dot1_offset[2]], \
                                [line_dot2_pose[0] + line_dot2_offset[0], line_dot2_pose[1] + line_dot2_offset[1], line_dot2_pose[2] + line_dot2_offset[2]])

    
    ## initiatl pose_1 : 일단 기지개 ##
    
    t = time.time()

    while(time.time()-t < 2):
    
        mp.pub_angles([0,0,0,0])

    ##-----------------------------##

    if circle_2:

        first_flag1 = True
        first_flag2 = True

        rate = rospy.Rate(circle_rate)

        for pos in ellips_positions1:
            
            angles = np.array(ik(pos[0], pos[1], pos[2]))

            if first_flag1:

                ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
                t = time.time()
                
                while(time.time()-t < 4):

                    angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

                    mp.pub_angles(angles)
                ##----------------------------------------------##
                
                ## initiatl pose_3 : 첫 지점에서 2초간 대기
                t = time.time()

                while(time.time()-t < 1):

                    angles = np.array(ik(pos[0], pos[1], pos[2]))

                    mp.pub_angles(angles)
                ##-----------------------------------##

                first_flag1 = False

                continue

            mp.pub_angles(angles)

            rate.sleep()

        ## 마지막 포지션 위로 ##

        t = time.time()

        while(time.time()-t < 2):

            # angles = np.array(ik(pos[0], pos[1], pos[2]+3))
            angles = np.array(ik(circle_pose[0],circle_pose[1],circle_pose[2]+4))

            mp.pub_angles(angles)        
        
        ##-----------------##

        for pos in ellips_positions2:
            
            angles = np.array(ik(pos[0], pos[1], pos[2]))

            if first_flag2:

                ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
                t = time.time()
                
                while(time.time()-t < 2):

                    angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

                    mp.pub_angles(angles)
                ##----------------------------------------------##
                
                ## initiatl pose_3 : 첫 지점에서 1초간 대기
                t = time.time()

                while(time.time()-t < 1):

                    angles = np.array(ik(pos[0], pos[1], pos[2]))

                    mp.pub_angles(angles)
                ##-----------------------------------##

                first_flag2 = False

                continue

            mp.pub_angles(angles)

            rate.sleep()


    if circle_1:

        first_flag = True

        rate = rospy.Rate(circle_rate)

        for pos in ellips_positions:
            
            angles = np.array(ik(pos[0], pos[1], pos[2]))

            if first_flag:
                
                ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
                t = time.time()
                
                while(time.time()-t < 8):

                    angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

                    mp.pub_angles(angles)
                ##----------------------------------------------##
                
                ## initiatl pose_3 : 첫 지점에서 2초간 대기
                t = time.time()

                while(time.time()-t < 1):

                    angles = np.array(ik(pos[0], pos[1], pos[2]))

                    mp.pub_angles(angles)
                ##-----------------------------------##

                first_flag = False

                continue

            mp.pub_angles(angles)

            rate.sleep()


    if line:

        first_flag = True

        rate = rospy.Rate(line_rate)

        for pos in line_positions:
            
            angles = np.array(ik(pos[0], pos[1], pos[2]))

            if first_flag:
                
                ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
                t = time.time()
                
                while(time.time()-t < 4):

                    angles = np.array(ik(pos[0], pos[1], pos[2]+0.6))

                    mp.pub_angles(angles)
                ##---------------------------------------------##
                
                ## initiatl pose_3 : 첫 지점에서 2초간 대기
                t = time.time()

                while(time.time()-t < 1):

                    angles = np.array(ik(pos[0], pos[1], pos[2]))

                    mp.pub_angles(angles)
                ##---------------------------------------------##

                first_flag = False

                continue

            mp.pub_angles(angles)

            rate.sleep()

    ## 마지막 포지션 위로 ##

    t = time.time()

    while(time.time()-t < 3):

        # angles = np.array(ik(pos[0], pos[1], pos[2]+3))
        angles = np.array(ik(pos[0]+3,pos[1],pos[2]+4))

        mp.pub_angles(angles)        
    
    ##-----------------##

    ## final pose : 다시 기지개 ##
    
    t = time.time()

    while(time.time()-t < 2):
    
        mp.pub_angles([0,0,0,0])
    
    ##------------------------##


    ########## 미션 끝 ##########


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3

# import os
# import rospy
# import numpy as np
# import time
# from re_project.msg import *
# from inverse_kinematics import inverse_kinematics
# import matplotlib.pyplot as plt

# AX_DXL_ID = [1, 2, 3, 4]
# XL_DXL_ID = [5]

# l1 = 8.8 #11.65
# l2 = 13
# l3 = 13
# l4 = 9.7 #5 #6.4+0.5#11.1 #12.3


# def plot(positions):
#     # x, y 데이터 분리
#     x_data = [point[0] for point in positions]
#     y_data = [point[1] for point in positions]

#     # Plot 생성
#     plt.plot(x_data, y_data, 'o-') # 'o-'는 데이터 포인트를 원으로 표시하고 선으로 연결한다는 의미입니다.

#     # 제목 및 축 라벨 설정
#     plt.title('Plot Example')
#     plt.xlabel('X axis')
#     plt.ylabel('Y axis')

#     # Plot 표시
#     plt.show()


# class mission_pen:

#     def __init__(self):
#         self.pub_angle = rospy.Publisher('set_position', SyncSetPosition, queue_size = 1)
#         self.sub_Present_Pose = rospy.Subscriber('present_position', SyncSetPosition, self.PoseCallback, queue_size=1) 

#         self.res_line = 300
#         self.res_circle = 500


#     def PoseCallback(self, msg):
#         self.present_pose = msg.ax_position 


#     def make_circle(self, center_point, r):

#         circle = []
        
#         # 원의 중심점 좌표
#         x, y, z = center_point
        
#         angles = np.linspace(0.5 * np.pi, 2.5 * np.pi, self.res_circle + 1)[:-1]
        
#         # 각 부분의 좌표 계산
#         for angle in angles:
#             x_coord = x + r * np.cos(angle)
#             y_coord = y + r * np.sin(angle)
#             circle.append([x_coord, y_coord, z])
        
#         return circle
    

#     def make_ellips(self, center_point, r):
        
#         # 원의 중심점 좌표
#         x, y, z = center_point

#         ellips = []

#         # 타원의 중심
#         center_x = x
#         center_y = y

#         # 장축과 단축의 길이
#         a = r+0.15  # 장축 반경
#         b = r-0.25  # 단축 반경

#         # 타원의 각도 (중점에서의 벡터와 평행)
#         theta = np.arctan2(center_y + 0, center_x + 6)  # 벡터 (1, 0)과 평행함

#         # 타원의 점들 생성
#         ts = np.linspace(-0.0 * np.pi, 2 * np.pi, self.res_circle)
#         ts = np.linspace(2.0 * np.pi, 0 * np.pi, self.res_circle)


#         # 각 부분의 좌표 계산
#         for t in ts:
#             x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
#             y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
#             ellips.append([x, y, z])
        
#         return ellips

   
#     def make_ellips2(self, center_point, r):
        
#         # 원의 중심점 좌표
#         x, y, z = center_point

#         ellips1 = []
#         ellips2 = []

#         # 타원의 중심
#         center_x = x
#         center_y = y

#         # 장축과 단축의 길이
#         a = r+0.1  # 장축 반경
#         b = r+0.1  # 단축 반경

#         # 타원의 각도 (중점에서의 벡터와 평행)
#         theta = np.arctan2(center_y + 0, center_x + 6)  # 벡터 (1, 0)과 평행함

#         # 타원의 점들 생성
#         ts = np.linspace(0.0 * np.pi, 1.05 * np.pi, self.res_circle)

#         # 각 부분의 좌표 계산
#         for t in ts:
#             x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
#             y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
#             ellips1.append([x, y, z])

#         # 타원의 점들 생성
#         ts = np.linspace(2.05 * np.pi, 1.0 * np.pi, self.res_circle)

#         # 각 부분의 좌표 계산
#         for t in ts:
#             x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
#             y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
#             ellips2.append([x, y, z])
        
#         return ellips1, ellips2


#     def make_line(self, first_position, second_position):
        
#         step_sizes = [abs(second_position[i] - first_position[i]) / self.res_line for i in range(3)]

#         positions = []

#         residual_term = round(self.res_circle/100)

#         for i in range(residual_term):
#             position = [first_position[0] - (i if first_position[0]<second_position[0] else -i) * step_sizes[0],
#                         first_position[1] - (i if first_position[1]<second_position[1] else -i) * step_sizes[1],
#                         first_position[2] - (i if first_position[2]<second_position[2] else -i) * step_sizes[2]]
#             positions.append(position)
        
#         positions.reverse()

#         for i in range(self.res_line+residual_term):
#             position = [first_position[0] + (i if first_position[0]<second_position[0] else -i) * step_sizes[0],
#                         first_position[1] + (i if first_position[1]<second_position[1] else -i) * step_sizes[1],
#                         first_position[2] + (i if first_position[2]<second_position[2] else -i) * step_sizes[2]]
#             positions.append(position)

#         return positions
    
#     def pub_angles(self, angles):
#         data = SyncSetPosition()
#         data.ax_id = AX_DXL_ID
#         data.ax_position = angles
#         self.pub_angle.publish(data)
        
    
# def main():

#     rospy.init_node('mission_pen')

#     ###### ik 설정 #######

#     inv = inverse_kinematics(l1, l2, l3, l4, a = 3.3)    
    
#     ik = inv.calc_inv_to_ground
#     #ik = inv.calc_inv_to_ground_ver2

#     mp = mission_pen()


#     ###### 미션 설정 #######

#     line = True
#     #line = False
    
#     circle_1 = True
#     #circle_1 = False
    
#     circle_2 = True
#     circle_2 = False


#     ###### circle 미션 인자 설정 #######

#     circle_rate = 25

#     r = 4#3.2

#     # circle_offset = [-0.4, -0.3, -0.45]
#     # circle_pose = [9.2, -4, 0.0]
#     circle_offset = [-0.2, -0.7, -0.1]
#     circle_pose = [9.2, -4, 0.0]
#     # ver1
#     # circle_offset = [-0.0, -0.15, -0.1]
#     # circle_pose = [8.65, -2.2, 0]


#     ###### line 미션 인자 설정 #######

#     line_rate = 30#50
    
#     line_dot1_offset = [-0.25, -0.5, -0.2] #[-0.2 ,0, -0.7] #[-0.9, -0.5, -0.5]
#     line_dot1_pose = [7.5, 4.65, 0.0]

#     line_dot2_offset = [-0.5, -0.4, -0.1] #[-0.2 ,0, -0.7] #[-1, -0.05, -0.5]
#     line_dot2_pose = [3.9, -4.85, 0.0]


#     ########## 미션 시작!!!!!! ##########

#     ellips_positions1, ellips_positions2 = mp.make_ellips2([circle_pose[0] + circle_offset[0], circle_pose[1] + circle_offset[1], circle_pose[2] + circle_offset[2]], r)

#     ellips_positions = mp.make_ellips([circle_pose[0] + circle_offset[0], circle_pose[1] + circle_offset[1], circle_pose[2] + circle_offset[2]], r)

#     line_positions = mp.make_line([line_dot1_pose[0] + line_dot1_offset[0], line_dot1_pose[1] + line_dot1_offset[1], line_dot1_pose[2] + line_dot1_offset[2]], \
#                                 [line_dot2_pose[0] + line_dot2_offset[0], line_dot2_pose[1] + line_dot2_offset[1], line_dot2_pose[2] + line_dot2_offset[2]])

    
#     ## initiatl pose_1 : 일단 기지개 ##
    
#     t = time.time()

#     while(time.time()-t < 2):
    
#         mp.pub_angles([0,0,0,0])

#     ##-----------------------------##

#     if circle_2:

#         first_flag1 = True
#         first_flag2 = True

#         rate = rospy.Rate(circle_rate)

#         for pos in ellips_positions1:
            
#             angles = np.array(ik(pos[0], pos[1], pos[2]))

#             if first_flag1:

#                 ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
#                 t = time.time()
                
#                 while(time.time()-t < 4):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

#                     mp.pub_angles(angles)
#                 ##----------------------------------------------##
                
#                 ## initiatl pose_3 : 첫 지점에서 2초간 대기
#                 t = time.time()

#                 while(time.time()-t < 1):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]))

#                     mp.pub_angles(angles)
#                 ##-----------------------------------##

#                 first_flag1 = False

#                 continue

#             mp.pub_angles(angles)

#             rate.sleep()

#         ## 마지막 포지션 위로 ##

#         t = time.time()

#         while(time.time()-t < 2):

#             # angles = np.array(ik(pos[0], pos[1], pos[2]+3))
#             angles = np.array(ik(circle_pose[0],circle_pose[1],circle_pose[2]+4))

#             mp.pub_angles(angles)        
        
#         ##-----------------##

#         for pos in ellips_positions2:
            
#             angles = np.array(ik(pos[0], pos[1], pos[2]))

#             if first_flag2:

#                 ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
#                 t = time.time()
                
#                 while(time.time()-t < 2):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

#                     mp.pub_angles(angles)
#                 ##----------------------------------------------##
                
#                 ## initiatl pose_3 : 첫 지점에서 1초간 대기
#                 t = time.time()

#                 while(time.time()-t < 1):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]))

#                     mp.pub_angles(angles)
#                 ##-----------------------------------##

#                 first_flag2 = False

#                 continue

#             mp.pub_angles(angles)

#             rate.sleep()


#     if circle_1:

#         first_flag = True

#         rate = rospy.Rate(circle_rate)

#         for pos in ellips_positions:
            
#             angles = np.array(ik(pos[0], pos[1], pos[2]))

#             if first_flag:
                
#                 ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
#                 t = time.time()
                
#                 while(time.time()-t < 8):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]+0.5))

#                     mp.pub_angles(angles)
#                 ##----------------------------------------------##
                
#                 ## initiatl pose_3 : 첫 지점에서 2초간 대기
#                 t = time.time()

#                 while(time.time()-t < 1):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]))

#                     mp.pub_angles(angles)
#                 ##-----------------------------------##

#                 first_flag = False

#                 continue

#             mp.pub_angles(angles)

#             rate.sleep()


#     if line:

#         first_flag = True

#         rate = rospy.Rate(line_rate)

#         for pos in line_positions:
            
#             angles = np.array(ik(pos[0], pos[1], pos[2]))

#             if first_flag:
                
#                 ## initiatl pose_2 : 첫 지점에서 조금 공중에서 4초간 대기
#                 t = time.time()
                
#                 while(time.time()-t < 4):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]+0.6))

#                     mp.pub_angles(angles)
#                 ##---------------------------------------------##
                
#                 ## initiatl pose_3 : 첫 지점에서 2초간 대기
#                 t = time.time()

#                 while(time.time()-t < 1):

#                     angles = np.array(ik(pos[0], pos[1], pos[2]))

#                     mp.pub_angles(angles)
#                 ##---------------------------------------------##

#                 first_flag = False

#                 continue

#             mp.pub_angles(angles)

#             rate.sleep()
#         ## 마지막 포지션 위로 ##

#         t = time.time()

#         while(time.time()-t < 1):

#             # angles = np.array(ik(pos[0], pos[1], pos[2]+3))
#             angles = np.array(ik(pos[0],pos[1],pos[2]+3))

#             mp.pub_angles(angles)        
        
#         ##-----------------##

#     ## final pose : 다시 기지개 ##
    
#     t = time.time()

#     while(time.time()-t < 2):
    
#         mp.pub_angles([0,0,0,0])
    
#     ##------------------------##


#     ########## 미션 끝 ##########


# if __name__ == "__main__":
#     main()