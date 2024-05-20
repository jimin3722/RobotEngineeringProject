#!/usr/bin/env python
# -- coding: utf-8 --

import math
import numpy as np


THETA_LIMIT = 130*math.pi/180


class inverse_kinematics:

    def __init__(self, l1, l2, l3, l4):
        
        # 링크 길이
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

        # world 좌표계와 원점 좌표계 사이 translation matrix 
        self.T_U0 = [-6, 0, 0]

    def calc_inv_to_ground(self,px,py,pz):

        try:

            px -= self.T_U0[0]
            py -= self.T_U0[1]
            pz -= self.T_U0[2]

            # 어차피 link_4는 지면과 수직이니까
            pz += self.l4

            theta_1 = math.atan2(py,px)
            
            if abs(math.sin(theta_1)) < 0.000001:
                alpha = px/math.cos(theta_1)
            else:
                alpha = py/math.sin(theta_1)

            beta = - pz + self.l1

            c3 = (alpha**2 + beta**2 - self.l2**2 - self.l3**2)/(2*self.l2*self.l3)

            s3_1 = +math.sqrt(1-c3**2)
            s3_2 = -math.sqrt(1-c3**2)

            theta_3_1 = math.atan2(s3_1, c3)
            theta_3_2 = math.atan2(s3_2, c3)

            u_1 = self.l3*math.cos(theta_3_1) + self.l2
            v_1 = self.l3*math.sin(theta_3_1)
            
            u_2 = self.l3*math.cos(theta_3_2) + self.l2
            v_2 = self.l3*math.sin(theta_3_2)

            pi_1 = math.atan2(v_1, u_1)
            pi_2 = math.atan2(v_2, u_2)

            theta_2_1 = math.atan2(beta, alpha) - pi_1
            theta_2_2 = math.atan2(beta, alpha) - pi_2

            # 각도 하나의 해로 클리핑
            if(theta_2_1 < 0):
                theta_2 = theta_2_1
                theta_3 = theta_3_1
            else:
                theta_2 = theta_2_2
                theta_3 = theta_3_2

            theta_4 = math.pi/2 - theta_2 - theta_3 

            theta_list = [ theta_1, theta_2 + math.pi/2, theta_3, theta_4 ]

            # 엔드 이펙터 좌표가 원점과 너무 가까우면 3번째 joint가 가동범위를 벗어나는 상황이 생김
            # >> 이런 경우는 3번째 joint를 THETA_LIMIT으로 고정하고, 4번째 joint를 지면과 수직으로 고정이 아닌 자유도를 준다.
            if abs(theta_3) > THETA_LIMIT:

                theta_3 = THETA_LIMIT

                ### theta_3 = THETA_LIMIT 고정시키고 다시 inverse 구하는 코드 ###


            # 이 외의 joint각도가 THETA_LIMIT을 벗어나는 지 확인
            for idx, theta in enumerate(theta_list):

                if abs(theta) > THETA_LIMIT:

                    print("theta_3 : ",theta*180/math.pi)

                    theta_list[idx] = 0
                    
                    print(idx+1, "번 모터가 가동범위를 벗어났습니다.!!!!!") 

            return theta_list
        
        except ValueError as e:
            
            print("해가 없습니다.", e)


# 예시코드
def main():

    inv = inverse_kinematics(l1 = 11.75, l2 = 15, l3 = 15, l4 = 7)

    qs = np.array(inv.calc_inv_to_ground(3,9,3))
    qs = qs*180/math.pi

    print(qs)

if __name__ == "__main__":
    main()