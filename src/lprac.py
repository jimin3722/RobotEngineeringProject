import numpy as np
import matplotlib.pyplot as plt

ellips = []

# 타원의 중심
center_x = 1
center_y = 2

# 장축과 단축의 길이
a = 5 / 2  # 장축 반경
b = 4 / 2  # 단축 반경

# 타원의 각도 (중점에서의 벡터와 평행)
theta = np.arctan2(2, 1)  # 벡터 (1, 0)과 평행함

# 타원의 점들 생성
ts = np.linspace(0, 2 * np.pi, 200)

# 각 부분의 좌표 계산
for t in ts:

    x = center_x + a * np.cos(t) * np.cos(theta) - b * np.sin(t) * np.sin(theta)
    y = center_y + a * np.cos(t) * np.sin(theta) + b * np.sin(t) * np.cos(theta)
    ellips.append([x, y])

# 점들 시각화
plt.plot(ellips)
plt.scatter(center_x, center_y, color='red')  # 중심 표시
plt.gca().set_aspect('equal', adjustable='box')
plt.title('타원')
plt.xlabel('X축')
plt.ylabel('Y축')
plt.grid()
plt.show()