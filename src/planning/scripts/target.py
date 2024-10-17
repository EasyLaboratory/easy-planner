# import numpy as np
# import matplotlib.pyplot as plt

# # 定义ego和target的坐标
# ego_pos = np.array([0, 0])  # ego的初始位置
# target_pos = np.array([3, 4])  # target的位置

# # 计算yaw角度
# delta_y = target_pos[1] - ego_pos[1]
# delta_x = target_pos[0] - ego_pos[0]
# epsilon_yaw = np.arctan2(delta_y, delta_x)

# # 设置图像
# fig, ax = plt.subplots()

# # 画出ego和target的点
# ax.scatter(*ego_pos, color='blue', label='Ego State', zorder=5)
# ax.scatter(*target_pos, color='red', label='Target State', zorder=5)

# # 画出ego和target之间的连线
# ax.plot([ego_pos[0], target_pos[0]], [ego_pos[1], target_pos[1]], 'k--', label=f'Yaw Angle: {np.degrees(epsilon_yaw):.2f}°')

# # 画出ego面朝的方向（即x轴正方向）
# ax.arrow(ego_pos[0], ego_pos[1], 1, 0, head_width=0.2, head_length=0.3, fc='blue', ec='blue', label='Ego X-axis')

# # 图形设置
# ax.set_aspect('equal')
# ax.set_xlabel('X (forward)')
# ax.set_ylabel('Y (left)')
# ax.legend()
# plt.title(f'Ego to Target Yaw Angle (epsilon_yaw) = {np.degrees(epsilon_yaw):.2f}°')
# plt.grid(True)

# plt.show()


import numpy as np
import matplotlib.pyplot as plt

# 目标的速度矢量
target_vel = np.array([3, 4])  # 目标在x方向和y方向的速度分量

# 计算目标物体的yaw角度
target_yaw = np.arctan2(target_vel[1], target_vel[0])

# 设置图像
fig, ax = plt.subplots()

# 画出目标物体的位置
target_pos = np.array([0, 0])  # 目标的位置在原点
ax.scatter(*target_pos, color='red', label='Target State', zorder=5)

# 画出目标物体的速度矢量
ax.arrow(target_pos[0], target_pos[1], target_vel[0], target_vel[1], 
         head_width=0.2, head_length=0.3, fc='red', ec='red', label=f'Velocity Vector')

# 图形设置
ax.set_aspect('equal')
ax.set_xlabel('X (forward)')
ax.set_ylabel('Y (left)')
ax.legend()
plt.title(f'Target Yaw Angle (target_yaw) = {np.degrees(target_yaw):.2f}°')
plt.grid(True)

plt.show()
