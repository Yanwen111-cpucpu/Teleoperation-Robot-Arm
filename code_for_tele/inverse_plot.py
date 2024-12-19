from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

# 定义 5 DOF 机械臂的链条
my_chain = Chain(name='5DOF_arm', links=[
    # 基础链接 (固定，且不设置为活动链接)
    OriginLink(),

    # Base (沿 z 轴旋转)
    URDFLink(
        name="base",
        origin_translation=[0, 0, 0.04],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),

    # Joint 1 (沿 x 轴旋转)
    URDFLink(
        name="joint_1",
        origin_translation=[0, 0, 0.1],
        origin_orientation=[np.pi, np.pi, np.pi],
        rotation=[1, 0, 0],
        bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
    ),

    # Joint 2 (沿 x 轴旋转)
    URDFLink(
        name="joint_2",
        origin_translation=[0.1, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],
        bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
    ),

    # Joint 4: gripper (沿 x 轴旋转)
    URDFLink(
        name="gripper",
        origin_translation=[0.17, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],
        bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
    )
])


print(my_chain)

import matplotlib.pyplot as plt
from ikpy.utils.plot import plot_chain

# 目标位置 (target position)
target_position = [0.15, 0.2, 0.17]

# 计算目标位置的逆运动学解
ik_solution = my_chain.inverse_kinematics(target_position)
# 检查实际末端位置
actual_position = my_chain.forward_kinematics(ik_solution)[:3, 3]
error = np.linalg.norm(np.array(target_position) - actual_position)

print(f"目标位置: {target_position}")
print(f"实际末端位置: {actual_position}")
print(f"位置误差: {error}")

if error > 0.01:  # 误差阈值
    print("目标位置不在可达范围内。")
# 打印逆运动学解
print("Inverse Kinematics Solution:")
print(ik_solution)
print(type(ik_solution))


# 创建一个3D图
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# 绘制机械臂（ik最终位置）
plot_chain(chain=my_chain, ax=ax, joints=ik_solution)




# 定义机械臂的默认关节角度（初始姿态）
joints = [0] * len(my_chain.links)  # 每个关节的默认角度为 0

# 绘制机械臂链条（初始位置）
plot_chain(chain=my_chain, joints=joints, ax=ax)


# # 绘制每个关节的坐标轴
# for link in my_chain.links:
#     # 获取每个关节的变换矩阵（表示关节的位置和方向）
#     transform_matrix = link.get_transformation_matrix(joints=my_chain.joints)
#     origin = transform_matrix[:3, 3]  # 提取坐标轴的原点
#     x_axis = transform_matrix[:3, 0]  # 提取 X 轴方向
#     y_axis = transform_matrix[:3, 1]  # 提取 Y 轴方向
#     z_axis = transform_matrix[:3, 2]  # 提取 Z 轴方向
#
#     # 绘制坐标轴
#     scale = 0.05  # 坐标轴长度
#     ax.quiver(*origin, *x_axis * scale, color="r", label="X-axis" if link.name == my_chain.links[1].name else "")
#     ax.quiver(*origin, *y_axis * scale, color="g", label="Y-axis" if link.name == my_chain.links[1].name else "")
#     ax.quiver(*origin, *z_axis * scale, color="b", label="Z-axis" if link.name == my_chain.links[1].name else "")

# # 去除多余的标签，确保每个轴只显示一次标签
# handles, labels = plt.gca().get_legend_handles_labels()
# by_label = dict(zip(labels, handles))
# plt.legend(by_label.values(), by_label.keys())

# 设置图形细节
ax.set_title("5 DOF Robotic Arm with Joint Axes", fontsize=14)
ax.set_xlabel("X axis")
ax.set_ylabel("Y axis")
ax.set_zlabel("Z axis")

# 调整连杆的粗细
for line in ax.get_lines():
    line.set_linewidth(2)  # 设置线宽

# 调整视角
ax.view_init(elev=20, azim=60)  # 设置俯仰角和方位角

# 显示图形
plt.show()
