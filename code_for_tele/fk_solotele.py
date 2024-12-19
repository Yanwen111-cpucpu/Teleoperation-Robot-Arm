import numpy as np
import arm_control_sim
import time
import sys  # 用于安全退出

# Denavit-Hartenberg 参数表
dh_params = [
    [0, 0.33, 0.05, np.pi / 2],  # 第1关节
    [0, 0, 0.44, 0],             # 第2关节
    [0, 0, 0.035, np.pi / 2],    # 第3关节
    [0, 0.42, 0, -np.pi / 2],    # 第4关节
    [0, 0, 0, np.pi / 2],        # 第5关节
    [0, 0.08, 0, 0],             # 第6关节
]

def dh_transformation_matrix(theta, d, a, alpha):
    """
    计算每个关节的DH变换矩阵。
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),                d],
        [0,              0,                            0,                            1]
    ])

def forward_kinematics(joint_angles):
    """
    使用正运动学计算六轴机械臂末端空间坐标。
    """
    assert len(joint_angles) == 6, "需要提供6个关节角度"
    
    # 从基坐标开始，初始化变换矩阵
    transformation_matrix = np.eye(4)
    
    # 遍历每个关节，计算总变换矩阵
    for i in range(6):
        theta, d, a, alpha = dh_params[i]
        # 将关节角度叠加到theta
        theta += joint_angles[i]
        # 当前关节的变换矩阵
        current_matrix = dh_transformation_matrix(theta, d, a, alpha)
        # 累乘总变换矩阵
        transformation_matrix = np.dot(transformation_matrix, current_matrix)
    
    # 提取末端坐标
    x, y, z = transformation_matrix[:3, 3]
    return x, y, z

class fk_arm:
    
    def __init__(self):
        self.dxl_arm = arm_control_sim.DXL_Arm()
        self.init_angle = self.dxl_arm.get_joint_angle()
        self.gripper=0
        self.joint0=0
        self.joint1=0

    def get_ee_coordinate(self):
        current_angle = self.dxl_arm.get_joint_angle()
        self.joint0=(current_angle[0]-self.init_angle[0])/3.14*180
        self.joint1=(current_angle[1]-self.init_angle[1])/3.14*180
        self.joint2=(current_angle[2]-self.init_angle[2])/3.14*180
        self.joint3=(current_angle[4]-self.init_angle[4])/3.14*180
        ee_state=list(forward_kinematics(current_angle - self.init_angle))
        ee_state.append(self.gripper)

        for i in range(3):
            ee_state[i]+=0.1 #调整reachable space

        return(ee_state)

if __name__ == "__main__":
    arm = fk_arm()
    try:
        while True:
            print(arm.get_ee_coordinate())
            time.sleep(0.5)  # 降低刷新频率，避免过多打印
    except KeyboardInterrupt:
        print("主程序中断，安全退出")
        sys.exit(0)  # 确保程序完全退出
