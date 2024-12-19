import numpy as np
import arm_control_sim
import serial
import time

def send_coordinates_to_arduino(port, baudrate, init_angle):
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"已连接到 Arduino: {port}，波特率: {baudrate}")
        while True:  # 进入传输坐标的循环
            try:
                joint_angles = arm.get_joint_angle() - init_angle
                coordinates = forward_kinematics(joint_angles)  # 计算末端坐标
                message = f"{coordinates[0]:.2f} {coordinates[1]:.2f} {coordinates[2]:.2f}\n"
                
                # 发送数据
                ser.write(message.encode('utf-8'))  # 将字符串编码为字节发送
                print(f"发送数据: {message.strip()}")
                
                ser.reset_input_buffer()
                # 等待 Arduino 的响应
                time.sleep(0.2)
                
                if ser.in_waiting > 0:
                    # 尝试解码数据，忽略无法解码的字节
                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Arduino 回复: {response}")

            
            except KeyboardInterrupt:
                print("用户中断，退出循环")
                break  # 跳出循环
            except Exception as e:
                print(f"传输数据出错: {e}")
                break  # 跳出循环
        
    finally:
        # 确保退出循环时释放串口
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("串口已关闭")

# Denavit-Hartenberg 参数表
# [theta, d, a, alpha]
dh_params = [
    [0, 0.33, 0.05, np.pi/2],  # 第1关节
    [0, 0, 0.44, 0],          # 第2关节
    [0, 0, 0.035, np.pi/2],    # 第3关节
    [0, 0.42, 0, -np.pi/2],   # 第4关节
    [0, 0, 0, np.pi/2],        # 第5关节
    [0, 0.08, 0, 0],           # 第6关节
]

def dh_transformation_matrix(theta, d, a, alpha):
    """
    计算每个关节的DH变换矩阵。
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),              d],
        [0,              0,                           0,                          1]
    ])

def forward_kinematics(joint_angles):
    """
    使用正运动学计算六轴机械臂末端空间坐标。
    :param joint_angles: 六个关节角度（单位：弧度）
    :return: 末端在空间中的坐标 [x, y, z]
    """
    assert len(joint_angles) == 6, "需要提供6个关节角度"
    
    # 从基坐标开始，初始化变换矩阵
    transformation_matrix = np.eye(4)
    
    # 遍历每个关节，计算总变换矩阵
    for i in range(6):
        theta, d, a, alpha = dh_params[i]
        # 将关节角度叠加到theta
        theta = joint_angles[i]
        # 当前关节的变换矩阵
        current_matrix = dh_transformation_matrix(theta, d, a, alpha)
        # 累乘总变换矩阵
        transformation_matrix = np.dot(transformation_matrix, current_matrix)
    
    # 提取末端坐标
    x, y, z = transformation_matrix[:3, 3]
    return x, y, z

if __name__ == "__main__":
    arm = arm_control_sim.DXL_Arm()
    init_angle = arm.get_joint_angle()
    try:
        send_coordinates_to_arduino(port="COM14", baudrate=9600, init_angle=init_angle)
    except KeyboardInterrupt:
        print("主程序中断，退出")
