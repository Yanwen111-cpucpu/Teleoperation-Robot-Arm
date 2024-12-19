import serial
import time
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import fk_solotele
import keyboard

# # 蓝牙和串口设置
# BLUETOOTH_PORT = "COM5"  # 替换为HC05模块的蓝牙端口
# BLUETOOTH_BAUDRATE = 9600

SERIAL_PORT = "COM8"  # 替换为Arduino的串口端口
SERIAL_BAUDRATE = 9600

def send_joint_angles_to_arduino(joint_angles, gripper_angle, serial_conn):
    """
    将关节角度通过串口发送到Arduino。
    :param joint_angles: 关节角度列表，例如 [angle1, angle2, angle3, angle4]
    :param gripper_angle: 抓手角度
    :param serial_conn: 串口对象
    """
    try:
        # 构造要发送的字符串
        data = ",".join([f"{angle:.2f}" for angle in joint_angles]) + f",{gripper_angle:.2f}\n"
        # data_joint2="45\n"
        serial_conn.write(data.encode())  # 发送数据
        # serial_conn.write(data_joint2.encode())  # 发送数据
        print(f"发送数据到Arduino: {data}")

        # 清空输入缓冲区，避免读取到旧数据
        serial_conn.reset_input_buffer()
        
        # 等待 Arduino 的响应
        time.sleep(0.2)
        if serial_conn.in_waiting > 0:
            response = serial_conn.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino 回复: {response}")
    except Exception as e:
        print(f"发送失败: {e}")


# 定义 5 DOF 机械臂的链条
my_chain = Chain(name='5DOF_arm', links=[
    # 基础链接 (起点)
    OriginLink(),

    # Base (沿 z 轴旋转)
    URDFLink(
        name="base",
        origin_translation=[0, 0, 0.04],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
        # bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
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

    # # Joint 3 (沿 x 轴旋转)
    # URDFLink(
    #     name="joint_3",
    #     origin_translation=[0.06, 0, 0],
    #     origin_orientation=[0, 0, 0],
    #     rotation=[1, 0, 0],
    #     bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
    # ),

    # Joint 4: gripper (沿 x 轴旋转)
    URDFLink(
        name="gripper",
        origin_translation=[0.17, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],
        bounds=(-np.pi * 3 / 4, np.pi * 3 / 4)
    )
])

# 假设已经有一个逆运动学函数
def inverse_kinematics(x, y, z):
    ik_solution = my_chain.inverse_kinematics([x,y,z])
    return ik_solution[:4]

# # 初始化蓝牙和串口连接
# def init_connections():
#     try:
#         bt_conn = serial.Serial(BLUETOOTH_PORT, BLUETOOTH_BAUDRATE, timeout=1)
#         print("蓝牙连接成功")
#     except Exception as e:
#         print(f"蓝牙连接失败: {e}")
#         return None, None

#     try:
#         serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
#         print("串口连接成功")
#     except Exception as e:
#         print(f"串口连接失败: {e}")
#         return bt_conn, None

#     return bt_conn, serial_conn

# 主循环
def main():
    # bt_conn, serial_conn = init_connections()
    # if not bt_conn or not serial_conn:
    #     print("初始化连接失败")
    #     return

        # 初始化串口
    try:
        serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        print("串口连接成功")
    except Exception as e:
        print(f"串口连接失败: {e}")
        return

    arm=fk_solotele.fk_arm()
    gripper_state = 0 
    try:
        while True:
            if keyboard.is_pressed('a'):  # 检测键盘输入 'a'
                gripper_state = 120 if gripper_state == 0 else 0
                print(f"夹爪状态已切换，当前状态: {'打开' if gripper_state == 120 else '关闭'}")
                time.sleep(0.5)  # 防止过快切换
            start_time = time.time()  # 记录起始时间

            ee_state=arm.get_ee_coordinate()
            x=ee_state[0]
            y=ee_state[1]
            z=ee_state[2]

            # 控制抓手

            # 逆运动学计算关节角度aa
            joint_angles = inverse_kinematics(x, y, z)/3.14*180+45
            joint_angles[0]=arm.joint0+150
            joint_angles[1]=arm.joint1+10
            # joint_angles[2]=135-arm.joint2
            joint_angles[3]=arm.joint3+60
            if joint_angles is None or joint_angles.size == 0:
                print(f"逆运动学计算失败: {x, y, z}")
                continue

            send_joint_angles_to_arduino(joint_angles, gripper_state, serial_conn)
            time.sleep(0.05)  # 控制发送频率

            # 将关节角度拼接为字符串
            angle_data = ",".join([f"joint{i+1}:{angle:.2f}" for i, angle in enumerate(joint_angles)])
            angle_data += f",gripper:{gripper_state}"
            print(angle_data)
    except KeyboardInterrupt:
        print("程序终止")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        serial_conn.close()  # 关闭串口连接
if __name__ == "__main__":
    main()
