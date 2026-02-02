# 文件名: robot_lib_arm.py
# 功能：ROS2 机械臂控制功能的 Python 封装
# 作者：自动生成
# 日期：2026-02-02

import subprocess
import time
import math
import sys

class RobotArm:
    """
    机器人机械臂控制类
    
    该类封装了 ROS2 机械臂的控制功能，包括：
    - 关节空间控制（直接控制关节角度）
    - 笛卡尔空间控制（控制末端位置）
    - 夹爪控制
    - 云台/yaw轴控制
    - PWM输出控制
    - 状态反馈（关节角度、末端位置）
    
    适用于两自由度机械臂（大臂+小臂）+ 独立云台 + 夹爪的配置
    """
    
    def __init__(self):
        """
        构造函数：初始化机械臂控制类
        
        成员变量：
        - joint_states: 当前关节状态 [joint1, joint2, yaw]
        - end_effector_pose: 末端执行器位置 {x, y}
        - gripper_state: 夹爪状态 (0-10)
        - arm_length_1: 大臂长度（mm）
        - arm_length_2: 小臂长度（mm）
        """
        self.joint_states = [0.0, 0.0, 0.0]  # [joint1, joint2, yaw]
        self.end_effector_pose = {"x": 0.0, "y": 0.0}
        self.gripper_state = 0
        
        # 机械臂参数（需要根据实际机械臂调整）
        self.arm_length_1 = 200.0  # 大臂长度 200mm
        self.arm_length_2 = 150.0  # 小臂长度 150mm
        
        # 关节限位（度）
        self.joint1_limit = [-90, 90]   # 大臂关节限位
        self.joint2_limit = [-90, 90]   # 小臂关节限位
        self.yaw_limit = [-180, 180]    # 云台限位
    
    def arm_home(self):
        """
        机械臂复位到初始姿态
        
        功能说明：
        将机械臂移动到预定义的初始位置（通常是垂直向上或安全位置）。
        
        封装原理：
        1. 发布预定义的初始姿态角度到关节控制话题
        2. 通常初始姿态为 joint1=0°, joint2=0°（根据实际情况调整）
        3. 等待机械臂运动到位
        
        参数：
        - 无
        
        返回值：
        - bool: 复位成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.arm_home()  # 复位到初始姿态
        """
        print("[机械臂] 复位到初始姿态...")
        
        # 定义初始姿态角度（根据实际机械臂调整）
        home_joint1 = 0.0   # 大臂角度
        home_joint2 = 0.0   # 小臂角度
        home_yaw = 0.0      # 云台角度
        
        # 发送关节角度命令
        success = self.set_joint_angles(home_joint1, home_joint2)
        if success:
            self.set_yaw_angle(home_yaw)
            
            # 夹爪打开
            self.set_gripper(10)
            
            print("[机械臂] 复位完成")
            return True
        else:
            print("[机械臂] 复位失败")
            return False
    
    def set_joint_angles(self, joint1, joint2):
        """
        设置机械臂关节角度
        
        功能说明：
        直接控制机械臂的两个主关节旋转到指定角度。
        
        封装原理：
        1. 检查角度是否在限位范围内
        2. 将角度转换为弧度（如果底层使用弧度）
        3. 构建关节控制消息
        4. 发布到机械臂关节控制话题（如 /arm_joint_command）
        
        参数：
        - joint1 (float): 大臂（靠近底座的关节）角度，单位度
        - joint2 (float): 小臂（第二段关节）角度，单位度
        
        返回值：
        - bool: 命令发送成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.set_joint_angles(45, -30)  # 大臂45度，小臂-30度
        """
        # 检查限位
        if not self._check_joint_limit(joint1, self.joint1_limit, "joint1"):
            return False
        if not self._check_joint_limit(joint2, self.joint2_limit, "joint2"):
            return False
        
        print(f"[机械臂] 设置关节角度: Joint1={joint1}°, Joint2={joint2}°")
        
        # 转换为弧度
        joint1_rad = math.radians(joint1)
        joint2_rad = math.radians(joint2)
        
        # 构建消息（假设使用 Float64MultiArray）
        msg = f"{{data: [{joint1_rad}, {joint2_rad}]}}"
        
        topic_name = "/arm_joint_command"
        msg_type = "std_msgs/msg/Float64MultiArray"
        
        try:
            cmd = [
                "ros2", "topic", "pub",
                "--once",
                topic_name,
                msg_type,
                msg
            ]
            
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            # 更新状态
            self.joint_states[0] = joint1
            self.joint_states[1] = joint2
            
            return True
            
        except Exception as e:
            print(f"[错误] 设置关节角度失败: {e}")
            print("[提示] 请确保机械臂驱动节点已启动")
            return False
    
    def set_yaw_angle(self, angle):
        """
        设置云台/yaw轴角度
        
        功能说明：
        控制机械臂的独立旋转关节（通常用于调整末端朝向或相机角度）。
        
        封装原理：
        1. 检查角度限位
        2. 发布到云台控制话题
        3. 底层驱动将角度转换为舵机信号
        
        参数：
        - angle (float): 云台角度，单位度
        
        返回值：
        - bool: 命令发送成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.set_yaw_angle(45)  # 云台旋转到45度
        """
        # 检查限位
        if not self._check_joint_limit(angle, self.yaw_limit, "yaw"):
            return False
        
        print(f"[机械臂] 设置云台角度: {angle}°")
        
        # 转换为弧度
        angle_rad = math.radians(angle)
        
        # 构建消息
        msg = f"{{data: {angle_rad}}}"
        
        topic_name = "/arm_yaw_command"
        msg_type = "std_msgs/msg/Float64"
        
        try:
            cmd = [
                "ros2", "topic", "pub",
                "--once",
                topic_name,
                msg_type,
                msg
            ]
            
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            # 更新状态
            self.joint_states[2] = angle
            
            return True
            
        except Exception as e:
            print(f"[错误] 设置云台角度失败: {e}")
            return False
    
    def set_arm_position(self, x, y):
        """
        设置机械臂末端位置（笛卡尔空间控制）
        
        功能说明：
        通过逆运动学解算，控制机械臂末端移动到指定的平面坐标。
        
        封装原理：
        1. 调用逆运动学函数计算所需的关节角度
        2. 检查解是否存在（目标点是否在工作空间内）
        3. 调用 set_joint_angles 执行运动
        
        数学原理：
        对于平面两关节机械臂：
        - 正运动学: x = L1*cos(θ1) + L2*cos(θ1+θ2)
                   y = L1*sin(θ1) + L2*sin(θ1+θ2)
        - 逆运动学: 通过几何关系求解 θ1 和 θ2
        
        参数：
        - x (float): 目标点X坐标（相对于机械臂底座），单位 mm
        - y (float): 目标点Y坐标，单位 mm
        
        返回值：
        - bool: 运动成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.set_arm_position(250, 100)  # 移动到 (250mm, 100mm)
        """
        print(f"[机械臂] 移动末端到: ({x}mm, {y}mm)")
        
        # 调用逆运动学求解
        solution = self._inverse_kinematics(x, y)
        
        if solution is None:
            print("[错误] 目标位置超出工作空间")
            return False
        
        joint1, joint2 = solution
        
        # 执行运动
        return self.set_joint_angles(joint1, joint2)
    
    def set_gripper(self, value):
        """
        控制夹爪开合
        
        功能说明：
        控制机械臂末端夹爪的开合程度。
        
        封装原理：
        1. 将 0-10 的数值映射到夹爪的实际控制范围
        2. 发布到夹爪控制话题
        3. 底层驱动将数值转换为舵机 PWM 信号
        
        参数：
        - value (int/float): 夹爪开合程度，0=完全闭合，10=完全打开
        
        返回值：
        - bool: 命令发送成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.set_gripper(10)  # 完全打开
        >>> arm.set_gripper(0)   # 完全闭合
        >>> arm.set_gripper(5)   # 半开
        """
        # 限制范围 0-10
        value = max(0, min(10, value))
        
        print(f"[机械臂] 设置夹爪: {value}/10")
        
        # 构建消息
        msg = f"{{data: {value}}}"
        
        topic_name = "/gripper_command"
        msg_type = "std_msgs/msg/Float32"
        
        try:
            cmd = [
                "ros2", "topic", "pub",
                "--once",
                topic_name,
                msg_type,
                msg
            ]
            
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            # 更新状态
            self.gripper_state = value
            
            return True
            
        except Exception as e:
            print(f"[错误] 设置夹爪失败: {e}")
            return False
    
    def get_arm_pose_xy(self):
        """
        获取机械臂末端的笛卡尔坐标
        
        功能说明：
        通过正运动学计算或订阅反馈话题，获取末端位置。
        
        封装原理：
        方法1：从关节状态计算（正运动学）
        方法2：订阅末端位姿反馈话题
        
        返回值：
        - tuple: (x, y) 坐标，单位 mm；失败返回 (0, 0)
        
        使用示例：
        >>> arm = RobotArm()
        >>> x, y = arm.get_arm_pose_xy()
        >>> print(f"末端位置: ({x}, {y})")
        """
        # 方法1：从当前关节状态计算
        joint1_rad = math.radians(self.joint_states[0])
        joint2_rad = math.radians(self.joint_states[1])
        
        # 正运动学计算
        x = (self.arm_length_1 * math.cos(joint1_rad) + 
             self.arm_length_2 * math.cos(joint1_rad + joint2_rad))
        y = (self.arm_length_1 * math.sin(joint1_rad) + 
             self.arm_length_2 * math.sin(joint1_rad + joint2_rad))
        
        self.end_effector_pose = {"x": x, "y": y}
        
        return (x, y)
    
    def get_joint_states(self):
        """
        获取当前所有关节的实时角度
        
        功能说明：
        从机械臂驱动节点订阅关节状态反馈。
        
        封装原理：
        1. 订阅关节状态话题 /joint_states
        2. 消息类型为 sensor_msgs/msg/JointState
        3. 提取关节角度值
        
        返回值：
        - tuple: (joint1_angle, joint2_angle, yaw_angle)，单位度
        
        使用示例：
        >>> arm = RobotArm()
        >>> j1, j2, yaw = arm.get_joint_states()
        >>> print(f"关节角度: {j1}°, {j2}°, {yaw}°")
        """
        topic_name = "/joint_states"
        
        try:
            cmd = [
                "ros2", "topic", "echo",
                topic_name,
                "--once"
            ]
            
            output = subprocess.check_output(
                cmd,
                timeout=1.0,
                stderr=subprocess.DEVNULL
            ).decode("utf-8")
            
            # 解析关节角度（需要根据实际消息格式调整）
            import re
            # 提取 position 数组
            pos_match = re.search(r"position:.*?\[(.*?)\]", output, re.DOTALL)
            
            if pos_match:
                positions_str = pos_match.group(1)
                positions = [float(x.strip()) for x in positions_str.split(",")]
                
                # 假设顺序为 [joint1, joint2, yaw]
                if len(positions) >= 3:
                    # 转换为角度
                    joint1 = math.degrees(positions[0])
                    joint2 = math.degrees(positions[1])
                    yaw = math.degrees(positions[2])
                    
                    # 更新状态
                    self.joint_states = [joint1, joint2, yaw]
                    
                    return (joint1, joint2, yaw)
            
            # 如果解析失败，返回缓存的状态
            return tuple(self.joint_states)
            
        except subprocess.TimeoutExpired:
            print("[警告] 获取关节状态超时")
            return tuple(self.joint_states)
        except Exception as e:
            print(f"[错误] 获取关节状态失败: {e}")
            return tuple(self.joint_states)
    
    def set_pwm(self, pin, duty_cycle):
        """
        设置主控板IO引脚的PWM占空比
        
        功能说明：
        控制主控板的PWM输出引脚，可用于控制舵机、电机调速等。
        
        封装原理：
        1. 构建PWM控制消息
        2. 发布到PWM控制话题
        3. 底层驱动将占空比转换为实际的PWM信号
        
        参数：
        - pin (int): IO引脚编号（1-16，具体取决于主控板）
        - duty_cycle (float): 占空比，范围 0.0-1.0 (0%=0.0, 100%=1.0)
        
        返回值：
        - bool: 命令发送成功返回 True，失败返回 False
        
        使用示例：
        >>> arm = RobotArm()
        >>> arm.set_pwm(1, 0.5)   # 设置引脚1为50%占空比
        >>> arm.set_pwm(2, 0.75)  # 设置引脚2为75%占空比
        """
        # 限制占空比范围
        duty_cycle = max(0.0, min(1.0, duty_cycle))
        
        print(f"[PWM] 设置引脚 {pin} 占空比: {duty_cycle*100:.1f}%")
        
        # 构建消息
        # 假设使用自定义消息或 Float32MultiArray
        msg = f"{{data: [{pin}, {duty_cycle}]}}"
        
        topic_name = "/pwm_command"
        msg_type = "std_msgs/msg/Float32MultiArray"
        
        try:
            cmd = [
                "ros2", "topic", "pub",
                "--once",
                topic_name,
                msg_type,
                msg
            ]
            
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            return True
            
        except Exception as e:
            print(f"[错误] 设置PWM失败: {e}")
            print("[提示] 该功能需要底盘驱动支持，可能需要修改串口协议")
            return False
    
    # ==================== 辅助函数 ====================
    
    def _inverse_kinematics(self, x, y):
        """
        内部函数：逆运动学求解
        
        功能说明：
        根据目标位置计算所需的关节角度。
        
        参数：
        - x, y: 目标位置坐标（mm）
        
        返回值：
        - tuple: (joint1, joint2) 关节角度（度），无解返回 None
        """
        L1 = self.arm_length_1
        L2 = self.arm_length_2
        
        # 计算目标点距离原点的距离
        distance = math.sqrt(x**2 + y**2)
        
        # 检查是否在工作空间内
        if distance > (L1 + L2) or distance < abs(L1 - L2):
            return None
        
        # 使用余弦定理求解 joint2
        cos_joint2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        
        # 检查是否有有效解
        if abs(cos_joint2) > 1.0:
            return None
        
        # 计算 joint2（选择肘部向上的解）
        joint2_rad = math.acos(cos_joint2)
        
        # 计算 joint1
        alpha = math.atan2(y, x)
        beta = math.atan2(L2 * math.sin(joint2_rad),
                         L1 + L2 * math.cos(joint2_rad))
        joint1_rad = alpha - beta
        
        # 转换为角度
        joint1 = math.degrees(joint1_rad)
        joint2 = math.degrees(joint2_rad)
        
        return (joint1, joint2)
    
    def _check_joint_limit(self, angle, limit, joint_name):
        """
        内部函数：检查关节角度是否在限位范围内
        
        参数：
        - angle: 目标角度（度）
        - limit: 限位范围 [min, max]
        - joint_name: 关节名称（用于提示）
        
        返回值：
        - bool: 在范围内返回 True，超出返回 False
        """
        if angle < limit[0] or angle > limit[1]:
            print(f"[警告] {joint_name} 角度 {angle}° 超出限位范围 {limit}")
            return False
        return True


# 模块测试代码
if __name__ == "__main__":
    print("=" * 60)
    print("ROS2 机械臂控制功能测试")
    print("=" * 60)
    
    arm = RobotArm()
    
    try:
        print("\n[测试1] 机械臂复位...")
        arm.arm_home()
        time.sleep(2)
        
        print("\n[测试2] 设置关节角度...")
        arm.set_joint_angles(45, -30)
        time.sleep(2)
        
        print("\n[测试3] 获取关节状态...")
        j1, j2, yaw = arm.get_joint_states()
        print(f"关节角度: J1={j1:.1f}°, J2={j2:.1f}°, Yaw={yaw:.1f}°")
        
        print("\n[测试4] 获取末端位置...")
        x, y = arm.get_arm_pose_xy()
        print(f"末端位置: ({x:.1f}mm, {y:.1f}mm)")
        
        print("\n[测试5] 笛卡尔空间运动...")
        arm.set_arm_position(250, 100)
        time.sleep(2)
        
        print("\n[测试6] 夹爪控制...")
        arm.set_gripper(0)   # 闭合
        time.sleep(1)
        arm.set_gripper(10)  # 打开
        
        print("\n测试完成！")
        
    except KeyboardInterrupt:
        print("\n\n测试被中断")
