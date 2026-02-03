# 文件名: robot_lib.py
import subprocess
import time
import signal
import re
import os

class Robot:
    def __init__(self):
        self.driver_process = None  # 底盘驱动进程句柄
        self.keyboard_process = None # 键盘进程句柄
        
        # 定义车型映射关系
        # 键是 initialize 传入的简写，值是传给 launch 文件的具体参数
        # 如果你的 launch 文件不需要转换，可以直接传值
        self.ROBOT_TYPE_MAP = {
            "akm": "ackermann",    # 阿克曼
            "diff": "diff",        # 差速
            "mec": "mecanum"       # 麦轮
        }

    def initialize(self, robot_type):
        """
        初始化机器人底盘
        :param robot_type: str, 车型枚举 "akm", "diff", "mec"
        """
        # 1. 检查参数合法性
        if robot_type not in self.ROBOT_TYPE_MAP:
            print(f"[Error] 未知的车型: {robot_type}. 支持: {list(self.ROBOT_TYPE_MAP.keys())}")
            return False

        # 保存车型信息，供后续函数使用
        self.robot_type = robot_type

        # 获取实际的 ROS 参数值
        real_type_name = self.ROBOT_TYPE_MAP[robot_type]
        print(f"[System] 正在启动 {real_type_name} ({robot_type}) 底盘驱动...")

        # 2. 构建命令
        # 假设 launch 文件通过 robot_type:=xxx 来区分车型
        # 注意：你需要确认你的 launch 文件是否接受这个参数。
        # 如果是 Wheeltec 某些旧版本，可能不需要传参，而是依赖环境变量，这里演示传参方式：
        cmd = [
            "ros2", "launch", 
            "turn_on_wheeltec_robot", 
            "turn_on_wheeltec_robot.launch.py",
            f"robot_type:={real_type_name}" 
        ]

        try:
            # 启动后台进程
            self.driver_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # 屏蔽底层日志，保持清爽
                stderr=subprocess.PIPE     # 保留错误输出以便调试
            )
            
            # 3. 强制等待硬件初始化 (雷达/IMU/串口)
            print("[System] 正在初始化硬件，请等待 5 秒...")
            time.sleep(5)
            
            # 检查进程是否因为报错而立刻退出了
            if self.driver_process.poll() is not None:
                print("[Error] 驱动启动失败！请检查错误日志。")
                return False
                
            print(f"[System] {real_type_name} 底盘初始化完成！")
            return True

        except Exception as e:
            print(f"[Error] 无法启动驱动: {e}")
            return False

    def get_battery_voltage(self):
        """
        获取当前电池电压
        原理: 运行一次 'ros2 topic echo' 获取数据并解析
        """
        # 注意：Wheeltec 的电压话题通常是 /voltage 或 /PowerVoltage，请根据实际情况修改
        topic_name = "/PowerVoltage" 
        
        try:
            # 使用 check_output 获取一次命令的返回值
            # --once 表示只接收一条消息就退出
            cmd = ["ros2", "topic", "echo", topic_name, "--once", "--field", "data"]
            
            # 设置 timeout，防止如果没有数据发布导致程序卡死
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            
            # 清理字符串（去掉换行符等）
            voltage = float(output.strip())
            return voltage
            
        except subprocess.TimeoutExpired:
            print("[Warning] 获取电压超时 (底盘没上电或话题错误)")
            return 0.0
        except Exception as e:
            print(f"[Error] 获取电压失败: {e}")
            return 0.0

    def start_keyboard_control(self):
        """
        开启键盘控制 (阻塞式，直到用户按 Ctrl+C)
        """
        print("\n" + "="*40)
        print("[App] 进入键盘控制模式...")
        cmd = ["ros2", "run", "wheeltec_robot_keyboard", "wheeltec_keyboard"]
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            pass
        print("[App] 键盘控制已结束")

    def shutdown(self):
        """
        关闭所有子进程，安全退出
        """
        print("[System] 正在关闭机器人系统...")
        
        if self.driver_process:
            # 发送 SIGINT (Ctrl+C) 信号给 ROS launch 进程
            self.driver_process.send_signal(signal.SIGINT)
            try:
                self.driver_process.wait(timeout=5) # 等待最长5秒
            except subprocess.TimeoutExpired:
                self.driver_process.kill() # 如果关不掉，强制杀进程
            print("[System] 底盘驱动已关闭。")
            self.driver_process = None

    # ========== 以下是新增的系统管理函数 ==========

    def emergency_stop(self):
        """
        紧急停止，立即发送全0速度指令
        """
        print("[System] 执行急停...")
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
            print("[System] 急停命令已发送")
        except:
            print("[Warning] 急停命令发送失败")

    def get_software_version(self):
        """
        获取软件版本号
        :return: str, 版本号字符串
        """
        # 尝试从参数服务器获取
        try:
            cmd = ["ros2", "param", "get", "/wheeltec_robot", "firmware_version"]
            output = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            if "String value is:" in output:
                version = output.split("String value is:")[-1].strip()
                return version
        except:
            pass
        
        # 返回 ROS2 版本
        try:
            cmd = ["ros2", "--version"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode("utf-8")
            return f"ROS2 {output.strip()}"
        except:
            return "Unknown"
        try:
            cmd = ["ros2", "--version"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode("utf-8")
            return f"ROS2 {output.strip()}"
        except:
            return "Unknown"

    # ==================== 底盘运动控制函数 ====================
    
    def set_velocity(self, v_x, v_y=0.0, w_z=0.0):
        """
        设置机器人运动速度
        :param v_x: float, X方向线速度 (m/s)
        :param v_y: float, Y方向线速度 (m/s)，仅麦轮有效
        :param w_z: float, Z轴角速度 (rad/s)
        """
        # 对于非麦轮车型，忽略 v_y
        if self.robot_type in ["akm", "diff"] and v_y != 0.0:
            v_y = 0.0
        
        twist_msg = (
            f"{{linear: {{x: {v_x}, y: {v_y}, z: 0.0}}, "
            f"angular: {{x: 0.0, y: 0.0, z: {w_z}}}}}"
        )
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            twist_msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except:
            pass

    def move_distance(self, distance, speed):
        """
        移动指定距离（适用于差速和阿克曼）
        :param distance: float, 移动距离（米），正数前进，负数后退
        :param speed: float, 移动速度（米/秒）
        :return: bool, 是否成功
        """
        if self.robot_type == "mec":
            print("[Warning] 麦轮车型请使用 move_distance_xy")
            return False
        
        # 获取起始位置
        start_pose = self._get_odom()
        if not start_pose:
            return False
        
        start_x, start_y = start_pose["x"], start_pose["y"]
        target_distance = abs(distance)
        v_x = speed if distance > 0 else -speed
        
        # 持续运动直到达到目标距离
        fail_count = 0
        max_fails = 5
        
        while True:
            current_pose = self._get_odom()
            if not current_pose:
                fail_count += 1
                if fail_count >= max_fails:
                    print("[Error] move_distance: 多次无法读取位置，停止运动")
                    self.set_velocity(0, 0, 0)
                    return False
                time.sleep(0.1)
                continue
            
            fail_count = 0  # 重置失败计数
            
            dx = current_pose["x"] - start_x
            dy = current_pose["y"] - start_y
            traveled = math.sqrt(dx**2 + dy**2)
            
            if traveled >= target_distance - 0.05:
                break
            
            self.set_velocity(v_x, 0.0, 0.0)
            time.sleep(0.1)
        
        self.set_velocity(0, 0, 0)
        return True

    def move_distance_xy(self, distance_x, distance_y, speed_x, speed_y):
        """
        麦轮车型：在X和Y方向移动指定距离
        :param distance_x: float, X方向移动距离（米）
        :param distance_y: float, Y方向移动距离（米）
        :param speed_x: float, X方向速度（米/秒）
        :param speed_y: float, Y方向速度（米/秒）
        :return: bool, 是否成功
        """
        if self.robot_type != "mec":
            print("[Error] 该函数仅适用于麦轮车型")
            return False
        
        # 获取起始位置
        start_pose = self._get_odom()
        if not start_pose:
            return False
        
        start_x, start_y = start_pose["x"], start_pose["y"]
        target_x = start_x + distance_x
        target_y = start_y + distance_y
        
        vx = speed_x if distance_x >= 0 else -speed_x
        vy = speed_y if distance_y >= 0 else -speed_y
        
        # 持续运动直到到达目标
        fail_count = 0
        max_fails = 5
        
        while True:
            current_pose = self._get_odom()
            if not current_pose:
                fail_count += 1
                if fail_count >= max_fails:
                    print("[Error] move_distance_xy: 多次无法读取位置，停止运动")
                    self.set_velocity(0, 0, 0)
                    return False
                time.sleep(0.1)
                continue
            
            fail_count = 0  # 重置失败计数
            
            error_x = target_x - current_pose["x"]
            error_y = target_y - current_pose["y"]
            distance_error = math.sqrt(error_x**2 + error_y**2)
            
            if distance_error < 0.05:
                break
            
            self.set_velocity(vx, vy, 0.0)
            time.sleep(0.1)
        
        self.set_velocity(0, 0, 0)
        return True

    def rotate_angle(self, angle_degrees, angular_speed):
        """
        原地旋转指定角度
        :param angle_degrees: float, 旋转角度（度），正数逆时针，负数顺时针
        :param angular_speed: float, 角速度（弧度/秒）
        :return: bool, 是否成功
        """
        # 获取起始角度
        start_pose = self._get_odom()
        if not start_pose:
            return False
        
        start_theta = start_pose["theta"]
        angle_radians = math.radians(angle_degrees)
        target_theta = self._normalize_angle(start_theta + angle_radians)
        
        w_z = angular_speed if angle_radians >= 0 else -angular_speed
        v_x = 0.05 if self.robot_type == "akm" else 0.0  # 阿克曼需要微小线速度
        
        # 持续旋转直到达到目标角度
        fail_count = 0
        max_fails = 5
        
        while True:
            current_pose = self._get_odom()
            if not current_pose:
                fail_count += 1
                if fail_count >= max_fails:
                    print("[Error] rotate_angle: 多次无法读取位置，停止运动")
                    self.set_velocity(0, 0, 0)
                    return False
                time.sleep(0.1)
                continue
            
            fail_count = 0  # 重置失败计数
            
            current_theta = current_pose["theta"]
            error = self._angle_difference(target_theta, current_theta)
            
            if abs(error) < math.radians(2):  # 2度容差
                break
            
            self.set_velocity(v_x, 0.0, w_z)
            time.sleep(0.1)
        
        self.set_velocity(0, 0, 0)
        return True

    def get_wheel_speeds(self):
        """
        获取四个车轮的速度
        :return: list, [左前, 右前, 左后, 右后] 速度（rad/s）
        """
        topic_name = "/wheel_speeds"
        
        try:
            cmd = ["ros2", "topic", "echo", topic_name, "--once"]
            output = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            import re
            matches = re.findall(r"[-+]?\d*\.?\d+", output)
            if len(matches) >= 4:
                return [float(m) for m in matches[:4]]
        except:
            pass
        
        return [0.0, 0.0, 0.0, 0.0]

    def get_imu_data(self):
        """
        获取IMU 6轴数据
        :return: dict, 包含加速度和角速度
        """
        topic_name = "/imu/data"
        
        try:
            cmd = ["ros2", "topic", "echo", topic_name, "--once"]
            output = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            import re
            accel_match = re.search(
                r"linear_acceleration:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
                output, re.DOTALL
            )
            gyro_match = re.search(
                r"angular_velocity:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
                output, re.DOTALL
            )
            
            if accel_match and gyro_match:
                return {
                    "accel_x": float(accel_match.group(1)),
                    "accel_y": float(accel_match.group(2)),
                    "accel_z": float(accel_match.group(3)),
                    "gyro_x": float(gyro_match.group(1)),
                    "gyro_y": float(gyro_match.group(2)),
                    "gyro_z": float(gyro_match.group(3))
                }
        except:
            pass
        
        return {"accel_x": 0, "accel_y": 0, "accel_z": 0, "gyro_x": 0, "gyro_y": 0, "gyro_z": 0}

    def get_robot_pose(self):
        """
        获取机器人当前位姿
        :return: dict, {"x": x坐标, "y": y坐标, "theta": 朝向角}
        """
        return self._get_odom()

    def set_wheel_speeds(self, fl, fr, rl, rr):
        """
        直接设置四个轮子的速度
        :param fl, fr, rl, rr: float, 各轮速度（rad/s）
        """
        msg = f"{{data: [{fl}, {fr}, {rl}, {rr}]}}"
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/motor_speeds",
            "std_msgs/msg/Float32MultiArray",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except:
            pass

    def set_ackermann_angle(self, steering_angle):
        """
        设置阿克曼转向角度（仅阿克曼车型）
        :param steering_angle: float, 转向角度（度）
        """
        if self.robot_type != "akm":
            print("[Warning] 该函数仅适用于阿克曼车型")
            return
        
        angle_rad = math.radians(steering_angle)
        msg = f"{{data: {angle_rad}}}"
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/steering_angle",
            "std_msgs/msg/Float32",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except:
            pass

    # ==================== 辅助函数 ====================
    
    def _get_odom(self):
        """获取里程计数据"""
        try:
            cmd = ["ros2", "topic", "echo", "/odom", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0, stderr=subprocess.PIPE).decode("utf-8")
            
            import re
            # 改进的正则表达式，支持科学计数法（如 1.0e-09）
            number_pattern = r"([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)"
            
            # 匹配位置信息
            pos_pattern = f"position:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}"
            pos_match = re.search(pos_pattern, output, re.DOTALL)
            
            # 匹配方向（四元数）
            quat_pattern = f"orientation:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}.*?z:\\s*{number_pattern}.*?w:\\s*{number_pattern}"
            quat_match = re.search(quat_pattern, output, re.DOTALL)
            
            if pos_match and quat_match:
                x = float(pos_match.group(1))
                y = float(pos_match.group(2))
                qx = float(quat_match.group(1))
                qy = float(quat_match.group(2))
                qz = float(quat_match.group(3))
                qw = float(quat_match.group(4))
                
                # 四元数转欧拉角（yaw）- 使用完整公式
                theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                
                return {"x": x, "y": y, "theta": theta}
        except subprocess.TimeoutExpired:
            print("[Error] _get_odom: 读取 /odom 话题超时")
        except subprocess.CalledProcessError:
            print("[Error] _get_odom: 无法读取 /odom 话题，请确保 ROS 底盘驱动已启动")
        except ValueError as e:
            print(f"[Error] _get_odom: 数据解析失败 - {e}")
        except Exception as e:
            print(f"[Error] _get_odom: {type(e).__name__}: {e}")
        
        return None

    def _normalize_angle(self, angle):
        """角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _angle_difference(self, target, current):
        """计算两个角度的最短差值"""
        diff = target - current
        return self._normalize_angle(diff)
    # ==================== 机械臂控制函数 ====================
    
    def arm_home(self):
        """
        机械臂复位到初始姿态
        :return: bool, 是否成功
        """
        return self.set_joint_angles(0.0, 0.0)

    def set_joint_angles(self, joint1, joint2):
        """
        设置机械臂关节角度
        :param joint1: float, 大臂角度（度）
        :param joint2: float, 小臂角度（度）
        :return: bool, 是否成功
        """
        # 转换为弧度
        joint1_rad = math.radians(joint1)
        joint2_rad = math.radians(joint2)
        
        msg = f"{{data: [{joint1_rad}, {joint2_rad}]}}"
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/arm_joint_command",
            "std_msgs/msg/Float64MultiArray",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
            return True
        except:
            return False

    def set_yaw_angle(self, angle):
        """
        设置云台角度
        :param angle: float, 云台角度（度）
        :return: bool, 是否成功
        """
        angle_rad = math.radians(angle)
        msg = f"{{data: {angle_rad}}}"
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/arm_yaw_command",
            "std_msgs/msg/Float64",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
            return True
        except:
            return False

    def set_arm_position(self, x, y):
        """
        设置机械臂末端位置（笛卡尔空间）
        :param x: float, X坐标（mm）
        :param y: float, Y坐标（mm）
        :return: bool, 是否成功
        """
        # 逆运动学求解
        solution = self._inverse_kinematics(x, y)
        if solution is None:
            print("[Error] 目标位置超出工作空间")
            return False
        
        joint1, joint2 = solution
        return self.set_joint_angles(joint1, joint2)

    def set_gripper(self, value):
        """
        控制夹爪开合
        :param value: int/float, 夹爪开度 0-10（0=闭合, 10=打开）
        :return: bool, 是否成功
        """
        value = max(0, min(10, value))
        msg = f"{{data: {value}}}"
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/gripper_command",
            "std_msgs/msg/Float32",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
            return True
        except:
            return False

    def get_arm_pose_xy(self):
        """
        获取机械臂末端位置
        :return: tuple, (x, y) 坐标（mm）
        """
        # 通过正运动学计算
        try:
            # 从关节状态话题获取
            cmd = ["ros2", "topic", "echo", "/joint_states", "--once"]
            output = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            import re
            pos_match = re.search(r"position:.*?\[(.*?)\]", output, re.DOTALL)
            if pos_match:
                positions_str = pos_match.group(1)
                positions = [float(x.strip()) for x in positions_str.split(",")]
                
                if len(positions) >= 2:
                    joint1_rad = positions[0]
                    joint2_rad = positions[1]
                    
                    # 正运动学计算
                    x = (self.arm_length_1 * math.cos(joint1_rad) + 
                         self.arm_length_2 * math.cos(joint1_rad + joint2_rad))
                    y = (self.arm_length_1 * math.sin(joint1_rad) + 
                         self.arm_length_2 * math.sin(joint1_rad + joint2_rad))
                    
                    return (x, y)
        except:
            pass
        
        return (0.0, 0.0)

    def get_joint_states(self):
        """
        获取关节角度
        :return: tuple, (joint1, joint2, yaw) 角度（度）
        """
        try:
            cmd = ["ros2", "topic", "echo", "/joint_states", "--once"]
            output = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            import re
            pos_match = re.search(r"position:.*?\[(.*?)\]", output, re.DOTALL)
            if pos_match:
                positions_str = pos_match.group(1)
                positions = [float(x.strip()) for x in positions_str.split(",")]
                
                if len(positions) >= 3:
                    return (math.degrees(positions[0]), 
                           math.degrees(positions[1]), 
                           math.degrees(positions[2]))
        except:
            pass
        
        return (0.0, 0.0, 0.0)

    def set_pwm(self, pin, duty_cycle):
        """
        设置PWM输出
        :param pin: int, 引脚编号
        :param duty_cycle: float, 占空比 0.0-1.0
        :return: bool, 是否成功
        """
        duty_cycle = max(0.0, min(1.0, duty_cycle))
        msg = f"{{data: [{pin}, {duty_cycle}]}}"
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/pwm_command",
            "std_msgs/msg/Float32MultiArray",
            msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
            return True
        except:
            return False

    def _inverse_kinematics(self, x, y):
        """逆运动学求解"""
        L1 = self.arm_length_1
        L2 = self.arm_length_2
        
        distance = math.sqrt(x**2 + y**2)
        
        # 检查工作空间
        if distance > (L1 + L2) or distance < abs(L1 - L2):
            return None
        
        # 余弦定理求解 joint2
        cos_joint2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        
        if abs(cos_joint2) > 1.0:
            return None
        
        joint2_rad = math.acos(cos_joint2)
        
        # 求解 joint1
        alpha = math.atan2(y, x)
        beta = math.atan2(L2 * math.sin(joint2_rad), L1 + L2 * math.cos(joint2_rad))
        joint1_rad = alpha - beta
        
        return (math.degrees(joint1_rad), math.degrees(joint2_rad))
    # ==================== 感知与功能函数 ====================
    
    def launch_lidar(self):
        """
        启动雷达驱动
        :return: bool, 是否成功
        """
        if self.lidar_process:
            print("[Warning] 雷达已在运行")
            return True
        
        print("[Sensor] 正在启动雷达...")
        cmd = ["ros2", "launch", "wheeltec_lidar_ros2", "wheeltec_lidar.launch.py"]
        
        try:
            self.lidar_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(3)
            
            if self.lidar_process.poll() is not None:
                print("[Error] 雷达启动失败")
                return False
            
            print("[Sensor] 雷达启动成功")
            return True
        except:
            return False

    def stop_lidar(self):
        """停止雷达驱动"""
        if self.lidar_process:
            self.lidar_process.send_signal(signal.SIGINT)
            try:
                self.lidar_process.wait(timeout=5)
            except:
                self.lidar_process.terminate()
            self.lidar_process = None
            print("[Sensor] 雷达已停止")

    def launch_camera(self):
        """
        启动相机驱动
        :return: bool, 是否成功
        """
        if self.camera_process:
            print("[Warning] 相机已在运行")
            return True
        
        print("[Sensor] 正在启动相机...")
        cmd = ["ros2", "launch", "usb_cam", "camera.launch.py"]
        
        try:
            self.camera_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(3)
            
            if self.camera_process.poll() is not None:
                print("[Error] 相机启动失败")
                return False
            
            print("[Sensor] 相机启动成功")
            return True
        except:
            return False

    def stop_camera(self):
        """停止相机驱动"""
        if self.camera_process:
            self.camera_process.send_signal(signal.SIGINT)
            try:
                self.camera_process.wait(timeout=5)
            except:
                self.camera_process.terminate()
            self.camera_process = None
            print("[Sensor] 相机已停止")

    def start_visual_follow(self, color):
        """
        启动视觉跟随
        :param color: str, 目标颜色 'red', 'blue', 'green', 'yellow'
        :return: bool, 是否成功
        """
        if self.application_process:
            print("[Warning] 已有应用在运行")
            return False
        
        if not self.camera_process:
            self.launch_camera()
        
        print(f"[App] 启动视觉跟随（颜色: {color}）...")
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_kcf", "visual_follow.launch.py",
            f"target_color:={color}"
        ]
        
        try:
            self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[Error] 视觉跟随启动失败")
                return False
            
            print("[App] 视觉跟随已启动")
            return True
        except:
            return False

    def start_line_tracking(self, color):
        """
        启动视觉巡线
        :param color: str, 线条颜色 'black', 'red', 'yellow', 'white'
        :return: bool, 是否成功
        """
        if self.application_process:
            print("[Warning] 已有应用在运行")
            return False
        
        if not self.camera_process:
            self.launch_camera()
        
        print(f"[App] 启动视觉巡线（颜色: {color}）...")
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_kcf", "line_tracking.launch.py",
            f"line_color:={color}"
        ]
        
        try:
            self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[Error] 视觉巡线启动失败")
                return False
            
            print("[App] 视觉巡线已启动")
            return True
        except:
            return False

    def start_lidar_follow(self, target_dist=0.8):
        """
        启动雷达跟随
        :param target_dist: float, 目标距离（米）
        :return: bool, 是否成功
        """
        if self.application_process:
            print("[Warning] 已有应用在运行")
            return False
        
        if not self.lidar_process:
            self.launch_lidar()
        
        print(f"[App] 启动雷达跟随（目标距离: {target_dist}m）...")
        cmd = [
            "ros2", "launch",
            "simple_follower_ros2", "lidar_follow.launch.py",
            f"target_distance:={target_dist}"
        ]
        
        try:
            self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[Error] 雷达跟随启动失败")
                return False
            
            print("[App] 雷达跟随已启动")
            return True
        except:
            return False

    def stop_application(self):
        """停止当前应用"""
        if self.application_process:
            self.application_process.send_signal(signal.SIGINT)
            try:
                self.application_process.wait(timeout=5)
            except:
                self.application_process.terminate()
            self.application_process = None
            
            # 发送停止命令
            self.set_velocity(0, 0, 0)
            print("[App] 应用已停止")

    def get_lidar_distance(self, angle_degrees=0):
        """
        获取雷达指定角度的距离
        :param angle_degrees: float, 角度（度）
        :return: float, 距离（米），失败返回 -1
        """
        if not self.lidar_process:
            return -1
        
        try:
            cmd = ["ros2", "topic", "echo", "/scan", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0, stderr=subprocess.DEVNULL).decode("utf-8")
            
            import re
            angle_min_match = re.search(r"angle_min:\s*([-\d.]+)", output)
            angle_inc_match = re.search(r"angle_increment:\s*([-\d.]+)", output)
            ranges_match = re.search(r"ranges:.*?\[(.*?)\]", output, re.DOTALL)
            
            if all([angle_min_match, angle_inc_match, ranges_match]):
                angle_min = float(angle_min_match.group(1))
                angle_increment = float(angle_inc_match.group(1))
                
                ranges_str = ranges_match.group(1)
                ranges = [float(x.strip()) for x in ranges_str.split(",") if x.strip()]
                
                target_angle = math.radians(angle_degrees)
                index = int((target_angle - angle_min) / angle_increment)
                
                if 0 <= index < len(ranges):
                    distance = ranges[index]
                    return distance if distance != float('inf') else -1
        except:
            pass
        
        return -1
    # ==================== 建图与导航函数 ====================
    
    def start_mapping(self, method="gmapping"):
        """
        启动SLAM建图
        :param method: str, 建图算法 "gmapping", "cartographer"
        :return: bool, 是否成功
        """
        if self.mapping_process:
            print("[Warning] 建图已在运行")
            return True
        
        print(f"[Nav] 启动{method}建图...")
        
        if method == "gmapping":
            cmd = ["ros2", "launch", "wheeltec_robot_slam", "gmapping.launch.py"]
        elif method == "cartographer":
            cmd = ["ros2", "launch", "wheeltec_robot_slam", "cartographer.launch.py"]
        else:
            print("[Error] 不支持的建图算法")
            return False
        
        try:
            self.mapping_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(5)
            
            if self.mapping_process.poll() is not None:
                print("[Error] 建图启动失败")
                return False
            
            print("[Nav] 建图已启动")
            return True
        except:
            return False

    def save_map(self, map_name="my_map"):
        """
        保存当前地图
        :param map_name: str, 地图名称
        :return: bool, 是否成功
        """
        if not self.mapping_process:
            print("[Warning] 建图未运行")
            return False
        
        print(f"[Nav] 保存地图: {map_name}...")
        
        # 创建地图目录
        maps_dir = "maps"
        if not os.path.exists(maps_dir):
            os.makedirs(maps_dir)
        
        map_path = os.path.join(maps_dir, map_name)
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
        
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10.0)
            
            if result.returncode == 0:
                print(f"[Nav] 地图已保存到: {map_path}.pgm/.yaml")
                return True
            else:
                print("[Error] 地图保存失败")
                return False
        except:
            print("[Error] 保存地图超时")
            return False

    def start_keyboard_control(self):
        """
        启动键盘控制（阻塞式）
        """
        print("\n[Nav] 进入键盘控制模式...")
        cmd = ["ros2", "run", "wheeltec_robot_keyboard", "wheeltec_keyboard"]
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            pass
        print("[Nav] 键盘控制已结束")

    def load_map_and_start_navigation(self, map_name="my_map"):
        """
        加载地图并启动导航
        :param map_name: str, 地图名称
        :return: bool, 是否成功
        """
        if self.navigation_process:
            print("[Warning] 导航已在运行")
            return True
        
        print(f"[Nav] 加载地图并启动导航: {map_name}...")
        
        # 检查地图文件
        map_path = os.path.join("maps", f"{map_name}.yaml")
        if not os.path.exists(map_path):
            print(f"[Error] 地图文件不存在: {map_path}")
            return False
        
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_nav2", "navigation.launch.py",
            f"map:={os.path.abspath(map_path)}"
        ]
        
        try:
            self.navigation_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            print("[Nav] 等待导航系统初始化（约15秒）...")
            time.sleep(15)
            
            if self.navigation_process.poll() is not None:
                print("[Error] 导航启动失败")
                return False
            
            print("[Nav] 导航系统已启动")
            return True
        except:
            return False

    def move_to_goal(self, x, y, theta=0.0):
        """
        导航到目标点
        :param x: float, X坐标（米）
        :param y: float, Y坐标（米）
        :param theta: float, 朝向角度（度）
        :return: bool, 是否成功
        """
        if not self.navigation_process:
            print("[Error] 导航系统未启动")
            return False
        
        print(f"[Nav] 导航到目标: ({x:.2f}, {y:.2f}, {theta:.1f}°)")
        
        # 角度转四元数
        theta_rad = math.radians(theta)
        qz = math.sin(theta_rad / 2.0)
        qw = math.cos(theta_rad / 2.0)
        
        pose_msg = (
            f"{{header: {{frame_id: 'map'}}, "
            f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}"
        )
        
        cmd = [
            "ros2", "topic", "pub", "--once",
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            pose_msg
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
            print("[Nav] 目标已发送")
            return True
        except:
            print("[Error] 发送目标失败")
            return False

    def cancel_navigation(self):
        """
        取消当前导航
        :return: bool, 是否成功
        """
        if not self.navigation_process:
            return False
        
        print("[Nav] 取消导航...")
        self.set_velocity(0, 0, 0)
        print("[Nav] 导航已取消")
        return True
