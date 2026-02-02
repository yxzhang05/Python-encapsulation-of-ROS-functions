# 文件名: robot_lib_motion.py
# 功能：ROS2 底盘运动控制功能的 Python 封装
# 作者：自动生成
# 日期：2026-02-02

import subprocess
import time
import math
import threading
import sys

class RobotMotion:
    """
    机器人底盘运动控制类
    
    该类封装了 ROS2 机器人底盘的运动控制功能，包括：
    - 基础速度控制（线速度、角速度）
    - 闭环运动控制（移动指定距离、旋转指定角度）
    - 传感器数据获取（里程计、IMU、车轮速度）
    - 底层电机控制（直接控制四轮速度、阿克曼转向角）
    
    支持三种车型：
    - akm (阿克曼转向)
    - diff (差速轮)
    - mec (麦克纳姆轮)
    """
    
    def __init__(self, robot_type="mec"):
        """
        构造函数：初始化运动控制类
        
        参数：
        - robot_type (str): 机器人车型 ("akm", "diff", "mec")
        
        成员变量：
        - robot_type: 机器人车型
        - current_pose: 当前位姿 {x, y, theta}
        - current_velocity: 当前速度 {vx, vy, wz}
        - wheel_speeds: 四轮速度 [fl, fr, rl, rr]
        - imu_data: IMU 数据
        - pose_lock: 位姿数据的线程锁
        """
        self.robot_type = robot_type
        
        # 当前状态
        self.current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.current_velocity = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # [左前, 右前, 左后, 右后]
        self.imu_data = {
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0
        }
        
        # 线程锁，用于保护共享数据
        self.pose_lock = threading.Lock()
        
        # 车轮半径和轴距（单位：米）
        # 注意：这些参数需要根据实际机器人调整
        self.wheel_radius = 0.05      # 车轮半径 5cm
        self.wheel_base = 0.4         # 前后轮距 40cm
        self.wheel_track = 0.4        # 左右轮距 40cm
    
    def set_velocity(self, v_x, v_y=0.0, w_z=0.0):
        """
        设置机器人运动速度
        
        功能说明：
        这是最底层的运动控制接口，直接发布速度命令到 /cmd_vel 话题。
        
        封装原理：
        1. 构建 geometry_msgs/Twist 消息
        2. 使用 ros2 topic pub 命令发布到 /cmd_vel 话题
        3. 使用 --rate 参数持续发布，保持速度
        
        参数：
        - v_x (float): X 方向线速度，单位 m/s（前进为正，后退为负）
        - v_y (float): Y 方向线速度，单位 m/s（仅麦轮有效，左移为正，右移为负）
        - w_z (float): Z 轴角速度，单位 rad/s（逆时针为正，顺时针为负）
        
        车型兼容性：
        - 阿克曼 (akm): 只支持 v_x 和 w_z，v_y 自动忽略
        - 差速 (diff): 只支持 v_x 和 w_z，v_y 自动忽略
        - 麦轮 (mec): 支持全部三个参数，可以全向移动
        
        使用示例：
        >>> motion = RobotMotion("mec")
        >>> motion.set_velocity(0.2, 0.0, 0.0)  # 以 0.2m/s 前进
        >>> motion.set_velocity(0.0, 0.1, 0.0)  # 以 0.1m/s 左移（仅麦轮）
        >>> motion.set_velocity(0.0, 0.0, 0.5)  # 以 0.5rad/s 原地旋转
        """
        # 对于非麦轮车型，自动忽略 v_y
        if self.robot_type in ["akm", "diff"] and v_y != 0.0:
            print(f"[警告] {self.robot_type} 车型不支持横向移动，v_y 已被忽略")
            v_y = 0.0
        
        # 构建 Twist 消息
        # linear: 线速度 (x, y, z)
        # angular: 角速度 (x, y, z)
        twist_msg = (
            f"{{linear: {{x: {v_x}, y: {v_y}, z: 0.0}}, "
            f"angular: {{x: 0.0, y: 0.0, z: {w_z}}}}}"
        )
        
        # 构建 ROS2 命令
        cmd = [
            "ros2", "topic", "pub",
            "--once",                       # 只发布一次
            "/cmd_vel",                     # 速度话题
            "geometry_msgs/msg/Twist",      # 消息类型
            twist_msg                       # 消息内容
        ]
        
        try:
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            # 更新当前速度状态
            self.current_velocity = {"vx": v_x, "vy": v_y, "wz": w_z}
            
        except subprocess.TimeoutExpired:
            print("[警告] 发送速度命令超时")
        except Exception as e:
            print(f"[错误] 发送速度命令失败: {e}")
    
    def stop(self):
        """
        停止机器人运动
        
        功能说明：
        发送全 0 速度命令，让机器人停止。
        这是 set_velocity(0, 0, 0) 的快捷方式。
        
        使用示例：
        >>> motion = RobotMotion()
        >>> motion.set_velocity(0.5, 0, 0)  # 前进
        >>> time.sleep(2)
        >>> motion.stop()  # 停止
        """
        self.set_velocity(0.0, 0.0, 0.0)
    
    def move_distance_mecanum(self, distance_x, distance_y, speed_x, speed_y):
        """
        麦轮车型：移动指定距离
        
        功能说明：
        控制麦轮机器人在 X 和 Y 方向上移动指定距离。
        
        封装原理：
        1. 记录起始位姿（从里程计获取）
        2. 根据目标距离和速度计算运动时间
        3. 持续发布速度命令
        4. 实时检查当前位姿，判断是否到达目标
        5. 到达后停止运动
        
        参数：
        - distance_x (float): X 方向移动距离，单位米（正为前进，负为后退）
        - distance_y (float): Y 方向移动距离，单位米（正为左移，负为右移）
        - speed_x (float): X 方向速度，单位 m/s（必须为正数）
        - speed_y (float): Y 方向速度，单位 m/s（必须为正数）
        
        返回值：
        - bool: 成功到达返回 True，失败返回 False
        
        使用示例：
        >>> motion = RobotMotion("mec")
        >>> motion.move_distance_mecanum(1.0, 0.5, 0.2, 0.2)  # 前进1米，左移0.5米
        """
        if self.robot_type != "mec":
            print(f"[错误] 该函数仅支持麦轮车型，当前车型: {self.robot_type}")
            return False
        
        print(f"[运动] 麦轮移动: X={distance_x}m, Y={distance_y}m")
        
        # 获取起始位姿
        start_pose = self._get_current_odom()
        if start_pose is None:
            print("[错误] 无法获取起始位姿")
            return False
        
        start_x, start_y = start_pose["x"], start_pose["y"]
        
        # 计算目标位姿
        target_x = start_x + distance_x
        target_y = start_y + distance_y
        
        # 确定速度方向
        vx = speed_x if distance_x >= 0 else -speed_x
        vy = speed_y if distance_y >= 0 else -speed_y
        
        # 设置容差（米）
        tolerance = 0.05
        
        # 开始运动
        try:
            while True:
                # 获取当前位姿
                current = self._get_current_odom()
                if current is None:
                    print("[警告] 无法获取当前位姿")
                    time.sleep(0.1)
                    continue
                
                # 计算距离误差
                error_x = target_x - current["x"]
                error_y = target_y - current["y"]
                distance_error = math.sqrt(error_x**2 + error_y**2)
                
                # 判断是否到达
                if distance_error < tolerance:
                    print(f"[运动] 到达目标位置")
                    break
                
                # 发送速度命令
                self.set_velocity(vx, vy, 0.0)
                time.sleep(0.1)
            
            # 停止运动
            self.stop()
            return True
            
        except KeyboardInterrupt:
            print("\n[运动] 用户中断")
            self.stop()
            return False
    
    def move_distance_ackermann_diff(self, distance, speed, turn_radius=None):
        """
        阿克曼/差速车型：移动指定距离
        
        功能说明：
        控制阿克曼或差速机器人直线或曲线移动指定距离。
        
        封装原理：
        1. 如果 turn_radius 为 None，执行直线运动
        2. 如果指定了 turn_radius，执行曲线运动（计算角速度）
        3. 通过里程计反馈判断是否到达目标距离
        
        参数：
        - distance (float): 移动距离，单位米（正为前进，负为后退）
        - speed (float): 线速度，单位 m/s（必须为正数）
        - turn_radius (float): 转弯半径，单位米（None 表示直线运动）
        
        返回值：
        - bool: 成功到达返回 True，失败返回 False
        
        使用示例：
        >>> motion = RobotMotion("diff")
        >>> motion.move_distance_ackermann_diff(2.0, 0.3)  # 直线前进 2 米
        >>> motion.move_distance_ackermann_diff(3.14, 0.2, 1.0)  # 半径 1 米转弯
        """
        if self.robot_type not in ["akm", "diff"]:
            print(f"[错误] 该函数不支持 {self.robot_type} 车型")
            return False
        
        print(f"[运动] 移动距离: {distance}m, 速度: {speed}m/s")
        if turn_radius:
            print(f"[运动] 转弯半径: {turn_radius}m")
        
        # 获取起始位姿
        start_pose = self._get_current_odom()
        if start_pose is None:
            print("[错误] 无法获取起始位姿")
            return False
        
        start_x, start_y = start_pose["x"], start_pose["y"]
        
        # 计算角速度（如果是曲线运动）
        if turn_radius and turn_radius != 0:
            w_z = speed / turn_radius  # v = r * ω
        else:
            w_z = 0.0
        
        # 确定线速度方向
        v_x = speed if distance >= 0 else -speed
        
        # 设置容差
        tolerance = 0.05
        target_distance = abs(distance)
        
        # 开始运动
        try:
            while True:
                # 获取当前位姿
                current = self._get_current_odom()
                if current is None:
                    time.sleep(0.1)
                    continue
                
                # 计算已行驶距离
                dx = current["x"] - start_x
                dy = current["y"] - start_y
                traveled = math.sqrt(dx**2 + dy**2)
                
                # 判断是否到达
                if traveled >= target_distance - tolerance:
                    print(f"[运动] 已行驶: {traveled:.2f}m")
                    break
                
                # 发送速度命令
                self.set_velocity(v_x, 0.0, w_z)
                time.sleep(0.1)
            
            # 停止运动
            self.stop()
            return True
            
        except KeyboardInterrupt:
            print("\n[运动] 用户中断")
            self.stop()
            return False
    
    def rotate_angle(self, angle_degrees, angular_speed=0.5):
        """
        原地旋转指定角度
        
        功能说明：
        控制机器人原地旋转（差速/麦轮）或小半径转向（阿克曼）。
        
        封装原理：
        1. 从里程计获取当前朝向角
        2. 计算目标角度
        3. 持续发送角速度命令
        4. 实时检查当前角度，判断是否到达
        5. 考虑角度的周期性（-π 到 π）
        
        参数：
        - angle_degrees (float): 旋转角度，单位度（正为逆时针，负为顺时针）
        - angular_speed (float): 角速度，单位 rad/s（默认 0.5）
        
        返回值：
        - bool: 成功到达返回 True，失败返回 False
        
        使用示例：
        >>> motion = RobotMotion("diff")
        >>> motion.rotate_angle(90)    # 逆时针旋转 90 度
        >>> motion.rotate_angle(-180)  # 顺时针旋转 180 度
        """
        print(f"[运动] 旋转角度: {angle_degrees}°")
        
        # 将角度转换为弧度
        angle_radians = math.radians(angle_degrees)
        
        # 获取起始角度
        start_pose = self._get_current_odom()
        if start_pose is None:
            print("[错误] 无法获取起始位姿")
            return False
        
        start_theta = start_pose["theta"]
        
        # 计算目标角度
        target_theta = self._normalize_angle(start_theta + angle_radians)
        
        # 确定旋转方向
        w_z = angular_speed if angle_radians >= 0 else -angular_speed
        
        # 如果是阿克曼车型，需要添加小线速度
        v_x = 0.05 if self.robot_type == "akm" else 0.0
        
        # 设置容差（弧度）
        tolerance = math.radians(2)  # 2 度
        
        # 开始旋转
        try:
            while True:
                # 获取当前角度
                current = self._get_current_odom()
                if current is None:
                    time.sleep(0.1)
                    continue
                
                current_theta = current["theta"]
                
                # 计算角度误差
                error = self._angle_difference(target_theta, current_theta)
                
                # 判断是否到达
                if abs(error) < tolerance:
                    print(f"[运动] 到达目标角度")
                    break
                
                # 如果接近目标，降低速度
                if abs(error) < math.radians(10):
                    w_z = w_z * 0.3
                
                # 发送速度命令
                self.set_velocity(v_x, 0.0, w_z)
                time.sleep(0.1)
            
            # 停止运动
            self.stop()
            return True
            
        except KeyboardInterrupt:
            print("\n[运动] 用户中断")
            self.stop()
            return False
    
    def get_wheel_speeds(self):
        """
        获取四个车轮的实时速度
        
        功能说明：
        从底盘驱动节点获取四个车轮的实时转速。
        
        封装原理：
        1. 订阅车轮速度话题（通常是 /wheel_speeds 或自定义消息）
        2. 解析消息获取四个轮子的转速（rad/s 或 m/s）
        3. 返回列表 [左前, 右前, 左后, 右后]
        
        注意：
        Wheeltec 部分底盘可能没有单独的车轮速度话题，
        此时需要从电机编码器话题或速度控制反馈中获取。
        
        返回值：
        - list: [左前, 右前, 左后, 右后] 速度值，单位 rad/s
        
        使用示例：
        >>> motion = RobotMotion()
        >>> speeds = motion.get_wheel_speeds()
        >>> print(f"车轮速度: {speeds}")
        """
        # Wheeltec 的车轮速度话题（需要根据实际情况调整）
        topic_name = "/wheel_speeds"
        
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
            
            # 解析输出（这里需要根据实际消息格式调整）
            # 假设消息格式为: data: [fl, fr, rl, rr]
            import re
            matches = re.findall(r"[-+]?\d*\.?\d+", output)
            if len(matches) >= 4:
                speeds = [float(m) for m in matches[:4]]
                self.wheel_speeds = speeds
                return speeds
            else:
                print("[警告] 车轮速度数据格式异常")
                return self.wheel_speeds
                
        except subprocess.TimeoutExpired:
            print(f"[警告] 获取车轮速度超时")
            return self.wheel_speeds
        except Exception as e:
            print(f"[错误] 获取车轮速度失败: {e}")
            return self.wheel_speeds
    
    def get_imu_data(self):
        """
        获取陀螺仪 6 轴数据
        
        功能说明：
        从 IMU 传感器获取 3 轴加速度和 3 轴角速度。
        
        封装原理：
        1. 订阅 IMU 话题 /imu/data
        2. 消息类型为 sensor_msgs/msg/Imu
        3. 提取加速度（linear_acceleration）和角速度（angular_velocity）
        
        返回值：
        - dict: 包含 6 个字段的字典
            {
                "accel_x": x轴加速度 (m/s²),
                "accel_y": y轴加速度 (m/s²),
                "accel_z": z轴加速度 (m/s²),
                "gyro_x": x轴角速度 (rad/s),
                "gyro_y": y轴角速度 (rad/s),
                "gyro_z": z轴角速度 (rad/s)
            }
        
        使用示例：
        >>> motion = RobotMotion()
        >>> imu = motion.get_imu_data()
        >>> print(f"Z轴角速度: {imu['gyro_z']:.3f} rad/s")
        """
        topic_name = "/imu/data"
        
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
            
            # 解析 IMU 数据
            import re
            
            # 提取加速度
            accel_match = re.search(
                r"linear_acceleration:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
                output, re.DOTALL
            )
            
            # 提取角速度
            gyro_match = re.search(
                r"angular_velocity:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
                output, re.DOTALL
            )
            
            if accel_match and gyro_match:
                self.imu_data = {
                    "accel_x": float(accel_match.group(1)),
                    "accel_y": float(accel_match.group(2)),
                    "accel_z": float(accel_match.group(3)),
                    "gyro_x": float(gyro_match.group(1)),
                    "gyro_y": float(gyro_match.group(2)),
                    "gyro_z": float(gyro_match.group(3))
                }
                return self.imu_data
            else:
                print("[警告] IMU 数据格式异常")
                return self.imu_data
                
        except subprocess.TimeoutExpired:
            print("[警告] 获取 IMU 数据超时")
            return self.imu_data
        except Exception as e:
            print(f"[错误] 获取 IMU 数据失败: {e}")
            return self.imu_data
    
    def get_robot_pose(self):
        """
        获取机器人当前位姿
        
        功能说明：
        从里程计获取机器人在世界坐标系中的位置和朝向。
        
        封装原理：
        1. 订阅里程计话题 /odom
        2. 提取位置 (x, y) 和朝向角 theta
        3. 四元数转欧拉角获取 yaw 角度
        
        返回值：
        - dict: {"x": x坐标(m), "y": y坐标(m), "theta": 朝向角(rad)}
        
        使用示例：
        >>> motion = RobotMotion()
        >>> pose = motion.get_robot_pose()
        >>> print(f"位置: ({pose['x']:.2f}, {pose['y']:.2f})")
        >>> print(f"朝向: {math.degrees(pose['theta']):.1f}°")
        """
        return self._get_current_odom()
    
    def set_wheel_speeds(self, fl, fr, rl, rr):
        """
        直接设置四个轮子的速度
        
        功能说明：
        绕过运动学解算，直接控制四个电机的转速。
        
        封装原理：
        1. 发布到底层电机控制话题（通常是 /motor_speeds）
        2. 消息包含四个电机的目标转速
        3. 单位通常是 rad/s 或 RPM
        
        注意：
        该功能需要底盘驱动支持直接电机控制接口。
        Wheeltec 部分底盘可能需要修改串口协议才能支持。
        
        参数：
        - fl (float): 左前轮速度，单位 rad/s
        - fr (float): 右前轮速度，单位 rad/s
        - rl (float): 左后轮速度，单位 rad/s
        - rr (float): 右后轮速度，单位 rad/s
        
        使用示例：
        >>> motion = RobotMotion()
        >>> motion.set_wheel_speeds(50, 50, 50, 50)  # 四轮同速前进
        """
        print(f"[控制] 设置车轮速度: FL={fl}, FR={fr}, RL={rl}, RR={rr}")
        
        # 构建消息（需要根据实际话题格式调整）
        # 这里假设使用自定义消息类型
        msg = f"{{data: [{fl}, {fr}, {rl}, {rr}]}}"
        
        topic_name = "/motor_speeds"
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
            
            self.wheel_speeds = [fl, fr, rl, rr]
            
        except Exception as e:
            print(f"[错误] 设置车轮速度失败: {e}")
            print("[提示] 该功能需要底盘驱动支持，可能需要修改串口协议")
    
    def set_ackermann_angle(self, steering_angle):
        """
        设置阿克曼转向角度
        
        功能说明：
        对于阿克曼转向车型，直接设置前轮转向角度。
        
        封装原理：
        1. 发布到转向角度话题（如 /steering_angle）
        2. 底层驱动将角度转换为舵机 PWM 信号
        
        注意：
        该功能仅适用于阿克曼转向车型。
        
        参数：
        - steering_angle (float): 转向角度，单位度（正为左转，负为右转）
        
        使用示例：
        >>> motion = RobotMotion("akm")
        >>> motion.set_ackermann_angle(30)  # 左转 30 度
        """
        if self.robot_type != "akm":
            print(f"[警告] 该函数仅适用于阿克曼车型，当前: {self.robot_type}")
            return
        
        print(f"[控制] 设置转向角度: {steering_angle}°")
        
        # 转换为弧度
        angle_rad = math.radians(steering_angle)
        
        # 构建消息
        msg = f"{{data: {angle_rad}}}"
        
        topic_name = "/steering_angle"
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
            
        except Exception as e:
            print(f"[错误] 设置转向角度失败: {e}")
            print("[提示] 该功能需要底盘驱动支持，可能需要修改串口协议")
    
    # ==================== 辅助函数 ====================
    
    def _get_current_odom(self):
        """
        内部函数：从里程计获取当前位姿
        
        返回值：
        - dict: {"x": x坐标, "y": y坐标, "theta": 朝向角} 或 None
        """
        topic_name = "/odom"
        
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
            
            # 提取位置
            import re
            pos_match = re.search(
                r"position:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+)",
                output, re.DOTALL
            )
            
            # 提取四元数
            quat_match = re.search(
                r"orientation:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+).*?w:\s*([-\d.]+)",
                output, re.DOTALL
            )
            
            if pos_match and quat_match:
                x = float(pos_match.group(1))
                y = float(pos_match.group(2))
                
                # 四元数转欧拉角
                qx = float(quat_match.group(1))
                qy = float(quat_match.group(2))
                qz = float(quat_match.group(3))
                qw = float(quat_match.group(4))
                
                # 计算 yaw 角
                theta = math.atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz))
                
                return {"x": x, "y": y, "theta": theta}
            else:
                return None
                
        except:
            return None
    
    def _normalize_angle(self, angle):
        """
        内部函数：将角度归一化到 [-π, π] 范围
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _angle_difference(self, target, current):
        """
        内部函数：计算两个角度之间的最短差值
        """
        diff = target - current
        return self._normalize_angle(diff)


# 模块测试代码
if __name__ == "__main__":
    print("=" * 60)
    print("ROS2 底盘运动控制功能测试")
    print("=" * 60)
    
    motion = RobotMotion("mec")
    
    try:
        print("\n[测试] 基础速度控制...")
        motion.set_velocity(0.2, 0.0, 0.0)
        print("前进 0.2m/s，持续 2 秒...")
        time.sleep(2)
        motion.stop()
        
        print("\n[测试] 获取位姿...")
        pose = motion.get_robot_pose()
        if pose:
            print(f"位置: ({pose['x']:.2f}, {pose['y']:.2f})")
            print(f"朝向: {math.degrees(pose['theta']):.1f}°")
        
        print("\n[测试] 获取 IMU 数据...")
        imu = motion.get_imu_data()
        print(f"角速度 Z: {imu['gyro_z']:.3f} rad/s")
        
        print("\n测试完成！")
        
    except KeyboardInterrupt:
        print("\n\n测试被中断")
        motion.stop()
