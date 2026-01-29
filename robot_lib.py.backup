# 文件名: robot_lib.py
"""
ROS2 Python封装库
提供简化的Python接口来控制ROS2机器人，无需用户学习ROS命令

注意: 某些获取数据的函数（如 _get_odom, get_wheel_speeds, get_imu_data 等）
      返回占位符数据，需要根据实际的ROS2消息格式进行解析实现。
      这些函数已标记为需要完善的部分。
"""

import subprocess
import time
import signal
import math
import os


class Robot:
    def __init__(self):
        """初始化机器人对象"""
        self.driver_process = None  # 底盘驱动进程句柄
        self.keyboard_process = None  # 键盘进程句柄
        self.lidar_process = None  # 雷达进程句柄
        self.camera_process = None  # 相机进程句柄
        self.mapping_process = None  # 建图进程句柄
        self.navigation_process = None  # 导航进程句柄
        self.visual_follow_process = None  # 视觉跟随进程句柄
        self.line_tracking_process = None  # 巡线进程句柄
        self.lidar_follow_process = None  # 雷达跟随进程句柄
        
        # 定义车型映射关系
        self.ROBOT_TYPE_MAP = {
            "akm": "ackermann",    # 阿克曼
            "diff": "diff",        # 差速
            "mec": "mecanum"       # 麦轮
        }
        
        self.robot_type = None  # 当前车型
        
    # ==================== 系统管理 ====================
    
    def initialize(self, robot_type):
        """
        初始化机器人底盘
        :param robot_type: str, 车型枚举 "akm", "diff", "mec"
        :return: bool, 成功返回True
        """
        # 1. 检查参数合法性，确保传入的车型在支持列表中
        if robot_type not in self.ROBOT_TYPE_MAP:
            print(f"[Error] 未知的车型: {robot_type}. 支持: {list(self.ROBOT_TYPE_MAP.keys())}")
            return False

        # 2. 保存当前车型，并获取实际的ROS参数名称
        self.robot_type = robot_type
        real_type_name = self.ROBOT_TYPE_MAP[robot_type]
        print(f"[System] 正在启动 {real_type_name} ({robot_type}) 底盘驱动...")

        # 3. 构建ROS2 launch命令
        # 使用turn_on_wheeltec_robot包启动底盘驱动
        # robot_type参数用于指定具体的车型配置
        cmd = [
            "ros2", "launch", 
            "turn_on_wheeltec_robot", 
            "turn_on_wheeltec_robot.launch.py",
            f"robot_type:={real_type_name}" 
        ]

        try:
            # 4. 启动后台进程
            # stdout=DEVNULL: 屏蔽标准输出，保持界面清爽
            # stderr=PIPE: 保留错误输出以便调试
            self.driver_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 5. 等待硬件初始化
            # 底盘需要时间连接串口、初始化IMU、雷达等硬件
            print("[System] 正在初始化硬件，请等待 5 秒...")
            time.sleep(5)
            
            # 6. 检查进程是否因报错而退出
            # poll()返回None表示进程仍在运行，返回退出码表示已结束
            if self.driver_process.poll() is not None:
                print("[Error] 驱动启动失败！请检查错误日志。")
                return False
                
            print(f"[System] {real_type_name} 底盘初始化完成！")
            return True

        except Exception as e:
            print(f"[Error] 无法启动驱动: {e}")
            return False

    def shutdown(self):
        """
        安全关闭机器人系统
        停止所有电机运动，关闭雷达/相机流，kill掉后台的ROS子进程
        """
        print("[System] 正在关闭机器人系统...")
        
        # 1. 首先停止所有运动，确保机器人处于静止状态
        self.emergency_stop()
        
        # 2. 准备关闭所有后台进程
        # 进程列表包含进程句柄和名称，方便日志输出
        processes = [
            (self.driver_process, "底盘驱动"),
            (self.keyboard_process, "键盘控制"),
            (self.lidar_process, "雷达"),
            (self.camera_process, "相机"),
            (self.mapping_process, "建图"),
            (self.navigation_process, "导航"),
            (self.visual_follow_process, "视觉跟随"),
            (self.line_tracking_process, "巡线"),
            (self.lidar_follow_process, "雷达跟随")
        ]
        
        # 3. 逐个关闭进程
        for process, name in processes:
            if process:
                try:
                    # 先发送SIGINT信号（Ctrl+C），让进程优雅退出
                    process.send_signal(signal.SIGINT)
                    # 等待最多5秒让进程自行结束
                    process.wait(timeout=5)
                    print(f"[System] {name}已关闭。")
                except subprocess.TimeoutExpired:
                    # 如果5秒后进程还没结束，强制kill
                    process.kill()
                    print(f"[System] {name}强制关闭。")
                except Exception as e:
                    print(f"[Warning] 关闭{name}时出错: {e}")
        
        # 4. 重置所有进程句柄为None，释放资源
        self.driver_process = None
        self.keyboard_process = None
        self.lidar_process = None
        self.camera_process = None
        self.mapping_process = None
        self.navigation_process = None
        self.visual_follow_process = None
        self.line_tracking_process = None
        self.lidar_follow_process = None
        
        print("[System] 机器人系统已完全关闭。")

    def get_battery_voltage(self):
        """
        获取当前底盘电池电压
        :return: float, 电压值(V)，失败返回0.0
        """
        # 指定电压话题名称，Wheeltec底盘通常使用/PowerVoltage话题
        topic_name = "/PowerVoltage" 
        
        try:
            # 构建ros2 topic echo命令
            # --once: 只获取一次消息就退出
            # --field data: 只提取消息中的data字段，简化输出
            cmd = ["ros2", "topic", "echo", topic_name, "--once", "--field", "data"]
            
            # 执行命令，设置2秒超时防止话题无数据时卡死
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            
            # 清理输出字符串（去除换行符等），转换为浮点数
            voltage = float(output.strip())
            return voltage
            
        except subprocess.TimeoutExpired:
            # 超时通常意味着底盘没有上电或话题名称错误
            print("[Warning] 获取电压超时 (底盘没上电或话题错误)")
            return 0.0
        except Exception as e:
            print(f"[Error] 获取电压失败: {e}")
            return 0.0

    def emergency_stop(self):
        """
        软件急停
        立即发送全0速度指令，停止所有运动
        """
        print("[System] 紧急停止！")
        self.set_velocity(0.0, 0.0, 0.0)

    def get_software_version(self):
        """
        获取下位机软件或固件的版本号
        :return: str, 版本号字符串
        """
        try:
            cmd = ["ros2", "topic", "echo", "/version", "--once", "--field", "data"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            return output.strip()
        except Exception as e:
            print(f"[Warning] 获取版本号失败: {e}")
            return "Unknown"
    
    # ==================== 底盘运动控制 ====================
    
    def set_velocity(self, v_x, v_y, w_z):
        """
        最底层的速度控制接口
        :param v_x: float, 线速度 (m/s), 前进为正
        :param v_y: float, 横向速度 (m/s), 仅麦轮有效
        :param w_z: float, 角速度 (rad/s), 左转为正
        """
        # 构建ROS2消息发布命令
        # 使用geometry_msgs/msg/Twist消息类型，这是ROS中标准的速度控制消息
        # linear.x: 前后方向线速度
        # linear.y: 左右方向线速度（仅全向轮/麦轮有效）
        # angular.z: 绕Z轴旋转的角速度
        cmd = [
            "ros2", "topic", "pub", "--once", "/cmd_vel",
            "geometry_msgs/msg/Twist",
            f"{{linear: {{x: {v_x}, y: {v_y}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {w_z}}}}}"
        ]
        
        try:
            # 执行命令，屏蔽输出以保持界面清爽
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置速度失败: {e}")

    def move_distance(self, distance, speed, lateral_distance=0.0, lateral_speed=0.0, timeout=30.0):
        """
        闭环控制移动指定距离
        阿克曼/差速: 仅使用distance和speed参数
        麦轮: 可使用所有参数实现全向移动
        
        :param distance: float, 前进距离(m), 负数后退
        :param speed: float, 前进速度(m/s), 必须为正
        :param lateral_distance: float, 横向距离(m), 仅麦轮有效
        :param lateral_speed: float, 横向速度(m/s), 仅麦轮有效
        :param timeout: float, 超时时间(秒), 默认30秒
        :return: bool, 成功返回True
        """
        # 1. 参数检查：速度必须为正数
        if speed <= 0:
            print("[Error] 速度必须为正数")
            return False
        
        print(f"[Motion] 移动距离: 前进={distance}m, 横向={lateral_distance}m")
        
        try:
            # 2. 获取初始位置
            # 通过里程计读取当前位置，作为移动的起点
            initial_odom = self._get_odom()
            if initial_odom is None:
                print("[Error] 无法获取里程计信息")
                return False
            
            # 3. 计算目标位置
            # 在当前位置基础上加上要移动的距离
            initial_x, initial_y = initial_odom['x'], initial_odom['y']
            target_x = initial_x + distance
            target_y = initial_y + lateral_distance
            
            # 4. 确定运动方向
            # 根据距离的正负确定速度方向：正距离用正速度，负距离用负速度
            v_x = speed if distance >= 0 else -speed
            v_y = lateral_speed if lateral_distance >= 0 else -lateral_speed
            
            # 5. 闭环控制：持续运动直到到达目标或超时
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                # 获取当前位置
                current_odom = self._get_odom()
                if current_odom is None:
                    break
                
                current_x, current_y = current_odom['x'], current_odom['y']
                
                # 计算与目标位置的剩余距离
                remaining_x = abs(target_x - current_x)
                remaining_y = abs(target_y - current_y)
                
                # 判断是否到达目标（允许2cm的误差）
                if remaining_x < 0.02 and remaining_y < 0.02:  # 2cm误差
                    break
                
                # 持续发送速度命令
                self.set_velocity(v_x, v_y, 0.0)
                time.sleep(0.1)  # 100ms的控制周期
            
            # 6. 停止运动
            self.set_velocity(0.0, 0.0, 0.0)
            
            # 7. 检查是否超时
            if (time.time() - start_time) >= timeout:
                print("[Warning] 移动超时")
                return False
            
            print("[Motion] 移动完成")
            return True
            
        except Exception as e:
            # 发生异常时确保停止运动
            print(f"[Error] 移动过程出错: {e}")
            self.set_velocity(0.0, 0.0, 0.0)
            return False

    def move_distance_with_radius(self, distance, speed, radius=None):
        """
        阿克曼/差速专用: 沿指定半径弧线移动
        :param distance: float, 移动距离(m)
        :param speed: float, 线速度(m/s)
        :param radius: float, 转弯半径(m), None表示直线
        :return: bool, 成功返回True
        """
        # 1. 麦轮车型不适用此函数，建议使用move_distance
        if self.robot_type == "mec":
            print("[Warning] 麦轮车型请使用 move_distance 函数")
            return self.move_distance(distance, speed)
        
        # 2. 如果半径为None，执行直线运动
        if radius is None:
            return self.move_distance(distance, speed)
        
        # 3. 计算角速度
        # 弧线运动公式: w = v / r
        # 其中 w 是角速度，v 是线速度，r 是转弯半径
        w_z = speed / radius if radius != 0 else 0.0
        
        print(f"[Motion] 弧线移动: 距离={distance}m, 半径={radius}m")
        
        try:
            # 4. 获取初始位置
            initial_odom = self._get_odom()
            if initial_odom is None:
                return False
            
            # 5. 累计已移动的距离
            # 通过连续采样位置，计算实际行驶距离
            distance_traveled = 0.0
            last_x, last_y = initial_odom['x'], initial_odom['y']
            
            # 6. 确定线速度方向
            v_x = speed if distance >= 0 else -speed
            
            # 7. 持续运动直到达到目标距离
            while distance_traveled < abs(distance):
                # 发送速度命令：包含线速度和角速度
                self.set_velocity(v_x, 0.0, w_z)
                time.sleep(0.1)
                
                # 获取当前位置
                current_odom = self._get_odom()
                if current_odom is None:
                    break
                
                # 计算自上次采样以来移动的距离（直线距离）
                current_x, current_y = current_odom['x'], current_odom['y']
                distance_traveled += math.sqrt(
                    (current_x - last_x)**2 + (current_y - last_y)**2
                )
                # 更新上次位置
                last_x, last_y = current_x, current_y
            
            # 8. 停止运动
            self.set_velocity(0.0, 0.0, 0.0)
            print("[Motion] 弧线移动完成")
            return True
            
        except Exception as e:
            print(f"[Error] 弧线移动出错: {e}")
            self.set_velocity(0.0, 0.0, 0.0)
            return False

    def rotate_angle(self, angle, speed, timeout=30.0):
        """
        原地或行进间旋转指定角度
        :param angle: float, 旋转角度(度), 正值左转
        :param speed: float, 旋转角速度(rad/s)
        :param timeout: float, 超时时间(秒), 默认30秒
        :return: bool, 成功返回True
        """
        # 1. 参数检查：角速度必须为正数
        if speed <= 0:
            print("[Error] 角速度必须为正数")
            return False
        
        # 2. 将角度转换为弧度
        angle_rad = math.radians(angle)
        print(f"[Motion] 旋转角度: {angle}度 ({angle_rad:.2f}弧度)")
        
        try:
            # 3. 获取初始朝向
            initial_odom = self._get_odom()
            if initial_odom is None:
                return False
            
            # 4. 计算目标朝向
            initial_yaw = initial_odom['yaw']
            target_yaw = initial_yaw + angle_rad
            
            # 5. 归一化角度到 [-pi, pi] 范围
            # 使用atan2处理角度归一化，避免角度累加导致的数值问题
            target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
            
            # 6. 确定旋转方向
            # 正角度左转（逆时针），负角度右转（顺时针）
            w_z = speed if angle >= 0 else -speed
            
            # 7. 闭环控制：持续旋转直到到达目标角度或超时
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                # 获取当前朝向
                current_odom = self._get_odom()
                if current_odom is None:
                    break
                
                current_yaw = current_odom['yaw']
                
                # 计算剩余需要旋转的角度
                # 处理角度跨越-pi到pi的边界情况
                remaining_angle = target_yaw - current_yaw
                remaining_angle = math.atan2(math.sin(remaining_angle), math.cos(remaining_angle))
                
                # 判断是否到达目标（允许约1度的误差）
                if abs(remaining_angle) < 0.02:  # 0.02弧度 ≈ 1.15度
                    break
                
                # 持续发送旋转命令
                self.set_velocity(0.0, 0.0, w_z)
                time.sleep(0.1)  # 100ms控制周期
            
            # 8. 停止旋转
            self.set_velocity(0.0, 0.0, 0.0)
            
            # 9. 检查是否超时
            if (time.time() - start_time) >= timeout:
                print("[Warning] 旋转超时")
                return False
            
            print("[Motion] 旋转完成")
            return True
            
        except Exception as e:
            # 发生异常时确保停止运动
            print(f"[Error] 旋转过程出错: {e}")
            self.set_velocity(0.0, 0.0, 0.0)
            return False

    def get_wheel_speeds(self):
        """
        获取四轮实时速度
        :return: dict, {'fl': front_left, 'fr': front_right, 'rl': rear_left, 'rr': rear_right}
        
        注意: 此函数需要根据实际的ROS2消息格式进行解析
              当前返回占位符数据，实际使用时需要完善
        """
        try:
            cmd = ["ros2", "topic", "echo", "/wheel_speeds", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # TODO: 解析轮速消息
            speeds = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
            return speeds
        except Exception as e:
            print(f"[Error] 获取轮速失败: {e}")
            return None

    def get_imu_data(self):
        """
        获取陀螺仪6轴信息
        :return: dict, {'accel': [x,y,z], 'gyro': [x,y,z]}
        
        注意: 此函数需要根据实际的ROS2 Imu消息格式进行解析
              当前返回占位符数据，实际使用时需要完善
        """
        try:
            cmd = ["ros2", "topic", "echo", "/imu", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # TODO: 解析IMU消息
            imu_data = {
                'accel': [0.0, 0.0, 0.0],
                'gyro': [0.0, 0.0, 0.0]
            }
            return imu_data
        except Exception as e:
            print(f"[Error] 获取IMU数据失败: {e}")
            return None

    def get_pose(self):
        """
        获取车身位姿
        :return: dict, {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
        """
        return self._get_odom()

    def set_wheel_speeds(self, fl, fr, rl, rr):
        """
        直接设置4个轮子的速度
        :param fl: float, 左前轮速度
        :param fr: float, 右前轮速度
        :param rl: float, 左后轮速度
        :param rr: float, 右后轮速度
        """
        cmd = [
            "ros2", "topic", "pub", "--once", "/wheel_speed_cmd",
            "wheeltec_msgs/msg/WheelSpeed",
            f"{{fl: {fl}, fr: {fr}, rl: {rl}, rr: {rr}}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置轮速失败: {e}")

    def set_ackermann_angle(self, angle):
        """
        设置阿克曼转向角度
        :param angle: float, 转向角度(度)
        """
        if self.robot_type != "akm":
            print("[Warning] 仅阿克曼车型支持此功能")
            return
        
        angle_rad = math.radians(angle)
        cmd = [
            "ros2", "topic", "pub", "--once", "/ackermann_cmd",
            "ackermann_msgs/msg/AckermannDrive",
            f"{{steering_angle: {angle_rad}}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置转向角失败: {e}")

    def _get_odom(self):
        """
        内部函数: 获取里程计信息
        :return: dict, {'x': x, 'y': y, 'yaw': yaw}
        
        注意: 此函数需要根据实际的ROS2 Odometry消息格式进行解析
              当前返回占位符数据，实际使用时需要完善
        """
        try:
            cmd = ["ros2", "topic", "echo", "/odom", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # TODO: 解析Odometry消息
            # 需要从output中提取position.x, position.y和orientation (quaternion转euler)
            odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            return odom
        except Exception as e:
            return None
    
    # ==================== 机械臂控制 ====================
    
    def arm_home(self):
        """
        机械臂复位到初始姿态
        :return: bool, 成功返回True
        """
        print("[Arm] 机械臂复位中...")
        cmd = ["ros2", "service", "call", "/arm_home", "std_srvs/srv/Trigger"]
        
        try:
            subprocess.run(cmd, timeout=5.0)
            print("[Arm] 复位完成")
            return True
        except Exception as e:
            print(f"[Error] 复位失败: {e}")
            return False

    def set_joint_angles(self, joint1, joint2):
        """
        设置机械臂关节角度
        :param joint1: float, 大臂角度(度)
        :param joint2: float, 小臂角度(度)
        """
        print(f"[Arm] 设置关节角度: joint1={joint1}°, joint2={joint2}°")
        
        joint1_rad = math.radians(joint1)
        joint2_rad = math.radians(joint2)
        
        cmd = [
            "ros2", "topic", "pub", "--once", "/joint_cmd",
            "sensor_msgs/msg/JointState",
            f"{{position: [{joint1_rad}, {joint2_rad}]}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置关节角度失败: {e}")

    def set_yaw_angle(self, angle):
        """
        设置云台旋转角度
        :param angle: float, 云台角度(度)
        """
        print(f"[Arm] 设置云台角度: {angle}°")
        
        angle_rad = math.radians(angle)
        cmd = [
            "ros2", "topic", "pub", "--once", "/yaw_cmd",
            "std_msgs/msg/Float64",
            f"{{data: {angle_rad}}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置云台角度失败: {e}")

    def set_arm_position(self, x, y):
        """
        通过逆运动学控制机械臂末端位置
        :param x: float, 目标X坐标(mm)
        :param y: float, 目标Y坐标(mm)
        """
        print(f"[Arm] 移动末端到位置: x={x}mm, y={y}mm")
        
        cmd = [
            "ros2", "service", "call", "/arm_ik",
            "wheeltec_msgs/srv/ArmIK",
            f"{{x: {x}, y: {y}}}"
        ]
        
        try:
            subprocess.run(cmd, timeout=5.0, stdout=subprocess.DEVNULL)
        except Exception as e:
            print(f"[Error] 移动末端失败: {e}")

    def set_gripper(self, value):
        """
        控制夹爪开合程度
        :param value: int, 0-10, 0完全闭合, 10完全张开
        """
        if not 0 <= value <= 10:
            print("[Error] 夹爪值必须在0-10之间")
            return
        
        print(f"[Arm] 设置夹爪: {value}")
        
        cmd = [
            "ros2", "topic", "pub", "--once", "/gripper_cmd",
            "std_msgs/msg/Int32",
            f"{{data: {value}}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置夹爪失败: {e}")

    def get_arm_pose_xy(self):
        """
        获取机械臂末端当前坐标
        :return: tuple, (x, y) 单位mm
        """
        try:
            cmd = ["ros2", "topic", "echo", "/arm_pose", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # 解析位置信息
            x, y = 0.0, 0.0
            return (x, y)
        except Exception as e:
            print(f"[Error] 获取末端位置失败: {e}")
            return None

    def get_joint_states(self):
        """
        获取所有关节的实时角度
        :return: tuple, (joint1_angle, joint2_angle, yaw_angle) 单位度
        """
        try:
            cmd = ["ros2", "topic", "echo", "/joint_states", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # 解析关节状态
            joint1, joint2, yaw = 0.0, 0.0, 0.0
            return (math.degrees(joint1), math.degrees(joint2), math.degrees(yaw))
        except Exception as e:
            print(f"[Error] 获取关节状态失败: {e}")
            return None

    def set_pwm(self, pin, duty_cycle):
        """
        设置主控板IO引脚的PWM占空比
        :param pin: int, 引脚编号
        :param duty_cycle: float, 占空比 0.0-1.0
        """
        if not 0.0 <= duty_cycle <= 1.0:
            print("[Error] 占空比必须在0.0-1.0之间")
            return
        
        print(f"[System] 设置引脚{pin} PWM={duty_cycle*100:.1f}%")
        
        cmd = [
            "ros2", "topic", "pub", "--once", "/pwm_cmd",
            "wheeltec_msgs/msg/PWM",
            f"{{pin: {pin}, duty_cycle: {duty_cycle}}}"
        ]
        
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception as e:
            print(f"[Error] 设置PWM失败: {e}")
    
    # ==================== 感知与功能 ====================
    
    def launch_lidar(self):
        """
        启动雷达驱动和可视化
        :return: bool, 成功返回True
        """
        print("[Sensor] 启动雷达...")
        
        # 1. 构建启动雷达驱动的命令
        cmd = ["ros2", "launch", "wheeltec_lidar", "wheeltec_lidar.launch.py"]
        
        try:
            # 2. 启动雷达驱动进程
            self.lidar_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            # 等待3秒让雷达初始化
            time.sleep(3)
            
            # 3. 检查进程是否正常运行
            if self.lidar_process.poll() is not None:
                print("[Error] 雷达启动失败")
                return False
            
            # 4. 启动rviz可视化工具
            # 尝试找到雷达包的配置文件路径
            try:
                # 获取wheeltec_lidar包的安装路径
                pkg_prefix = subprocess.check_output(
                    ["ros2", "pkg", "prefix", "wheeltec_lidar"],
                    timeout=2.0
                ).decode("utf-8").strip()
                
                # 构建完整的rviz配置文件路径
                rviz_config_path = f"{pkg_prefix}/share/wheeltec_lidar/rviz/lidar.rviz"
                
                # 启动rviz并加载配置文件
                subprocess.Popen(
                    ["rviz2", "-d", rviz_config_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception:
                # 如果找不到配置文件或包不存在，启动默认rviz
                subprocess.Popen(
                    ["rviz2"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            
            print("[Sensor] 雷达启动成功")
            return True
            
        except Exception as e:
            print(f"[Error] 启动雷达失败: {e}")
            return False

    def stop_lidar(self):
        """
        关闭雷达驱动
        """
        if self.lidar_process:
            print("[Sensor] 关闭雷达...")
            try:
                # 发送SIGINT信号优雅关闭
                self.lidar_process.send_signal(signal.SIGINT)
                # 等待最多5秒
                self.lidar_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # 超时则强制kill
                self.lidar_process.kill()
            self.lidar_process = None
            print("[Sensor] 雷达已关闭")

    def launch_camera(self):
        """
        启动相机驱动和可视化
        :return: bool, 成功返回True
        """
        print("[Sensor] 启动相机...")
        
        cmd = ["ros2", "launch", "wheeltec_camera", "wheeltec_camera.launch.py"]
        
        try:
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.camera_process.poll() is not None:
                print("[Error] 相机启动失败")
                return False
            
            # 启动rqt_image_view可视化
            subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            print("[Sensor] 相机启动成功")
            return True
            
        except Exception as e:
            print(f"[Error] 启动相机失败: {e}")
            return False

    def stop_camera(self):
        """
        关闭相机驱动
        """
        if self.camera_process:
            print("[Sensor] 关闭相机...")
            try:
                self.camera_process.send_signal(signal.SIGINT)
                self.camera_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
            self.camera_process = None
            print("[Sensor] 相机已关闭")

    def start_visual_follow(self, color, control_enabled=True):
        """
        启动视觉跟随
        :param color: str, 目标颜色 'red', 'blue', 'green', 'yellow'
        :param control_enabled: bool, 是否自动控制底盘
        :return: bool, 成功返回True
        
        注意: callback功能待实现，当前版本使用 get_visual_target_info() 获取目标信息
        """
        print(f"[App] 启动视觉跟随: {color}")
        
        # 构建视觉跟随的launch命令
        # color: 指定要跟随的颜色（需要预定义HSV阈值）
        # control: true表示自动控制底盘，false仅检测不控制
        cmd = [
            "ros2", "launch", "wheeltec_vision", "visual_follow.launch.py",
            f"color:={color}",
            f"control:={'true' if control_enabled else 'false'}"
        ]
        
        try:
            # 启动视觉跟随进程
            self.visual_follow_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            # 等待2秒让视觉节点初始化
            time.sleep(2)
            
            # 检查进程是否成功启动
            if self.visual_follow_process.poll() is not None:
                print("[Error] 视觉跟随启动失败")
                return False
            
            print("[App] 视觉跟随已启动")
            return True
            
        except Exception as e:
            print(f"[Error] 启动视觉跟随失败: {e}")
            return False

    def start_visual_follow_custom(self, hsv_lower, hsv_upper, control_enabled=True):
        """
        使用自定义HSV颜色范围启动视觉跟随
        :param hsv_lower: tuple, HSV下界 (h, s, v)
        :param hsv_upper: tuple, HSV上界 (h, s, v)
        :param control_enabled: bool, 是否自动控制底盘
        
        HSV颜色空间说明:
        - H (色调): 0-180, 表示颜色类型（红0-10/170-180, 绿35-85, 蓝100-130）
        - S (饱和度): 0-255, 表示颜色纯度（越高越鲜艳）
        - V (亮度): 0-255, 表示颜色明暗（越高越亮）
        """
        print(f"[App] 启动自定义颜色视觉跟随")
        
        # 构建自定义HSV参数的命令
        # hsv_lower和hsv_upper定义要检测的颜色范围
        cmd = [
            "ros2", "launch", "wheeltec_vision", "visual_follow.launch.py",
            f"hsv_lower:=[{hsv_lower[0]},{hsv_lower[1]},{hsv_lower[2]}]",
            f"hsv_upper:=[{hsv_upper[0]},{hsv_upper[1]},{hsv_upper[2]}]",
            f"control:={'true' if control_enabled else 'false'}"
        ]
        
        try:
            self.visual_follow_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            return True
        except Exception as e:
            print(f"[Error] 启动自定义视觉跟随失败: {e}")
            return False

    def get_visual_target_info(self):
        """
        获取视觉跟随目标信息
        :return: dict, {'detected': bool, 'x': x, 'y': y, 'area': area}
        """
        try:
            cmd = ["ros2", "topic", "echo", "/visual_target", "--once"]
            output = subprocess.check_output(cmd, timeout=1.0).decode("utf-8")
            # 解析目标信息
            info = {'detected': False, 'x': 0, 'y': 0, 'area': 0}
            return info
        except Exception as e:
            return {'detected': False, 'x': 0, 'y': 0, 'area': 0}

    def start_line_tracking(self, color, control_enabled=True):
        """
        启动视觉巡线
        :param color: str, 线条颜色 'black', 'red', 'yellow'
        :param control_enabled: bool, 是否自动控制底盘
        :return: bool, 成功返回True
        
        注意: callback功能待实现，当前版本使用 get_line_info() 获取线条信息
        """
        print(f"[App] 启动巡线: {color}")
        
        cmd = [
            "ros2", "launch", "wheeltec_vision", "line_tracking.launch.py",
            f"color:={color}",
            f"control:={'true' if control_enabled else 'false'}"
        ]
        
        try:
            self.line_tracking_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.line_tracking_process.poll() is not None:
                print("[Error] 巡线启动失败")
                return False
            
            print("[App] 巡线已启动")
            return True
            
        except Exception as e:
            print(f"[Error] 启动巡线失败: {e}")
            return False

    def start_line_tracking_custom(self, hsv_lower, hsv_upper, control_enabled=True):
        """
        使用自定义HSV颜色范围启动巡线
        :param hsv_lower: tuple, HSV下界 (h, s, v)
        :param hsv_upper: tuple, HSV上界 (h, s, v)
        :param control_enabled: bool, 是否自动控制底盘
        """
        print(f"[App] 启动自定义颜色巡线")
        
        cmd = [
            "ros2", "launch", "wheeltec_vision", "line_tracking.launch.py",
            f"hsv_lower:=[{hsv_lower[0]},{hsv_lower[1]},{hsv_lower[2]}]",
            f"hsv_upper:=[{hsv_upper[0]},{hsv_upper[1]},{hsv_upper[2]}]",
            f"control:={'true' if control_enabled else 'false'}"
        ]
        
        try:
            self.line_tracking_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            return True
        except Exception as e:
            print(f"[Error] 启动自定义巡线失败: {e}")
            return False

    def get_line_info(self):
        """
        获取巡线线条信息
        :return: dict, {'detected': bool, 'offset': offset, 'angle': angle}
        """
        try:
            cmd = ["ros2", "topic", "echo", "/line_info", "--once"]
            output = subprocess.check_output(cmd, timeout=1.0).decode("utf-8")
            # 解析线条信息
            info = {'detected': False, 'offset': 0.0, 'angle': 0.0}
            return info
        except Exception as e:
            return {'detected': False, 'offset': 0.0, 'angle': 0.0}

    def start_lidar_follow(self, target_dist):
        """
        启动雷达跟随
        :param target_dist: float, 保持的跟随距离(m)
        :return: bool, 成功返回True
        """
        print(f"[App] 启动雷达跟随: 距离={target_dist}m")
        
        cmd = [
            "ros2", "launch", "wheeltec_lidar", "lidar_follow.launch.py",
            f"target_distance:={target_dist}"
        ]
        
        try:
            self.lidar_follow_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.lidar_follow_process.poll() is not None:
                print("[Error] 雷达跟随启动失败")
                return False
            
            print("[App] 雷达跟随已启动")
            return True
            
        except Exception as e:
            print(f"[Error] 启动雷达跟随失败: {e}")
            return False

    def stop_application(self):
        """
        停止当前应用(跟随/巡线等)并停止底盘
        """
        print("[App] 停止应用...")
        
        # 停止底盘运动
        self.set_velocity(0.0, 0.0, 0.0)
        
        # 停止所有应用进程
        app_processes = [
            (self.visual_follow_process, "视觉跟随"),
            (self.line_tracking_process, "巡线"),
            (self.lidar_follow_process, "雷达跟随")
        ]
        
        for process, name in app_processes:
            if process:
                try:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=3)
                    print(f"[App] {name}已停止")
                except subprocess.TimeoutExpired:
                    process.kill()
                except Exception as e:
                    print(f"[Warning] 停止{name}时出错: {e}")
        
        self.visual_follow_process = None
        self.line_tracking_process = None
        self.lidar_follow_process = None
        
        print("[App] 应用已停止")

    def capture_image(self):
        """
        获取相机当前的一帧图像(拍照)
        :return: numpy array or None, 图像数据
        """
        try:
            # 使用ros2 topic echo获取一帧图像
            cmd = ["ros2", "topic", "echo", "/camera/image_raw", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            print("[Sensor] 图像已捕获")
            # 实际应用中需要解析sensor_msgs/Image消息
            # 这里返回None作为占位符
            return None
        except Exception as e:
            print(f"[Error] 捕获图像失败: {e}")
            return None

    def get_lidar_distance(self, angle):
        """
        获取雷达指定角度的距离信息
        :param angle: float, 角度(度), 0为正前方
        :return: float, 距离(m), 失败返回-1
        """
        try:
            cmd = ["ros2", "topic", "echo", "/scan", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # 解析LaserScan消息获取指定角度的距离
            # 这里需要根据angle_min, angle_max, angle_increment计算索引
            distance = 0.0
            return distance
        except Exception as e:
            print(f"[Error] 获取雷达距离失败: {e}")
            return -1.0
    
    # ==================== 建图与导航 ====================
    
    def start_mapping(self, method="gmapping"):
        """
        启动SLAM建图
        :param method: str, 建图算法 "gmapping" 或 "cartographer"
        :return: bool, 成功返回True
        
        说明:
        - gmapping: 基于粒子滤波的2D SLAM算法，计算量小，适合实时建图
        - cartographer: Google开发的SLAM算法，精度高但计算量大
        """
        print(f"[SLAM] 启动建图: {method}")
        
        # 1. 检查建图算法是否支持
        if method not in ["gmapping", "cartographer"]:
            print("[Error] 不支持的建图方法，支持: gmapping, cartographer")
            return False
        
        # 2. 构建启动建图的命令
        cmd = [
            "ros2", "launch", "wheeltec_slam", f"{method}.launch.py"
        ]
        
        try:
            # 3. 启动建图进程（非阻塞）
            # 建图节点会在后台运行，可以同时控制机器人移动
            self.mapping_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            # 等待3秒让SLAM节点初始化
            time.sleep(3)
            
            if self.mapping_process.poll() is not None:
                print("[Error] 建图启动失败")
                return False
            
            print("[SLAM] 建图已启动，可以开始控制机器人移动建图")
            return True
            
        except Exception as e:
            print(f"[Error] 启动建图失败: {e}")
            return False

    def save_map(self, map_name):
        """
        保存当前构建的地图
        :param map_name: str, 地图文件名(不含扩展名)
        :return: bool, 成功返回True
        
        说明:
        - 地图保存为两个文件: .yaml (元数据) 和 .pgm (图像)
        - 默认保存在 ~/maps/ 目录下
        """
        print(f"[SLAM] 保存地图: {map_name}")
        
        # 1. 确保地图保存目录存在
        # 使用expanduser展开~为用户主目录
        map_dir = os.path.expanduser("~/maps")
        os.makedirs(map_dir, exist_ok=True)
        
        # 2. 构建完整的地图路径
        map_path = os.path.join(map_dir, map_name)
        
        # 3. 构建地图保存命令
        # nav2_map_server的map_saver_cli工具用于保存地图
        # -f 参数指定输出文件名（不含扩展名）
        cmd = [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", map_path
        ]
        
        try:
            # 4. 执行保存命令，设置10秒超时
            result = subprocess.run(cmd, timeout=10.0, capture_output=True)
            if result.returncode == 0:
                # 成功保存，会生成.yaml和.pgm两个文件
                print(f"[SLAM] 地图已保存: {map_path}.yaml 和 {map_path}.pgm")
                return True
            else:
                print(f"[Error] 保存地图失败: {result.stderr.decode()}")
                return False
        except Exception as e:
            print(f"[Error] 保存地图失败: {e}")
            return False

    def start_keyboard_control(self):
        """
        启动键盘控制
        阻塞式，直到用户按Ctrl+C
        
        说明:
        - 使用键盘方向键或WASD控制机器人移动
        - 常用于建图时手动控制机器人探索环境
        - 函数会阻塞当前线程，直到用户按Ctrl+C退出
        """
        print("\n" + "="*40)
        print("[App] 进入键盘控制模式...")
        print("[App] 按 Ctrl+C 退出")
        cmd = ["ros2", "run", "wheeltec_robot_keyboard", "wheeltec_keyboard"]
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            pass
        print("[App] 键盘控制已结束")

    def start_navigation(self, map_name):
        """
        启动导航系统
        :param map_name: str, 地图文件名(不含扩展名)
        :return: bool, 成功返回True
        """
        print(f"[Nav] 启动导航: {map_name}")
        
        map_path = os.path.expanduser(f"~/maps/{map_name}.yaml")
        
        if not os.path.exists(map_path):
            print(f"[Error] 地图文件不存在: {map_path}")
            return False
        
        cmd = [
            "ros2", "launch", "wheeltec_nav", "navigation.launch.py",
            f"map:={map_path}"
        ]
        
        try:
            self.navigation_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(5)
            
            if self.navigation_process.poll() is not None:
                print("[Error] 导航启动失败")
                return False
            
            print("[Nav] 导航系统已启动")
            return True
            
        except Exception as e:
            print(f"[Error] 启动导航失败: {e}")
            return False

    def move_to_goal(self, x, y, theta):
        """
        导航到目标点
        :param x: float, 目标X坐标(m)
        :param y: float, 目标Y坐标(m)
        :param theta: float, 目标朝向(度)
        :return: bool, 成功返回True
        """
        print(f"[Nav] 导航到目标: x={x}, y={y}, theta={theta}°")
        
        theta_rad = math.radians(theta)
        
        cmd = [
            "ros2", "topic", "pub", "--once", "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            f"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{z: {math.sin(theta_rad/2)}, w: {math.cos(theta_rad/2)}}}}}}}"
        ]
        
        try:
            subprocess.run(cmd, timeout=2.0, stdout=subprocess.DEVNULL)
            print("[Nav] 导航目标已发送")
            return True
        except Exception as e:
            print(f"[Error] 发送导航目标失败: {e}")
            return False

    def cancel_navigation(self):
        """
        取消当前导航任务
        """
        print("[Nav] 取消导航...")
        
        try:
            # 发送取消命令
            subprocess.run(["ros2", "action", "cancel_goal", "/navigate_to_pose"], timeout=2.0)
            print("[Nav] 导航已取消")
        except Exception as e:
            print(f"[Error] 取消导航失败: {e}")

    def get_navigation_status(self):
        """
        获取导航状态
        :return: str, "idle", "navigating", "reached", "failed"
        """
        try:
            # 查询action状态
            cmd = ["ros2", "action", "list"]
            output = subprocess.check_output(cmd, timeout=1.0).decode("utf-8")
            # 解析状态
            return "idle"
        except Exception as e:
            return "unknown"

    def get_current_map(self):
        """
        获取当前建图进度的地图数据
        :return: dict or None, 地图信息
        """
        try:
            cmd = ["ros2", "topic", "echo", "/map", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # 解析OccupancyGrid消息
            return None
        except Exception as e:
            print(f"[Error] 获取地图失败: {e}")
            return None