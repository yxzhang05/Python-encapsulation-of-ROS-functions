# 文件名: robot_lib.py
import subprocess
import time
import signal
import re
import os
import math
import threading
import json

class Robot:
    def __init__(self):
        self.driver_process = None  # 底盘驱动进程句柄
        self.keyboard_process = None # 键盘进程句柄
        self.lidar_process = None   # 雷达进程句柄
        self.camera_process = None  # 相机进程句柄
        self.mapping_process = None # 建图进程句柄
        self.nav_process = None     # 导航进程句柄
        self.app_process = None     # 应用功能进程句柄（跟随、巡线等）
        self.arm_process = None     # 机械臂进程句柄
        
        # 定义车型映射关系
        # 键是 initialize 传入的简写，值是传给 launch 文件的具体参数
        # 如果你的 launch 文件不需要转换，可以直接传值
        self.ROBOT_TYPE_MAP = {
            "akm": "ackermann",    # 阿克曼
            "diff": "diff",        # 差速
            "mec": "mecanum"       # 麦轮
        }
        
        # 当前机器人类型
        self.current_robot_type = None

    def initialize(self, robot_type):
        """
        初始化机器人底盘
        :param robot_type: str, 车型枚举 "akm", "diff", "mec"
        """
        # 1. 检查参数合法性
        if robot_type not in self.ROBOT_TYPE_MAP:
            print(f"[Error] 未知的车型: {robot_type}. 支持: {list(self.ROBOT_TYPE_MAP.keys())}")
            return False

        # 获取实际的 ROS 参数值
        real_type_name = self.ROBOT_TYPE_MAP[robot_type]
        self.current_robot_type = robot_type
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
        
        # 关闭其他可能运行的进程
        for process_name, process in [
            ("雷达", self.lidar_process),
            ("相机", self.camera_process),
            ("建图", self.mapping_process),
            ("导航", self.nav_process),
            ("应用功能", self.app_process),
            ("机械臂", self.arm_process)
        ]:
            if process:
                try:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=3)
                    print(f"[System] {process_name}已关闭。")
                except:
                    try:
                        process.kill()
                    except:
                        pass
        
        self.lidar_process = None
        self.camera_process = None
        self.mapping_process = None
        self.nav_process = None
        self.app_process = None
        self.arm_process = None

    # ==================== 系统管理函数 ====================
    
    def emergency_stop(self):
        """
        软件急停，立即发送全0速度指令
        原理：发布一个空的速度指令到 /cmd_vel 话题
        """
        print("[Emergency] 执行急停！")
        try:
            # 使用 ros2 topic pub 命令发送一次空速度
            cmd = [
                "ros2", "topic", "pub", "--once",
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
            ]
            subprocess.run(cmd, timeout=1.0)
            print("[Emergency] 急停指令已发送！")
            return True
        except Exception as e:
            print(f"[Error] 急停失败: {e}")
            return False
    
    def get_software_version(self):
        """
        获取下位机软件或固件的版本号
        注意：这个功能需要下位机支持版本查询话题或服务
        假设版本信息发布在 /version 话题
        """
        try:
            # 尝试从版本话题获取版本信息
            cmd = ["ros2", "topic", "echo", "/version", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            print(f"[System] 软件版本: {output.strip()}")
            return output.strip()
        except:
            # 如果没有版本话题，返回未知
            print("[Warning] 无法获取软件版本信息（下位机可能不支持此功能）")
            return "Unknown"

    # ==================== 底盘运动控制函数 ====================
    
    def set_velocity(self, v_x, v_y, w_z):
        """
        设置底盘速度（最底层控制接口）
        :param v_x: float, 前进线速度，单位 m/s (正向前进)
        :param v_y: float, 横向速度，单位 m/s (仅麦轮有效，左为正)
        :param w_z: float, 角速度，单位 rad/s (逆时针为正)
        
        适用于所有车型：
        - 阿克曼/差速: 只使用 v_x 和 w_z，v_y 应设为 0
        - 麦轮: 可同时使用 v_x, v_y, w_z 实现全向移动
        """
        try:
            # 构建速度消息的JSON格式
            twist_msg = {
                "linear": {"x": float(v_x), "y": float(v_y), "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": float(w_z)}
            }
            
            # 使用 ros2 topic pub 发送速度指令
            cmd = [
                "ros2", "topic", "pub", "--once",
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                json.dumps(twist_msg)
            ]
            subprocess.run(cmd, timeout=0.5, stdout=subprocess.DEVNULL)
            return True
        except Exception as e:
            print(f"[Error] 速度设置失败: {e}")
            return False
    
    def move_distance(self, distance, speed, lateral_distance=0.0, lateral_speed=0.0):
        """
        闭环控制移动指定距离
        :param distance: float, 前向移动距离，单位米（负数后退）
        :param speed: float, 移动速度，单位 m/s（必须为正）
        :param lateral_distance: float, 横向移动距离，单位米（仅麦轮，正值向左）
        :param lateral_speed: float, 横向速度，单位 m/s（仅麦轮）
        
        功能说明：
        - 阿克曼/差速: 只使用 distance 和 speed 参数，忽略横向参数
        - 麦轮: 可同时使用前向和横向参数实现斜向移动
        
        原理：通过订阅 /odom 话题获取里程计反馈，计算已移动距离
        """
        print(f"[Motion] 开始移动 - 前向: {distance}m, 横向: {lateral_distance}m")
        
        try:
            # 获取初始位置
            start_pos = self._get_odom_position()
            if start_pos is None:
                print("[Error] 无法获取里程计数据")
                return False
            
            start_x, start_y = start_pos
            
            # 计算目标距离
            target_distance = math.sqrt(distance**2 + lateral_distance**2)
            
            # 确定运动方向
            v_x = speed if distance >= 0 else -speed
            v_y = lateral_speed if lateral_distance >= 0 else -lateral_speed
            
            # 仅麦轮使用横向速度
            if self.current_robot_type != "mec":
                v_y = 0.0
            
            # 开始移动
            moved_distance = 0.0
            while moved_distance < target_distance:
                # 发送速度指令
                self.set_velocity(v_x, v_y, 0.0)
                time.sleep(0.1)
                
                # 获取当前位置
                current_pos = self._get_odom_position()
                if current_pos is None:
                    continue
                
                current_x, current_y = current_pos
                
                # 计算已移动距离
                moved_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            # 停止运动
            self.set_velocity(0.0, 0.0, 0.0)
            print(f"[Motion] 移动完成，实际移动距离: {moved_distance:.3f}m")
            return True
            
        except Exception as e:
            print(f"[Error] 移动过程出错: {e}")
            self.set_velocity(0.0, 0.0, 0.0)
            return False
    
    def rotate_angle(self, angle, speed=0.5):
        """
        原地旋转指定角度
        :param angle: float, 旋转角度，单位度（正值逆时针，负值顺时针）
        :param speed: float, 旋转角速度，单位 rad/s（默认0.5）
        
        原理：通过订阅 /odom 或 /imu 话题获取当前朝向，计算旋转量
        """
        print(f"[Motion] 开始旋转 {angle} 度")
        
        try:
            # 获取初始角度
            start_yaw = self._get_odom_yaw()
            if start_yaw is None:
                print("[Error] 无法获取姿态数据")
                return False
            
            # 将角度转换为弧度
            target_angle_rad = math.radians(abs(angle))
            
            # 确定旋转方向
            w_z = speed if angle >= 0 else -speed
            
            # 开始旋转
            rotated_angle = 0.0
            last_yaw = start_yaw
            
            while rotated_angle < target_angle_rad:
                # 发送旋转指令
                self.set_velocity(0.0, 0.0, w_z)
                time.sleep(0.1)
                
                # 获取当前角度
                current_yaw = self._get_odom_yaw()
                if current_yaw is None:
                    continue
                
                # 计算角度变化（处理角度跳变）
                delta_yaw = current_yaw - last_yaw
                if delta_yaw > math.pi:
                    delta_yaw -= 2 * math.pi
                elif delta_yaw < -math.pi:
                    delta_yaw += 2 * math.pi
                
                rotated_angle += abs(delta_yaw)
                last_yaw = current_yaw
            
            # 停止旋转
            self.set_velocity(0.0, 0.0, 0.0)
            print(f"[Motion] 旋转完成，实际旋转角度: {math.degrees(rotated_angle):.1f}度")
            return True
            
        except Exception as e:
            print(f"[Error] 旋转过程出错: {e}")
            self.set_velocity(0.0, 0.0, 0.0)
            return False
    
    def get_wheel_speeds(self):
        """
        获取四个轮子的实时速度
        返回: (front_left, front_right, rear_left, rear_right) 单位 rad/s
        注意：需要底盘发布轮速话题，话题名可能是 /wheel_speeds 或类似
        """
        try:
            cmd = ["ros2", "topic", "echo", "/wheel_speeds", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            # 这里需要根据实际消息格式解析
            # 示例假设返回格式包含4个速度值
            print(f"[Info] 轮速数据: {output}")
            return output
        except Exception as e:
            print(f"[Warning] 无法获取轮速: {e}")
            return None
    
    def get_imu_data(self):
        """
        获取陀螺仪6轴信息（加速度xyz + 角速度xyz）
        返回: dict 包含 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        """
        try:
            # 获取IMU数据
            cmd = ["ros2", "topic", "echo", "/imu", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            
            # 简化处理：返回原始输出
            # 实际应用中应该解析为字典格式
            print(f"[Info] IMU数据获取成功")
            return output
            
        except Exception as e:
            print(f"[Warning] 无法获取IMU数据: {e}")
            return None
    
    def get_robot_pose(self):
        """
        获取机器人当前位姿 (x, y, yaw)
        返回: (x, y, yaw) 其中 x,y 单位为米，yaw 单位为弧度
        """
        try:
            pos = self._get_odom_position()
            yaw = self._get_odom_yaw()
            
            if pos and yaw is not None:
                print(f"[Info] 当前位姿: x={pos[0]:.3f}m, y={pos[1]:.3f}m, yaw={math.degrees(yaw):.1f}度")
                return (pos[0], pos[1], yaw)
            else:
                print("[Warning] 无法获取位姿信息")
                return None
                
        except Exception as e:
            print(f"[Error] 获取位姿失败: {e}")
            return None
    
    def set_wheel_speeds(self, fl, fr, rl, rr):
        """
        直接设置4个轮子的速度
        :param fl: float, 左前轮速度
        :param fr: float, 右前轮速度  
        :param rl: float, 左后轮速度
        :param rr: float, 右后轮速度
        注意：此功能需要下位机支持直接轮速控制，可能需要修改串口协议
        """
        print(f"[Motion] 设置轮速: FL={fl}, FR={fr}, RL={rl}, RR={rr}")
        print("[Warning] 此功能需要下位机支持，可能需要修改串口协议")
        # 实际实现需要根据具体的ROS话题或服务来完成
        return False
    
    def set_ackermann_angle(self, angle):
        """
        设置阿克曼转向角度
        :param angle: float, 转向角度，单位度
        注意：仅阿克曼车型有效，需要下位机支持
        """
        if self.current_robot_type != "akm":
            print("[Warning] 当前车型不是阿克曼，此功能无效")
            return False
        
        print(f"[Motion] 设置阿克曼转向角: {angle}度")
        print("[Warning] 此功能需要下位机支持，可能需要修改串口协议")
        # 实际实现需要根据具体的ROS话题或服务来完成
        return False

    # ==================== 机械臂控制函数 ====================
    
    def arm_home(self):
        """
        机械臂复位到初始姿态
        注意：需要机械臂控制节点已启动
        """
        print("[Arm] 机械臂复位中...")
        try:
            # 假设有一个服务或话题用于复位
            # 这里使用话题发布方式示例
            cmd = [
                "ros2", "service", "call",
                "/arm_home",
                "std_srvs/srv/Trigger",
                "{}"
            ]
            result = subprocess.run(cmd, timeout=5.0, capture_output=True)
            print("[Arm] 机械臂已复位")
            return True
        except Exception as e:
            print(f"[Warning] 机械臂复位功能需要相应ROS节点支持: {e}")
            return False
    
    def set_joint_angles(self, joint1, joint2):
        """
        设置机械臂关节角度
        :param joint1: float, 大臂角度，单位度
        :param joint2: float, 小臂角度，单位度
        """
        print(f"[Arm] 设置关节角度: Joint1={joint1}°, Joint2={joint2}°")
        try:
            # 发布关节角度指令
            # 需要根据实际的机械臂话题调整
            print("[Warning] 此功能需要机械臂ROS节点支持")
            return False
        except Exception as e:
            print(f"[Error] 设置关节角度失败: {e}")
            return False
    
    def set_yaw_angle(self, angle):
        """
        设置机械臂云台/独立旋转关节角度
        :param angle: float, 角度，单位度
        """
        print(f"[Arm] 设置云台角度: {angle}°")
        print("[Warning] 此功能需要机械臂ROS节点支持")
        return False
    
    def set_arm_position(self, x, y):
        """
        设置机械臂末端位置（通过逆运动学）
        :param x: float, 目标X坐标，单位 mm
        :param y: float, 目标Y坐标，单位 mm
        """
        print(f"[Arm] 设置末端位置: X={x}mm, Y={y}mm")
        print("[Warning] 此功能需要机械臂逆运动学节点支持")
        return False
    
    def set_gripper(self, value):
        """
        控制夹爪开合
        :param value: int, 夹爪开合程度 0-10 (0=完全闭合, 10=完全张开)
        """
        if not 0 <= value <= 10:
            print("[Error] 夹爪值必须在0-10之间")
            return False
        
        print(f"[Arm] 设置夹爪: {value}/10")
        print("[Warning] 此功能需要夹爪控制节点支持")
        return False
    
    def get_arm_pose_xy(self):
        """
        获取机械臂末端当前坐标
        返回: (x, y) 单位 mm
        """
        print("[Arm] 获取机械臂末端坐标")
        print("[Warning] 此功能需要机械臂状态发布节点支持")
        return None
    
    def get_joint_states(self):
        """
        获取所有关节的当前角度
        返回: (joint1, joint2, yaw) 单位度
        """
        print("[Arm] 获取关节状态")
        print("[Warning] 此功能需要关节状态话题支持")
        return None
    
    def set_pwm(self, pin, duty_cycle):
        """
        设置主控板IO引脚的PWM占空比
        :param pin: int, IO引脚编号
        :param duty_cycle: float, 占空比 0.0-1.0
        """
        print(f"[IO] 设置引脚{pin}的PWM占空比: {duty_cycle}")
        print("[Warning] 此功能需要下位机PWM控制支持")
        return False

    # ==================== 感知与功能函数 ====================
    
    def launch_lidar(self):
        """
        启动雷达驱动节点
        """
        if self.lidar_process:
            print("[Warning] 雷达已经在运行")
            return False
        
        print("[Sensor] 启动雷达驱动...")
        try:
            cmd = ["ros2", "launch", "turn_on_wheeltec_robot", "wheeltec_lidar.launch.py"]
            self.lidar_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(3)  # 等待雷达初始化
            
            if self.lidar_process.poll() is not None:
                print("[Error] 雷达启动失败")
                self.lidar_process = None
                return False
            
            print("[Sensor] 雷达已启动，数据发布在 /scan 话题")
            return True
        except Exception as e:
            print(f"[Error] 启动雷达失败: {e}")
            return False
    
    def stop_lidar(self):
        """
        关闭雷达驱动节点
        """
        if not self.lidar_process:
            print("[Warning] 雷达未运行")
            return False
        
        print("[Sensor] 关闭雷达...")
        try:
            self.lidar_process.send_signal(signal.SIGINT)
            self.lidar_process.wait(timeout=3)
            self.lidar_process = None
            print("[Sensor] 雷达已关闭")
            return True
        except:
            self.lidar_process.kill()
            self.lidar_process = None
            return True
    
    def launch_camera(self):
        """
        启动相机驱动节点
        """
        if self.camera_process:
            print("[Warning] 相机已经在运行")
            return False
        
        print("[Sensor] 启动相机驱动...")
        try:
            cmd = ["ros2", "launch", "turn_on_wheeltec_robot", "wheeltec_camera.launch.py"]
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(3)  # 等待相机初始化
            
            if self.camera_process.poll() is not None:
                print("[Error] 相机启动失败")
                self.camera_process = None
                return False
            
            print("[Sensor] 相机已启动，图像发布在 /camera/image_raw 话题")
            return True
        except Exception as e:
            print(f"[Error] 启动相机失败: {e}")
            return False
    
    def stop_camera(self):
        """
        关闭相机驱动节点
        """
        if not self.camera_process:
            print("[Warning] 相机未运行")
            return False
        
        print("[Sensor] 关闭相机...")
        try:
            self.camera_process.send_signal(signal.SIGINT)
            self.camera_process.wait(timeout=3)
            self.camera_process = None
            print("[Sensor] 相机已关闭")
            return True
        except:
            self.camera_process.kill()
            self.camera_process = None
            return True
    
    def start_visual_follow(self, color):
        """
        启动视觉跟随
        :param color: str, 目标颜色 'red', 'blue', 'green', 'yellow'
        """
        if self.app_process:
            print("[Warning] 已有应用在运行，请先停止")
            return False
        
        print(f"[App] 启动视觉跟随 - 目标颜色: {color}")
        try:
            cmd = ["ros2", "launch", "simple_follower_ros2", "visual_follower.launch.py",
                   f"target_color:={color}"]
            self.app_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.app_process.poll() is not None:
                print("[Error] 视觉跟随启动失败")
                self.app_process = None
                return False
            
            print("[App] 视觉跟随已启动")
            return True
        except Exception as e:
            print(f"[Error] 启动视觉跟随失败: {e}")
            return False
    
    def start_line_tracking(self, color='black'):
        """
        启动视觉巡线
        :param color: str, 线条颜色 'black', 'red', 'yellow'
        """
        if self.app_process:
            print("[Warning] 已有应用在运行，请先停止")
            return False
        
        print(f"[App] 启动视觉巡线 - 线条颜色: {color}")
        try:
            cmd = ["ros2", "launch", "simple_follower_ros2", "line_follower.launch.py",
                   f"line_color:={color}"]
            self.app_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.app_process.poll() is not None:
                print("[Error] 视觉巡线启动失败")
                self.app_process = None
                return False
            
            print("[App] 视觉巡线已启动")
            return True
        except Exception as e:
            print(f"[Error] 启动视觉巡线失败: {e}")
            return False
    
    def start_lidar_follow(self, target_dist=0.5):
        """
        启动雷达跟随
        :param target_dist: float, 跟随距离，单位米（默认0.5米）
        """
        if self.app_process:
            print("[Warning] 已有应用在运行，请先停止")
            return False
        
        print(f"[App] 启动雷达跟随 - 目标距离: {target_dist}m")
        try:
            cmd = ["ros2", "launch", "simple_follower_ros2", "laser_follower.launch.py",
                   f"target_distance:={target_dist}"]
            self.app_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(2)
            
            if self.app_process.poll() is not None:
                print("[Error] 雷达跟随启动失败")
                self.app_process = None
                return False
            
            print("[App] 雷达跟随已启动")
            return True
        except Exception as e:
            print(f"[Error] 启动雷达跟随失败: {e}")
            return False
    
    def stop_application(self):
        """
        停止当前运行的应用（跟随/巡线等）
        """
        if not self.app_process:
            print("[Warning] 没有应用在运行")
            return False
        
        print("[App] 停止应用...")
        try:
            self.app_process.send_signal(signal.SIGINT)
            self.app_process.wait(timeout=3)
            self.app_process = None
            
            # 发送停止指令
            self.emergency_stop()
            print("[App] 应用已停止")
            return True
        except:
            self.app_process.kill()
            self.app_process = None
            self.emergency_stop()
            return True
    
    def capture_camera_frame(self):
        """
        获取相机当前一帧图像（拍照）
        返回: 图像数据或文件路径
        """
        print("[Camera] 捕获相机图像")
        print("[Warning] 此功能需要图像处理节点支持，建议使用ROS的image_saver")
        return None
    
    def get_lidar_distance(self, angle):
        """
        获取雷达指定角度的距离信息
        :param angle: float, 角度，单位度（0度为正前方）
        返回: float, 距离值，单位米
        """
        try:
            # 获取激光雷达数据
            cmd = ["ros2", "topic", "echo", "/scan", "--once"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            
            print(f"[Lidar] 获取{angle}度方向距离")
            print("[Info] 需要解析雷达数据，返回原始数据")
            return output
            
        except Exception as e:
            print(f"[Error] 获取雷达距离失败: {e}")
            return None

    # ==================== 建图与导航函数 ====================
    
    def start_mapping(self, method='slam_toolbox'):
        """
        启动SLAM建图
        :param method: str, 建图算法 'gmapping', 'cartographer', 'slam_toolbox'
        """
        if self.mapping_process:
            print("[Warning] 建图已在运行")
            return False
        
        print(f"[SLAM] 启动建图 - 算法: {method}")
        
        # 根据不同算法选择不同的launch文件
        launch_map = {
            'gmapping': ['ros2', 'launch', 'slam_gmapping', 'slam_gmapping.launch.py'],
            'cartographer': ['ros2', 'launch', 'wheeltec_cartographer', 'cartographer.launch.py'],
            'slam_toolbox': ['ros2', 'launch', 'wheeltec_slam_toolbox', 'online_async_launch.py']
        }
        
        if method not in launch_map:
            print(f"[Error] 未知的建图算法: {method}")
            return False
        
        try:
            cmd = launch_map[method]
            self.mapping_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(5)  # 等待建图节点初始化
            
            if self.mapping_process.poll() is not None:
                print("[Error] 建图启动失败")
                self.mapping_process = None
                return False
            
            print("[SLAM] 建图已启动，可以开始移动机器人")
            print("[SLAM] 提示：使用 start_keyboard_control() 或移动控制函数来移动机器人")
            return True
        except Exception as e:
            print(f"[Error] 启动建图失败: {e}")
            return False
    
    def save_map(self, map_name='my_map'):
        """
        保存当前地图
        :param map_name: str, 地图文件名（不含扩展名）
        """
        print(f"[SLAM] 保存地图: {map_name}")
        try:
            # 使用 map_saver 保存地图
            # 地图会保存到当前目录或指定的地图目录
            cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_name]
            result = subprocess.run(cmd, timeout=10.0, capture_output=True)
            
            if result.returncode == 0:
                print(f"[SLAM] 地图已保存: {map_name}.pgm 和 {map_name}.yaml")
                return True
            else:
                print(f"[Error] 保存地图失败: {result.stderr.decode()}")
                return False
                
        except Exception as e:
            print(f"[Error] 保存地图失败: {e}")
            return False
    
    def start_navigation(self, map_file=None):
        """
        启动导航功能
        :param map_file: str, 地图文件路径（.yaml文件）
        """
        if self.nav_process:
            print("[Warning] 导航已在运行")
            return False
        
        print("[Nav] 启动导航...")
        try:
            cmd = ["ros2", "launch", "wheeltec_nav2", "wheeltec_nav2.launch.py"]
            if map_file:
                cmd.extend([f"map:={map_file}"])
            
            self.nav_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            time.sleep(8)  # 等待导航栈初始化
            
            if self.nav_process.poll() is not None:
                print("[Error] 导航启动失败")
                self.nav_process = None
                return False
            
            print("[Nav] 导航已启动")
            print("[Nav] 提示：使用 move_to_goal(x, y, theta) 发送导航目标")
            return True
        except Exception as e:
            print(f"[Error] 启动导航失败: {e}")
            return False
    
    def move_to_goal(self, x, y, theta=0.0):
        """
        发送导航目标点
        :param x: float, 目标X坐标，单位米
        :param y: float, 目标Y坐标，单位米  
        :param theta: float, 目标朝向，单位度
        """
        print(f"[Nav] 导航到目标: x={x}m, y={y}m, theta={theta}°")
        
        try:
            # 转换角度为四元数
            theta_rad = math.radians(theta)
            qz = math.sin(theta_rad / 2.0)
            qw = math.cos(theta_rad / 2.0)
            
            # 构建目标点消息
            goal_msg = {
                "pose": {
                    "position": {"x": float(x), "y": float(y), "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": float(qz), "w": float(qw)}
                }
            }
            
            # 发布导航目标
            cmd = [
                "ros2", "topic", "pub", "--once",
                "/goal_pose",
                "geometry_msgs/msg/PoseStamped",
                json.dumps(goal_msg)
            ]
            subprocess.run(cmd, timeout=2.0)
            print("[Nav] 导航目标已发送")
            return True
            
        except Exception as e:
            print(f"[Error] 发送导航目标失败: {e}")
            return False
    
    def cancel_navigation(self):
        """
        取消当前导航任务
        """
        print("[Nav] 取消导航")
        try:
            # 发送取消导航的指令
            # 通常通过调用导航的取消服务或发送空目标
            self.emergency_stop()
            print("[Nav] 导航已取消")
            return True
        except Exception as e:
            print(f"[Error] 取消导航失败: {e}")
            return False

    # ==================== 辅助函数 ====================
    
    def _get_odom_position(self):
        """
        内部函数：从里程计获取当前位置
        返回: (x, y) 或 None
        """
        try:
            cmd = ["ros2", "topic", "echo", "/odom", "--once", "--field", "pose.pose.position"]
            output = subprocess.check_output(cmd, timeout=1.0).decode("utf-8")
            
            # 解析位置信息
            lines = output.strip().split('\n')
            x, y = 0.0, 0.0
            for line in lines:
                if 'x:' in line:
                    x = float(line.split(':')[1].strip())
                elif 'y:' in line:
                    y = float(line.split(':')[1].strip())
            
            return (x, y)
        except:
            return None
    
    def _get_odom_yaw(self):
        """
        内部函数：从里程计获取当前朝向角
        返回: yaw (弧度) 或 None
        """
        try:
            cmd = ["ros2", "topic", "echo", "/odom", "--once", "--field", "pose.pose.orientation"]
            output = subprocess.check_output(cmd, timeout=1.0).decode("utf-8")
            
            # 解析四元数
            lines = output.strip().split('\n')
            qz, qw = 0.0, 1.0
            for line in lines:
                if 'z:' in line:
                    qz = float(line.split(':')[1].strip())
                elif 'w:' in line:
                    qw = float(line.split(':')[1].strip())
            
            # 四元数转欧拉角
            yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
            return yaw
        except:
            return None