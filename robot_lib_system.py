# 文件名: robot_lib_system.py
import subprocess
import time
import signal
import os

class Robot:
    def __init__(self):
        self.driver_process = None  # 底盘驱动进程句柄
        
        # 定义车型映射关系
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
        # 检查参数合法性
        if robot_type not in self.ROBOT_TYPE_MAP:
            print(f"[Error] 未知的车型: {robot_type}. 支持: {list(self.ROBOT_TYPE_MAP.keys())}")
            return False

        # 获取实际的 ROS 参数值
        real_type_name = self.ROBOT_TYPE_MAP[robot_type]
        print(f"[System] 正在启动 {real_type_name} ({robot_type}) 底盘驱动...")

        # 构建命令
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
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 强制等待硬件初始化
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

    def shutdown(self):
        """
        关闭所有子进程，安全退出
        """
        print("[System] 正在关闭机器人系统...")
        
        if self.driver_process:
            # 发送 SIGINT 信号
            self.driver_process.send_signal(signal.SIGINT)
            try:
                self.driver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.driver_process.kill()
            print("[System] 底盘驱动已关闭。")
            self.driver_process = None

    def get_battery_voltage(self):
        """
        获取当前电池电压
        :return: float, 电压值（伏特），获取失败返回 0.0
        """
        topic_name = "/PowerVoltage"
        
        try:
            cmd = ["ros2", "topic", "echo", topic_name, "--once", "--field", "data"]
            output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
            voltage = float(output.strip())
            return voltage
            
        except subprocess.TimeoutExpired:
            print("[Warning] 获取电压超时")
            return 0.0
        except Exception as e:
            print(f"[Error] 获取电压失败: {e}")
            return 0.0

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
