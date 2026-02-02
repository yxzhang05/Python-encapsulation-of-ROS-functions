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