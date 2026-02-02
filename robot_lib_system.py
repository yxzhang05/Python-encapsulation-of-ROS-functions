# 文件名: robot_lib_system.py
# 功能：ROS2 系统管理功能的 Python 封装
# 作者：自动生成
# 日期：2026-02-02

import subprocess
import time
import signal
import os
import sys

class RobotSystem:
    """
    机器人系统管理类
    
    该类封装了 ROS2 机器人系统的基础管理功能，包括：
    - 系统初始化与关闭
    - 电池电压监控
    - 急停功能
    - 软件版本查询
    
    所有函数都通过调用 ROS2 底层命令或话题来实现，
    用户无需了解 ROS2 的具体操作即可使用这些功能。
    """
    
    def __init__(self):
        """
        构造函数：初始化系统管理类的内部状态
        
        成员变量：
        - driver_process: 底盘驱动进程句柄，用于管理 ROS2 launch 启动的进程
        - robot_type: 当前机器人的车型（akm/diff/mec）
        - is_initialized: 标记系统是否已初始化
        """
        self.driver_process = None  # 底盘驱动进程句柄
        self.robot_type = None      # 当前机器人车型
        self.is_initialized = False # 初始化状态标志
        
        # 定义车型映射关系
        # 键是用户传入的简写，值是 ROS2 launch 文件实际需要的参数
        self.ROBOT_TYPE_MAP = {
            "akm": "ackermann",    # 阿克曼转向车型
            "diff": "diff",        # 差速转向车型
            "mec": "mecanum"       # 麦克纳姆轮（麦轮）车型
        }
    
    def initialize(self, robot_type):
        """
        初始化机器人系统
        
        功能说明：
        该函数是整个机器人系统的入口，负责启动底盘驱动、TF变换和底盘控制器。
        
        封装原理：
        1. 通过 subprocess.Popen() 在后台启动 ROS2 launch 文件
        2. launch 文件会启动串口驱动、TF变换、底盘控制器等核心节点
        3. 使用 DEVNULL 屏蔽标准输出，保持控制台清爽
        4. 保留 stderr 以便调试时查看错误信息
        5. 等待 5 秒让硬件（雷达/IMU/串口）完全初始化
        6. 检查进程状态确保启动成功
        
        参数：
        - robot_type (str): 车型枚举，支持 "akm"（阿克曼）、"diff"（差速）、"mec"（麦轮）
        
        返回值：
        - bool: 初始化成功返回 True，失败返回 False
        
        使用示例：
        >>> robot = RobotSystem()
        >>> if robot.initialize("mec"):
        >>>     print("初始化成功")
        """
        # 1. 参数合法性检查
        if robot_type not in self.ROBOT_TYPE_MAP:
            print(f"[错误] 未知的车型: {robot_type}")
            print(f"[提示] 支持的车型: {list(self.ROBOT_TYPE_MAP.keys())}")
            return False
        
        # 如果已经初始化过，先关闭旧进程
        if self.is_initialized:
            print("[警告] 系统已初始化，正在重新初始化...")
            self.shutdown()
        
        # 2. 获取实际的 ROS2 参数值
        real_type_name = self.ROBOT_TYPE_MAP[robot_type]
        self.robot_type = robot_type
        print(f"[系统] 正在启动 {real_type_name} ({robot_type}) 底盘驱动...")
        
        # 3. 构建 ROS2 launch 命令
        # 命令格式: ros2 launch <package_name> <launch_file> <parameters>
        # 这里启动 Wheeltec 机器人的主驱动 launch 文件
        cmd = [
            "ros2", "launch",                              # ROS2 启动命令
            "turn_on_wheeltec_robot",                      # 功能包名称
            "turn_on_wheeltec_robot.launch.py",            # launch 文件名
            f"robot_type:={real_type_name}"                # 传递车型参数
        ]
        
        try:
            # 4. 启动后台进程
            # stdout=DEVNULL: 不显示标准输出，保持终端清爽
            # stderr=PIPE: 保留错误输出用于调试
            self.driver_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 5. 等待硬件初始化
            # 串口、雷达、IMU 等硬件需要时间初始化，强制等待确保稳定
            print("[系统] 正在初始化硬件，请等待 5 秒...")
            time.sleep(5)
            
            # 6. 检查进程状态
            # poll() 返回 None 表示进程仍在运行
            # 如果返回值不是 None，说明进程已退出（启动失败）
            if self.driver_process.poll() is not None:
                print("[错误] 驱动启动失败！进程异常退出。")
                print("[提示] 请检查：")
                print("  1. ROS2 环境是否正确配置")
                print("  2. 功能包是否正确编译")
                print("  3. 硬件（串口/雷达）是否正确连接")
                return False
            
            # 7. 标记初始化成功
            self.is_initialized = True
            print(f"[系统] {real_type_name} 底盘初始化完成！")
            return True
            
        except FileNotFoundError:
            print("[错误] 找不到 ros2 命令！")
            print("[提示] 请确保已经安装 ROS2 并执行了 source 命令")
            return False
        except Exception as e:
            print(f"[错误] 无法启动驱动: {e}")
            return False
    
    def shutdown(self):
        """
        安全关闭机器人系统
        
        功能说明：
        该函数负责停止所有运动、关闭传感器并终止所有 ROS2 进程。
        
        封装原理：
        1. 首先发送速度为 0 的命令让机器人停止运动（急停）
        2. 向驱动进程发送 SIGINT 信号（相当于 Ctrl+C）优雅关闭
        3. 等待进程自行退出，超时则强制 kill
        4. 清理进程句柄，重置初始化状态
        
        参数：
        - 无
        
        返回值：
        - 无
        
        使用示例：
        >>> robot = RobotSystem()
        >>> robot.initialize("mec")
        >>> # ... 使用机器人 ...
        >>> robot.shutdown()  # 安全关闭
        """
        print("[系统] 正在关闭机器人系统...")
        
        # 1. 先执行急停，确保机器人停止运动
        if self.is_initialized:
            self.emergency_stop()
        
        # 2. 关闭驱动进程
        if self.driver_process:
            try:
                # 发送 SIGINT 信号（Ctrl+C），让 ROS2 优雅退出
                self.driver_process.send_signal(signal.SIGINT)
                
                # 等待进程退出，最多等待 5 秒
                self.driver_process.wait(timeout=5)
                print("[系统] 底盘驱动已安全关闭。")
                
            except subprocess.TimeoutExpired:
                # 如果 5 秒后还没退出，强制杀掉进程
                print("[警告] 进程未能正常退出，强制终止...")
                self.driver_process.kill()
                print("[系统] 底盘驱动已强制关闭。")
            
            except Exception as e:
                print(f"[错误] 关闭进程时出错: {e}")
            
            finally:
                # 清理进程句柄
                self.driver_process = None
        
        # 3. 重置初始化状态
        self.is_initialized = False
        self.robot_type = None
        print("[系统] 系统已完全关闭。")
    
    def get_battery_voltage(self):
        """
        获取当前底盘电池电压
        
        功能说明：
        该函数用于实时监控电池电压，可用于实现低电量报警功能。
        
        封装原理：
        1. 使用 ros2 topic echo 命令订阅电压话题
        2. --once 参数表示只接收一条消息就退出
        3. --field data 表示只提取消息的 data 字段
        4. 设置 2 秒超时防止话题无数据时程序卡死
        5. 解析命令输出，转换为浮点数返回
        
        ROS2 话题说明：
        - 话题名称: /PowerVoltage (Wheeltec 机器人的标准电压话题)
        - 消息类型: std_msgs/Float32
        - 数据字段: data (浮点数，单位：伏特)
        
        参数：
        - 无
        
        返回值：
        - float: 电池电压值（单位：伏特），获取失败返回 0.0
        
        使用示例：
        >>> robot = RobotSystem()
        >>> robot.initialize("mec")
        >>> voltage = robot.get_battery_voltage()
        >>> print(f"当前电压: {voltage}V")
        >>> if voltage < 10.5:
        >>>     print("警告：电量过低，请充电！")
        """
        # 检查系统是否已初始化
        if not self.is_initialized:
            print("[警告] 系统未初始化，无法获取电压")
            return 0.0
        
        # Wheeltec 机器人的标准电压话题名称
        topic_name = "/PowerVoltage"
        
        try:
            # 构建 ROS2 命令：订阅话题并获取一次数据
            cmd = [
                "ros2", "topic", "echo",  # ROS2 话题回显命令
                topic_name,               # 话题名称
                "--once",                 # 只接收一条消息
                "--field", "data"         # 只提取 data 字段
            ]
            
            # 执行命令并获取输出
            # timeout=2.0: 设置 2 秒超时，防止话题无数据时卡死
            output = subprocess.check_output(
                cmd, 
                timeout=2.0,
                stderr=subprocess.DEVNULL  # 屏蔽警告信息
            ).decode("utf-8")
            
            # 清理输出字符串并转换为浮点数
            voltage = float(output.strip())
            
            return voltage
            
        except subprocess.TimeoutExpired:
            print(f"[警告] 获取电压超时 (话题 {topic_name} 可能无数据)")
            print("[提示] 请检查：")
            print("  1. 底盘是否已上电")
            print("  2. 驱动是否正常运行")
            return 0.0
            
        except ValueError:
            print(f"[错误] 无法解析电压数据: {output}")
            return 0.0
            
        except Exception as e:
            print(f"[错误] 获取电压失败: {e}")
            return 0.0
    
    def emergency_stop(self):
        """
        软件急停
        
        功能说明：
        立即发送全 0 速度指令，让机器人紧急停止运动。
        
        封装原理：
        1. 使用 ros2 topic pub 命令发布速度消息
        2. --once 参数表示只发布一次就退出
        3. 消息类型为 geometry_msgs/msg/Twist
        4. 所有速度分量都设为 0（线速度和角速度）
        
        ROS2 话题说明：
        - 话题名称: /cmd_vel (底盘速度控制话题)
        - 消息类型: geometry_msgs/msg/Twist
        - 消息结构:
            linear:  {x: 0.0, y: 0.0, z: 0.0}  # 线速度
            angular: {x: 0.0, y: 0.0, z: 0.0}  # 角速度
        
        参数：
        - 无
        
        返回值：
        - 无
        
        使用示例：
        >>> robot = RobotSystem()
        >>> robot.initialize("mec")
        >>> # 检测到危险情况
        >>> robot.emergency_stop()  # 立即停止
        """
        # 检查系统是否已初始化
        if not self.is_initialized:
            print("[警告] 系统未初始化，无法执行急停")
            return
        
        print("[系统] 执行急停...")
        
        # 构建全 0 速度的 Twist 消息
        # Twist 消息包含线速度(linear)和角速度(angular)
        cmd = [
            "ros2", "topic", "pub",           # ROS2 话题发布命令
            "--once",                         # 只发布一次
            "/cmd_vel",                       # 速度控制话题
            "geometry_msgs/msg/Twist",        # 消息类型
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ]
        
        try:
            # 执行命令
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0  # 1秒超时
            )
            print("[系统] 急停命令已发送")
            
        except subprocess.TimeoutExpired:
            print("[警告] 急停命令发送超时")
        except Exception as e:
            print(f"[错误] 急停失败: {e}")
    
    def get_software_version(self):
        """
        获取下位机软件或固件的版本号
        
        功能说明：
        该函数用于查询底盘控制器的固件版本，便于后续版本更新和问题排查。
        
        封装原理：
        1. 尝试订阅版本信息话题（如果存在）
        2. 如果话题不存在，尝试从参数服务器获取
        3. 如果都不存在，返回 ROS2 系统版本作为参考
        
        注意：
        Wheeltec 部分底盘可能没有专门的版本话题，需要通过串口协议查询。
        如果底层不支持，该函数将返回 "Unknown" 或 ROS2 版本号。
        
        参数：
        - 无
        
        返回值：
        - str: 版本号字符串，获取失败返回 "Unknown"
        
        使用示例：
        >>> robot = RobotSystem()
        >>> robot.initialize("mec")
        >>> version = robot.get_software_version()
        >>> print(f"固件版本: {version}")
        """
        # 检查系统是否已初始化
        if not self.is_initialized:
            print("[警告] 系统未初始化，无法获取版本")
            return "Unknown"
        
        # 尝试方法1：从话题获取版本信息
        # 注意：Wheeltec 部分底盘可能没有这个话题
        version_topic = "/firmware_version"
        
        try:
            cmd = [
                "ros2", "topic", "echo",
                version_topic,
                "--once",
                "--field", "data"
            ]
            
            output = subprocess.check_output(
                cmd,
                timeout=1.0,
                stderr=subprocess.DEVNULL
            ).decode("utf-8").strip()
            
            if output:
                return output
                
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            pass  # 话题不存在，尝试其他方法
        
        # 尝试方法2：从参数服务器获取
        try:
            cmd = [
                "ros2", "param", "get",
                "/wheeltec_robot",  # 节点名称
                "firmware_version"   # 参数名称
            ]
            
            output = subprocess.check_output(
                cmd,
                timeout=1.0,
                stderr=subprocess.DEVNULL
            ).decode("utf-8").strip()
            
            if "String value is:" in output:
                # 解析输出，提取版本号
                version = output.split("String value is:")[-1].strip()
                return version
                
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            pass  # 参数不存在
        
        # 方法3：返回 ROS2 版本作为参考
        try:
            cmd = ["ros2", "--version"]
            output = subprocess.check_output(
                cmd,
                stderr=subprocess.DEVNULL
            ).decode("utf-8").strip()
            return f"ROS2 {output}"
        except:
            pass
        
        # 所有方法都失败，返回未知
        print("[提示] 无法获取固件版本，底盘可能不支持该功能")
        return "Unknown"
    
    def __del__(self):
        """
        析构函数：对象销毁时自动调用
        
        确保即使用户忘记调用 shutdown()，
        程序退出时也能安全关闭所有进程。
        """
        if self.driver_process:
            self.shutdown()


# 模块测试代码
if __name__ == "__main__":
    """
    模块测试：直接运行此文件可以测试系统管理功能
    """
    print("=" * 60)
    print("ROS2 系统管理功能测试")
    print("=" * 60)
    
    # 创建机器人系统对象
    robot = RobotSystem()
    
    try:
        # 测试1：初始化系统
        print("\n[测试1] 初始化系统...")
        success = robot.initialize("mec")  # 使用麦轮车型
        
        if not success:
            print("初始化失败，测试终止")
            sys.exit(1)
        
        # 测试2：获取电池电压
        print("\n[测试2] 获取电池电压...")
        voltage = robot.get_battery_voltage()
        print(f"当前电池电压: {voltage:.2f} V")
        
        if voltage > 0 and voltage < 10.5:
            print("⚠️  警告：电量过低，请充电！")
        
        # 测试3：获取软件版本
        print("\n[测试3] 获取软件版本...")
        version = robot.get_software_version()
        print(f"软件版本: {version}")
        
        # 测试4：急停功能
        print("\n[测试4] 测试急停功能...")
        robot.emergency_stop()
        
        # 等待用户查看结果
        print("\n测试完成！按 Ctrl+C 退出...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    
    finally:
        # 测试5：安全关闭
        print("\n[测试5] 安全关闭系统...")
        robot.shutdown()
        print("\n所有测试完成！")
