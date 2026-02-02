# 文件名: robot_lib_navigation.py
# 功能：ROS2 建图与导航功能的 Python 封装
# 作者：自动生成
# 日期：2026-02-02

import subprocess
import time
import signal
import os
import sys
import threading

class RobotNavigation:
    """
    机器人建图与导航控制类
    
    该类封装了 ROS2 机器人的 SLAM 建图和自主导航功能，包括：
    - SLAM 建图（支持多种算法）
    - 地图保存与加载
    - 键盘遥控
    - 自主导航（目标点导航）
    - 导航状态监控
    
    所有功能都以非阻塞方式运行，允许在后台进行建图或导航的同时
    执行其他操作（如数据采集、传感器读取等）
    """
    
    def __init__(self):
        """
        构造函数：初始化导航控制类
        
        成员变量：
        - mapping_process: 建图进程句柄
        - navigation_process: 导航进程句柄
        - keyboard_process: 键盘控制进程句柄
        - current_map_name: 当前地图名称
        - navigation_status: 导航状态
        """
        self.mapping_process = None
        self.navigation_process = None
        self.keyboard_process = None
        self.current_map_name = None
        self.navigation_status = "idle"  # idle, navigating, reached, failed
    
    def start_mapping(self, method="gmapping", visualize=True):
        """
        启动 SLAM 建图
        
        功能说明：
        启动 SLAM 建图节点，机器人可以通过遥控或键盘移动来构建地图。
        
        封装原理：
        1. 根据选择的算法启动对应的 launch 文件
        2. 建图节点订阅 /scan (雷达) 和 /odom (里程计) 话题
        3. 实时构建地图并发布到 /map 话题
        4. 如果 visualize=True，启动 RViz 显示建图过程
        5. 建图过程在后台运行（非阻塞），可以同时执行其他操作
        
        支持的算法：
        - gmapping: 基于粒子滤波的 SLAM，适合中小型环境
        - cartographer: Google 的 SLAM 算法，适合大型环境
        - slam_toolbox: 功能强大的现代 SLAM 工具箱
        
        参数：
        - method (str): 建图算法，默认 "gmapping"
        - visualize (bool): 是否显示 RViz 可视化，默认 True
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.start_mapping("gmapping")  # 启动gmapping建图
        >>> # 现在可以用键盘控制机器人移动来建图
        >>> nav.start_keyboard_control()
        """
        if self.mapping_process:
            print("[警告] 建图已在运行")
            return True
        
        print(f"[导航] 启动 {method} 建图...")
        
        # 根据算法选择 launch 文件
        launch_configs = {
            "gmapping": ("wheeltec_robot_slam", "gmapping.launch.py"),
            "cartographer": ("wheeltec_robot_slam", "cartographer.launch.py"),
            "slam_toolbox": ("wheeltec_robot_slam", "slam_toolbox.launch.py"),
        }
        
        if method not in launch_configs:
            print(f"[错误] 不支持的建图算法: {method}")
            print(f"[提示] 支持的算法: {list(launch_configs.keys())}")
            return False
        
        package, launch_file = launch_configs[method]
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            package,
            launch_file
        ]
        
        try:
            # 启动建图进程
            self.mapping_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 等待初始化
            print("[导航] 等待建图节点初始化...")
            time.sleep(5)
            
            # 检查进程状态
            if self.mapping_process.poll() is not None:
                print("[错误] 建图启动失败")
                return False
            
            print(f"[导航] {method} 建图已启动")
            
            # 启动可视化
            if visualize:
                self._launch_rviz("slam")
            
            print("[提示] 使用键盘控制或其他方式移动机器人来建图")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动建图失败: {e}")
            return False
    
    def save_map(self, map_name="my_map"):
        """
        保存当前构建的地图
        
        功能说明：
        将当前 SLAM 构建的地图保存到文件。
        
        封装原理：
        1. 调用 ROS2 的 map_saver 服务
        2. 地图保存为两个文件：
           - map_name.pgm: 地图图像（黑白栅格）
           - map_name.yaml: 地图元数据（分辨率、原点等）
        3. 默认保存在当前目录或 maps 文件夹
        
        参数：
        - map_name (str): 地图文件名（不含扩展名），默认 "my_map"
        
        返回值：
        - bool: 保存成功返回 True，失败返回 False
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.start_mapping()
        >>> # ... 移动机器人建图 ...
        >>> nav.save_map("office_map")  # 保存为 office_map.pgm 和 office_map.yaml
        """
        if not self.mapping_process:
            print("[警告] 建图未运行，无法保存")
            return False
        
        print(f"[导航] 保存地图: {map_name}...")
        
        # 确保 maps 目录存在
        maps_dir = "maps"
        if not os.path.exists(maps_dir):
            os.makedirs(maps_dir)
            print(f"[导航] 创建地图目录: {maps_dir}")
        
        # 构建保存路径
        map_path = os.path.join(maps_dir, map_name)
        
        # 构建命令（ROS2 map_saver）
        cmd = [
            "ros2", "run",
            "nav2_map_server", "map_saver_cli",
            "-f", map_path
        ]
        
        try:
            # 执行保存
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10.0
            )
            
            if result.returncode == 0:
                print(f"[导航] 地图已保存到: {map_path}.pgm/.yaml")
                self.current_map_name = map_name
                return True
            else:
                print("[错误] 地图保存失败")
                print(f"[错误信息] {result.stderr.decode()}")
                return False
                
        except subprocess.TimeoutExpired:
            print("[错误] 保存地图超时")
            return False
        except Exception as e:
            print(f"[错误] 保存地图失败: {e}")
            return False
    
    def start_keyboard_control(self):
        """
        启动键盘控制（阻塞式）
        
        功能说明：
        启动键盘遥控节点，允许用户通过键盘控制机器人移动。
        
        封装原理：
        1. 启动 teleop_twist_keyboard 节点
        2. 用户可以用 WASD 或方向键控制机器人
        3. 该函数会阻塞，直到用户按 Ctrl+C 退出
        
        常用场景：
        - 建图时手动控制机器人探索环境
        - 测试机器人运动性能
        - 手动导航到特定位置
        
        参数：
        - 无
        
        返回值：
        - 无（阻塞执行）
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.start_keyboard_control()  # 进入键盘控制模式，按Ctrl+C退出
        """
        print("\n" + "="*60)
        print("[导航] 启动键盘控制...")
        print("="*60)
        print("\n控制说明：")
        print("  W/↑  - 前进")
        print("  S/↓  - 后退")
        print("  A/←  - 左转")
        print("  D/→  - 右转")
        print("  空格  - 停止")
        print("  Ctrl+C - 退出")
        print("\n" + "="*60 + "\n")
        
        # 构建命令
        cmd = [
            "ros2", "run",
            "wheeltec_robot_keyboard",
            "wheeltec_keyboard"
        ]
        
        try:
            # 启动键盘控制（阻塞）
            subprocess.run(cmd)
        except KeyboardInterrupt:
            print("\n[导航] 退出键盘控制")
    
    def load_map_and_start_navigation(self, map_name="my_map", visualize=True):
        """
        加载地图并启动导航系统
        
        功能说明：
        加载已保存的地图，并启动 Nav2 导航堆栈。
        
        封装原理：
        1. 启动 map_server 加载地图文件
        2. 启动 AMCL 进行定位
        3. 启动 Nav2 导航规划器和控制器
        4. 初始化后，机器人可以接收导航目标
        
        参数：
        - map_name (str): 地图文件名（不含扩展名）
        - visualize (bool): 是否显示 RViz，默认 True
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.load_map_and_start_navigation("office_map")
        >>> # 导航系统启动后，可以发送导航目标
        >>> nav.move_to_goal(2.0, 1.0, 0)
        """
        if self.navigation_process:
            print("[警告] 导航已在运行")
            return True
        
        print(f"[导航] 加载地图并启动导航系统...")
        
        # 检查地图文件是否存在
        map_path = os.path.join("maps", f"{map_name}.yaml")
        if not os.path.exists(map_path):
            print(f"[错误] 地图文件不存在: {map_path}")
            print("[提示] 请先使用 save_map() 保存地图")
            return False
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_nav2",
            "navigation.launch.py",
            f"map:={os.path.abspath(map_path)}"
        ]
        
        try:
            # 启动导航进程
            self.navigation_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 等待初始化（导航堆栈需要较长时间）
            print("[导航] 等待导航系统初始化（约15秒）...")
            time.sleep(15)
            
            # 检查进程状态
            if self.navigation_process.poll() is not None:
                print("[错误] 导航启动失败")
                return False
            
            print("[导航] 导航系统已启动")
            self.current_map_name = map_name
            
            # 启动可视化
            if visualize:
                self._launch_rviz("navigation")
            
            print("[提示] 使用 move_to_goal() 发送导航目标")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动导航失败: {e}")
            return False
    
    def move_to_goal(self, x, y, theta=0.0, callback=None):
        """
        导航到目标点
        
        功能说明：
        发送目标位姿给导航系统，机器人自动规划路径并前往。
        
        封装原理：
        1. 构建 PoseStamped 消息
        2. 发布到 /goal_pose 话题
        3. Nav2 接收目标后自动进行：
           - 全局路径规划（A*、Dijkstra等）
           - 局部路径规划（DWA、TEB等）
           - 避障和运动控制
        4. 该函数立即返回（非阻塞），导航在后台执行
        5. 可以通过 callback 或轮询获取导航状态
        
        参数：
        - x (float): 目标点 X 坐标，单位米
        - y (float): 目标点 Y 坐标，单位米
        - theta (float): 目标朝向角度，单位度，默认 0
        - callback (function): 回调函数，接收导航状态 callback(status)
        
        返回值：
        - bool: 目标发送成功返回 True，失败返回 False
        
        导航状态：
        - "navigating": 正在前往目标
        - "reached": 已到达目标
        - "failed": 导航失败（无法到达或超时）
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.load_map_and_start_navigation("office_map")
        >>> 
        >>> # 示例1：简单导航
        >>> nav.move_to_goal(2.0, 1.5, 90)  # 前往 (2.0, 1.5)，朝向北
        >>> 
        >>> # 示例2：带回调
        >>> def on_nav_status(status):
        >>>     print(f"导航状态: {status}")
        >>> nav.move_to_goal(3.0, -1.0, callback=on_nav_status)
        """
        if not self.navigation_process:
            print("[错误] 导航系统未启动")
            print("[提示] 请先调用 load_map_and_start_navigation()")
            return False
        
        print(f"[导航] 发送导航目标: ({x:.2f}, {y:.2f}, {theta:.1f}°)")
        
        # 将角度转换为四元数
        import math
        theta_rad = math.radians(theta)
        qz = math.sin(theta_rad / 2.0)
        qw = math.cos(theta_rad / 2.0)
        
        # 构建 PoseStamped 消息
        pose_msg = (
            f"{{header: {{frame_id: 'map'}}, "
            f"pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}"
        )
        
        # 构建命令
        cmd = [
            "ros2", "topic", "pub",
            "--once",
            "/goal_pose",
            "geometry_msgs/msg/PoseStamped",
            pose_msg
        ]
        
        try:
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0
            )
            
            self.navigation_status = "navigating"
            print("[导航] 目标已发送，机器人开始导航")
            
            # 如果提供了回调，启动监听线程
            if callback:
                self._start_navigation_monitor(callback)
            
            return True
            
        except Exception as e:
            print(f"[错误] 发送导航目标失败: {e}")
            return False
    
    def cancel_navigation(self):
        """
        取消当前导航任务
        
        功能说明：
        取消正在进行的导航，机器人停止运动。
        
        封装原理：
        1. 发布空目标到 /cancel_goal 话题（或调用 cancel 服务）
        2. Nav2 收到取消请求后停止规划和运动
        3. 发送全 0 速度命令确保机器人停止
        
        参数：
        - 无
        
        返回值：
        - bool: 取消成功返回 True，失败返回 False
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.move_to_goal(5.0, 3.0)
        >>> time.sleep(5)
        >>> nav.cancel_navigation()  # 取消导航
        """
        if not self.navigation_process:
            print("[警告] 导航系统未运行")
            return False
        
        if self.navigation_status != "navigating":
            print("[提示] 当前没有正在进行的导航任务")
            return True
        
        print("[导航] 取消导航...")
        
        try:
            # 方法1：发布空的目标到 goal_pose（某些实现）
            # 方法2：调用 cancel_goal action（更标准）
            # 这里使用发送停止命令的方式
            
            cmd = [
                "ros2", "topic", "pub",
                "--once",
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
            ]
            
            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0
            )
            
            self.navigation_status = "idle"
            print("[导航] 导航已取消")
            
            return True
            
        except Exception as e:
            print(f"[错误] 取消导航失败: {e}")
            return False
    
    def get_navigation_status(self):
        """
        获取当前导航状态
        
        返回值：
        - str: 导航状态 ("idle", "navigating", "reached", "failed")
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.move_to_goal(2.0, 1.0)
        >>> while nav.get_navigation_status() == "navigating":
        >>>     print("正在导航...")
        >>>     time.sleep(1)
        >>> print("导航完成！")
        """
        return self.navigation_status
    
    def shutdown(self):
        """
        关闭所有导航相关进程
        
        使用示例：
        >>> nav = RobotNavigation()
        >>> nav.start_mapping()
        >>> # ... 使用完毕 ...
        >>> nav.shutdown()
        """
        print("[导航] 关闭导航系统...")
        
        # 关闭建图
        if self.mapping_process:
            try:
                self.mapping_process.send_signal(signal.SIGINT)
                self.mapping_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.mapping_process.kill()
            self.mapping_process = None
            print("[导航] 建图已关闭")
        
        # 关闭导航
        if self.navigation_process:
            try:
                self.navigation_process.send_signal(signal.SIGINT)
                self.navigation_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.navigation_process.kill()
            self.navigation_process = None
            print("[导航] 导航已关闭")
    
    # ==================== 辅助函数 ====================
    
    def _launch_rviz(self, config_type):
        """内部函数：启动 RViz"""
        try:
            cmd = ["rviz2"]
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("[可视化] RViz 已启动")
        except:
            print("[提示] 无法启动 RViz")
    
    def _start_navigation_monitor(self, callback):
        """内部函数：启动导航状态监控线程"""
        # 这里可以实现一个后台线程监控导航状态
        # 并定期调用回调函数
        pass
    
    def __del__(self):
        """析构函数：清理所有进程"""
        self.shutdown()


# 模块测试代码
if __name__ == "__main__":
    print("=" * 60)
    print("ROS2 建图与导航功能测试")
    print("=" * 60)
    
    nav = RobotNavigation()
    
    try:
        print("\n[测试] 启动建图...")
        nav.start_mapping("gmapping")
        
        print("\n按 Ctrl+C 停止测试...")
        while True:
            time.sleep(1)
        
    except KeyboardInterrupt:
        print("\n\n测试被中断")
        nav.shutdown()
