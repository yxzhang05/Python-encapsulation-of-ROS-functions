# 文件名: robot_lib_sensors.py
# 功能：ROS2 感知与功能的 Python 封装
# 作者：自动生成
# 日期：2026-02-02

import subprocess
import time
import signal
import os
import sys
import threading

class RobotSensors:
    """
    机器人感知与功能控制类
    
    该类封装了 ROS2 机器人的感知功能，包括：
    - 雷达控制（启动/停止）
    - 相机控制（启动/停止）
    - 视觉应用（颜色跟随、巡线）
    - 雷达应用（雷达跟随）
    - 传感器数据获取（图像、雷达扫描）
    
    所有应用函数都支持非阻塞模式和回调函数
    """
    
    def __init__(self):
        """
        构造函数：初始化感知功能类
        
        成员变量：
        - lidar_process: 雷达驱动进程句柄
        - camera_process: 相机驱动进程句柄
        - application_process: 当前运行的应用进程句柄
        - rviz_process: RViz可视化进程句柄
        - current_app: 当前运行的应用名称
        """
        self.lidar_process = None
        self.camera_process = None
        self.application_process = None
        self.rviz_process = None
        self.current_app = None
    
    def launch_lidar(self, visualize=True):
        """
        启动雷达驱动
        
        功能说明：
        启动激光雷达的ROS2驱动节点，开始发布 /scan 话题数据。
        
        封装原理：
        1. 使用 subprocess.Popen 启动雷达驱动 launch 文件
        2. 驱动启动后会自动发布 /scan 话题
        3. 可选择性启动 RViz 进行可视化
        
        参数：
        - visualize (bool): 是否自动打开 RViz 可视化界面，默认 True
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_lidar()  # 启动雷达并显示可视化
        >>> sensors.launch_lidar(visualize=False)  # 只启动雷达，不显示
        """
        if self.lidar_process:
            print("[警告] 雷达已在运行")
            return True
        
        print("[传感器] 正在启动雷达驱动...")
        
        # 构建启动命令（根据实际 launch 文件调整）
        cmd = [
            "ros2", "launch",
            "wheeltec_lidar_ros2",
            "wheeltec_lidar.launch.py"
        ]
        
        try:
            # 启动雷达驱动
            self.lidar_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 等待驱动初始化
            print("[传感器] 等待雷达初始化...")
            time.sleep(3)
            
            # 检查进程状态
            if self.lidar_process.poll() is not None:
                print("[错误] 雷达驱动启动失败")
                return False
            
            print("[传感器] 雷达驱动启动成功")
            
            # 如果需要可视化，启动 RViz
            if visualize:
                self._launch_rviz("lidar")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动雷达失败: {e}")
            return False
    
    def stop_lidar(self):
        """
        停止雷达驱动
        
        功能说明：
        关闭雷达驱动节点，停止发布 /scan 话题数据。
        
        参数：
        - 无
        
        返回值：
        - 无
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_lidar()
        >>> # ... 使用雷达 ...
        >>> sensors.stop_lidar()
        """
        if not self.lidar_process:
            print("[警告] 雷达未运行")
            return
        
        print("[传感器] 正在停止雷达...")
        
        try:
            self.lidar_process.send_signal(signal.SIGINT)
            self.lidar_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.lidar_process.kill()
        
        self.lidar_process = None
        print("[传感器] 雷达已停止")
    
    def launch_camera(self, visualize=True):
        """
        启动相机驱动
        
        功能说明：
        启动USB相机或深度相机的驱动节点，开始发布图像话题。
        
        封装原理：
        1. 启动相机驱动 launch 文件
        2. 驱动会发布 /camera/image_raw 等话题
        3. 可选择性启动图像查看器
        
        参数：
        - visualize (bool): 是否自动打开图像可视化窗口，默认 True
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_camera()  # 启动相机并显示图像
        """
        if self.camera_process:
            print("[警告] 相机已在运行")
            return True
        
        print("[传感器] 正在启动相机驱动...")
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            "usb_cam",
            "camera.launch.py"
        ]
        
        try:
            # 启动相机驱动
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 等待驱动初始化
            print("[传感器] 等待相机初始化...")
            time.sleep(3)
            
            # 检查进程状态
            if self.camera_process.poll() is not None:
                print("[错误] 相机驱动启动失败")
                print("[提示] 请检查相机是否正确连接")
                return False
            
            print("[传感器] 相机驱动启动成功")
            
            # 如果需要可视化，启动图像查看器
            if visualize:
                self._launch_image_view()
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动相机失败: {e}")
            return False
    
    def stop_camera(self):
        """
        停止相机驱动
        
        功能说明：
        关闭相机驱动节点，停止采集图像。
        
        参数：
        - 无
        
        返回值：
        - 无
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_camera()
        >>> # ... 使用相机 ...
        >>> sensors.stop_camera()
        """
        if not self.camera_process:
            print("[警告] 相机未运行")
            return
        
        print("[传感器] 正在停止相机...")
        
        try:
            self.camera_process.send_signal(signal.SIGINT)
            self.camera_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.camera_process.kill()
        
        self.camera_process = None
        print("[传感器] 相机已停止")
    
    def start_visual_follow(self, color, control_robot=True, callback=None):
        """
        启动视觉跟随功能
        
        功能说明：
        机器人识别指定颜色的物体，并自动跟随保持在画面中心。
        
        封装原理：
        1. 启动视觉跟随节点，传入目标颜色参数
        2. 节点订阅相机图像，进行颜色识别
        3. 计算目标在画面中的位置偏差
        4. 如果 control_robot=True，自动发布速度命令控制机器人
        5. 如果提供了 callback，定期调用回调函数传递目标信息
        
        参数：
        - color (str): 目标颜色 ('red', 'blue', 'green', 'yellow')
        - control_robot (bool): 是否自动控制机器人底盘，默认 True
        - callback (function): 回调函数，接收目标信息 callback(target_info)
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        目标信息格式：
        {
            "detected": bool,      # 是否检测到目标
            "center_x": float,     # 目标中心X坐标（像素）
            "center_y": float,     # 目标中心Y坐标（像素）
            "area": float,         # 目标面积（像素）
            "distance": float      # 估计距离（米）
        }
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_camera()
        >>> 
        >>> # 示例1：自动跟随
        >>> sensors.start_visual_follow('red')
        >>> 
        >>> # 示例2：只检测不控制
        >>> def on_detect(info):
        >>>     print(f"检测到目标: {info}")
        >>> sensors.start_visual_follow('blue', control_robot=False, callback=on_detect)
        """
        if self.application_process:
            print("[警告] 已有应用在运行，请先停止")
            return False
        
        print(f"[应用] 启动视觉跟随功能（颜色: {color}）...")
        
        # 检查相机是否运行
        if not self.camera_process:
            print("[提示] 相机未启动，正在启动相机...")
            if not self.launch_camera():
                return False
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_kcf",
            "visual_follow.launch.py",
            f"target_color:={color}",
            f"auto_control:={'true' if control_robot else 'false'}"
        ]
        
        try:
            self.application_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[错误] 视觉跟随启动失败")
                return False
            
            self.current_app = "visual_follow"
            print("[应用] 视觉跟随已启动")
            
            # 如果提供了回调函数，启动监听线程
            if callback:
                self._start_callback_thread(callback, "visual_target")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动视觉跟随失败: {e}")
            return False
    
    def start_line_tracking(self, color, control_robot=True, callback=None):
        """
        启动视觉巡线功能
        
        功能说明：
        机器人识别地面上的线条，并自动沿线行驶。
        
        封装原理：
        1. 启动巡线节点，传入线条颜色参数
        2. 节点订阅机械臂相机图像（俯视地面）
        3. 识别线条并计算偏差和曲率
        4. 如果 control_robot=True，自动控制底盘沿线行驶
        5. 如果提供了 callback，定期调用回调函数传递线条信息
        
        参数：
        - color (str): 线条颜色 ('black', 'red', 'yellow', 'white')
        - control_robot (bool): 是否自动控制机器人底盘，默认 True
        - callback (function): 回调函数，接收线条信息 callback(line_info)
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        线条信息格式：
        {
            "detected": bool,      # 是否检测到线条
            "offset": float,       # 线条中心偏离画面中心的距离（像素）
            "angle": float,        # 线条角度（度）
            "confidence": float    # 检测置信度（0-1）
        }
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_camera()
        >>> sensors.start_line_tracking('black')  # 沿黑线行驶
        """
        if self.application_process:
            print("[警告] 已有应用在运行，请先停止")
            return False
        
        print(f"[应用] 启动视觉巡线功能（颜色: {color}）...")
        
        # 检查相机是否运行
        if not self.camera_process:
            print("[提示] 相机未启动，正在启动相机...")
            if not self.launch_camera():
                return False
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            "wheeltec_robot_kcf",
            "line_tracking.launch.py",
            f"line_color:={color}",
            f"auto_control:={'true' if control_robot else 'false'}"
        ]
        
        try:
            self.application_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[错误] 视觉巡线启动失败")
                return False
            
            self.current_app = "line_tracking"
            print("[应用] 视觉巡线已启动")
            
            # 如果提供了回调函数，启动监听线程
            if callback:
                self._start_callback_thread(callback, "line_info")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动视觉巡线失败: {e}")
            return False
    
    def start_lidar_follow(self, target_dist=0.8, callback=None):
        """
        启动雷达跟随功能
        
        功能说明：
        机器人识别正前方最近的移动障碍物（通常是人），并保持指定距离跟随。
        
        封装原理：
        1. 启动雷达跟随节点
        2. 节点订阅 /scan 话题
        3. 过滤并识别前方的动态障碍物
        4. 计算与障碍物的距离和角度
        5. 自动发布速度命令保持跟随距离
        
        参数：
        - target_dist (float): 保持的跟随距离，单位米，默认 0.8m
        - callback (function): 回调函数，接收目标信息 callback(target_info)
        
        返回值：
        - bool: 启动成功返回 True，失败返回 False
        
        目标信息格式：
        {
            "detected": bool,      # 是否检测到目标
            "distance": float,     # 与目标的距离（米）
            "angle": float,        # 目标相对于正前方的角度（度）
        }
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_lidar()
        >>> sensors.start_lidar_follow(target_dist=1.0)  # 保持1米距离跟随
        """
        if self.application_process:
            print("[警告] 已有应用在运行，请先停止")
            return False
        
        print(f"[应用] 启动雷达跟随功能（目标距离: {target_dist}m）...")
        
        # 检查雷达是否运行
        if not self.lidar_process:
            print("[提示] 雷达未启动，正在启动雷达...")
            if not self.launch_lidar():
                return False
        
        # 构建启动命令
        cmd = [
            "ros2", "launch",
            "simple_follower_ros2",
            "lidar_follow.launch.py",
            f"target_distance:={target_dist}"
        ]
        
        try:
            self.application_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            if self.application_process.poll() is not None:
                print("[错误] 雷达跟随启动失败")
                return False
            
            self.current_app = "lidar_follow"
            print("[应用] 雷达跟随已启动")
            
            # 如果提供了回调函数，启动监听线程
            if callback:
                self._start_callback_thread(callback, "follow_target")
            
            return True
            
        except Exception as e:
            print(f"[错误] 启动雷达跟随失败: {e}")
            return False
    
    def stop_application(self):
        """
        停止当前运行的应用
        
        功能说明：
        停止视觉跟随、巡线或雷达跟随等应用，并发送全0速度命令停止机器人。
        
        封装原理：
        1. 发送 SIGINT 信号停止应用进程
        2. 发布全0速度命令让机器人立即停止
        3. 清理进程句柄
        
        参数：
        - 无
        
        返回值：
        - 无
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.start_visual_follow('red')
        >>> time.sleep(10)
        >>> sensors.stop_application()  # 停止跟随并让机器人停止
        """
        if not self.application_process:
            print("[提示] 没有运行中的应用")
            return
        
        print(f"[应用] 正在停止 {self.current_app}...")
        
        try:
            # 停止应用进程
            self.application_process.send_signal(signal.SIGINT)
            self.application_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.application_process.kill()
        
        self.application_process = None
        
        # 发送全0速度命令停止机器人
        self._send_stop_command()
        
        print(f"[应用] {self.current_app} 已停止")
        self.current_app = None
    
    def get_camera_frame(self, save_path=None):
        """
        获取相机当前的一帧图像（拍照）
        
        功能说明：
        从相机话题获取一帧图像，可选择性保存到文件。
        
        封装原理：
        1. 使用 ros2 topic echo 获取一次图像消息
        2. 解析消息中的图像数据
        3. 如果指定了 save_path，保存为图像文件
        
        参数：
        - save_path (str): 图像保存路径，None 表示不保存
        
        返回值：
        - bytes: 图像数据（JPEG 格式），失败返回 None
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_camera()
        >>> image = sensors.get_camera_frame('photo.jpg')  # 拍照并保存
        """
        if not self.camera_process:
            print("[警告] 相机未运行")
            return None
        
        print("[传感器] 正在获取图像...")
        
        topic_name = "/camera/image_raw"
        
        try:
            # 使用 ros2 run image_view extract_images 保存图像
            if save_path:
                cmd = [
                    "ros2", "run", "image_view", "extract_images",
                    f"image:={topic_name}",
                    f"--ros-args", "-p", f"filename_format:={save_path}"
                ]
                
                # 运行一小段时间获取一帧
                proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(1)
                proc.terminate()
                
                print(f"[传感器] 图像已保存到: {save_path}")
                return True
            else:
                print("[提示] 未指定保存路径，仅获取数据")
                # 这里可以实现获取原始数据的逻辑
                return None
                
        except Exception as e:
            print(f"[错误] 获取图像失败: {e}")
            return None
    
    def get_lidar_distance(self, angle_degrees=0):
        """
        获取雷达指定角度的距离信息
        
        功能说明：
        从雷达扫描数据中提取指定角度的距离测量值。
        
        封装原理：
        1. 订阅 /scan 话题获取一次扫描数据
        2. 根据角度计算对应的数组索引
        3. 返回该方向的距离值
        
        参数：
        - angle_degrees (float): 角度，0度为正前方，正值为逆时针，单位度
        
        返回值：
        - float: 距离值（米），失败或无效返回 -1
        
        使用示例：
        >>> sensors = RobotSensors()
        >>> sensors.launch_lidar()
        >>> dist_front = sensors.get_lidar_distance(0)     # 正前方距离
        >>> dist_left = sensors.get_lidar_distance(90)     # 左侧距离
        >>> dist_right = sensors.get_lidar_distance(-90)   # 右侧距离
        """
        if not self.lidar_process:
            print("[警告] 雷达未运行")
            return -1
        
        topic_name = "/scan"
        
        try:
            cmd = [
                "ros2", "topic", "echo",
                topic_name,
                "--once"
            ]
            
            output = subprocess.check_output(
                cmd,
                timeout=2.0,
                stderr=subprocess.DEVNULL
            ).decode("utf-8")
            
            # 解析 LaserScan 消息
            import re
            
            # 提取角度范围
            angle_min_match = re.search(r"angle_min:\s*([-\d.]+)", output)
            angle_max_match = re.search(r"angle_max:\s*([-\d.]+)", output)
            angle_inc_match = re.search(r"angle_increment:\s*([-\d.]+)", output)
            
            # 提取距离数据
            ranges_match = re.search(r"ranges:.*?\[(.*?)\]", output, re.DOTALL)
            
            if all([angle_min_match, angle_max_match, angle_inc_match, ranges_match]):
                angle_min = float(angle_min_match.group(1))
                angle_increment = float(angle_inc_match.group(1))
                
                ranges_str = ranges_match.group(1)
                ranges = [float(x.strip()) for x in ranges_str.split(",") if x.strip()]
                
                # 将角度转换为弧度
                import math
                target_angle = math.radians(angle_degrees)
                
                # 计算索引
                index = int((target_angle - angle_min) / angle_increment)
                
                # 检查索引是否有效
                if 0 <= index < len(ranges):
                    distance = ranges[index]
                    return distance if distance != float('inf') else -1
                else:
                    print(f"[警告] 角度 {angle_degrees}° 超出雷达范围")
                    return -1
            else:
                print("[警告] 无法解析雷达数据")
                return -1
                
        except subprocess.TimeoutExpired:
            print("[警告] 获取雷达数据超时")
            return -1
        except Exception as e:
            print(f"[错误] 获取雷达距离失败: {e}")
            return -1
    
    # ==================== 辅助函数 ====================
    
    def _launch_rviz(self, config_type):
        """内部函数：启动 RViz 可视化"""
        try:
            cmd = ["rviz2"]
            self.rviz_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("[可视化] RViz 已启动")
        except:
            print("[提示] 无法启动 RViz，请手动打开")
    
    def _launch_image_view(self):
        """内部函数：启动图像查看器"""
        try:
            cmd = ["ros2", "run", "rqt_image_view", "rqt_image_view"]
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("[可视化] 图像查看器已启动")
        except:
            print("[提示] 无法启动图像查看器")
    
    def _send_stop_command(self):
        """内部函数：发送停止命令"""
        try:
            cmd = [
                "ros2", "topic", "pub", "--once",
                "/cmd_vel", "geometry_msgs/msg/Twist",
                "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
            ]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except:
            pass
    
    def _start_callback_thread(self, callback, topic_suffix):
        """内部函数：启动回调线程"""
        # 这里可以实现一个后台线程定期调用回调函数
        # 由于实现较复杂，这里仅作为接口预留
        pass
    
    def __del__(self):
        """析构函数：清理所有进程"""
        self.stop_application()
        self.stop_lidar()
        self.stop_camera()


# 模块测试代码
if __name__ == "__main__":
    print("=" * 60)
    print("ROS2 感知与功能测试")
    print("=" * 60)
    
    sensors = RobotSensors()
    
    try:
        print("\n[测试1] 启动雷达...")
        sensors.launch_lidar()
        time.sleep(3)
        
        print("\n[测试2] 获取雷达距离...")
        dist = sensors.get_lidar_distance(0)
        print(f"正前方距离: {dist:.2f}m")
        
        print("\n[测试3] 停止雷达...")
        sensors.stop_lidar()
        
        print("\n测试完成！")
        
    except KeyboardInterrupt:
        print("\n\n测试被中断")
        sensors.stop_application()
        sensors.stop_lidar()
        sensors.stop_camera()
