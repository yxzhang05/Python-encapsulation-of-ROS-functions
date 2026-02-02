#!/usr/bin/env python3
# 文件名: robot_lib_full.py
# 功能：整合所有 ROS2 功能的完整机器人控制库
# 说明：提供统一的接口访问所有机器人功能

from robot_lib_system import RobotSystem
from robot_lib_motion import RobotMotion
from robot_lib_arm import RobotArm
from robot_lib_sensors import RobotSensors
from robot_lib_navigation import RobotNavigation

class Robot:
    """
    完整的机器人控制类
    
    整合了所有子模块，提供统一的接口：
    - system: 系统管理（初始化、关闭、电压等）
    - motion: 底盘运动控制
    - arm: 机械臂控制
    - sensors: 感知功能（雷达、相机、视觉应用）
    - navigation: 建图与导航
    
    使用示例：
    >>> robot = Robot()
    >>> robot.initialize("mec")  # 初始化系统
    >>> robot.motion.set_velocity(0.2, 0, 0)  # 控制运动
    >>> robot.sensors.launch_lidar()  # 启动雷达
    >>> robot.shutdown()  # 关闭系统
    """
    
    def __init__(self, robot_type="mec"):
        """
        构造函数
        
        参数：
        - robot_type (str): 机器人车型，默认 "mec"
        """
        self.robot_type = robot_type
        self.is_initialized = False
        
        # 创建各功能模块
        self._system = RobotSystem()
        self._motion = RobotMotion(robot_type)
        self._arm = RobotArm()
        self._sensors = RobotSensors()
        self._navigation = RobotNavigation()
    
    # ==================== 系统管理接口 ====================
    
    @property
    def system(self):
        """系统管理模块"""
        return self._system
    
    @property
    def motion(self):
        """底盘运动控制模块"""
        return self._motion
    
    @property
    def arm(self):
        """机械臂控制模块"""
        return self._arm
    
    @property
    def sensors(self):
        """感知功能模块"""
        return self._sensors
    
    @property
    def navigation(self):
        """建图导航模块"""
        return self._navigation
    
    # ==================== 快捷方法 ====================
    
    def initialize(self, robot_type=None):
        """
        初始化机器人系统
        
        参数：
        - robot_type (str): 车型，如果不提供则使用构造函数中的默认值
        
        返回值：
        - bool: 初始化成功返回 True
        """
        if robot_type:
            self.robot_type = robot_type
        
        success = self._system.initialize(self.robot_type)
        if success:
            self.is_initialized = True
            self._motion.robot_type = self.robot_type
        
        return success
    
    def shutdown(self):
        """
        关闭机器人系统
        
        按顺序关闭所有模块
        """
        print("\n[Robot] 正在关闭机器人系统...")
        
        # 1. 停止所有应用
        self._sensors.stop_application()
        
        # 2. 停止运动
        self._motion.stop()
        
        # 3. 关闭传感器
        self._sensors.stop_camera()
        self._sensors.stop_lidar()
        
        # 4. 关闭导航
        self._navigation.shutdown()
        
        # 5. 关闭系统
        self._system.shutdown()
        
        self.is_initialized = False
        print("[Robot] 系统已完全关闭")
    
    def emergency_stop(self):
        """紧急停止"""
        self._system.emergency_stop()
    
    def get_battery_voltage(self):
        """获取电池电压"""
        return self._system.get_battery_voltage()
    
    # ==================== 运动控制快捷方法 ====================
    
    def move(self, vx, vy=0.0, wz=0.0):
        """
        控制机器人运动
        
        参数：
        - vx: X 方向速度 (m/s)
        - vy: Y 方向速度 (m/s)，仅麦轮有效
        - wz: 角速度 (rad/s)
        """
        self._motion.set_velocity(vx, vy, wz)
    
    def stop(self):
        """停止运动"""
        self._motion.stop()
    
    def forward(self, distance, speed=0.3):
        """
        前进指定距离
        
        参数：
        - distance: 距离 (m)
        - speed: 速度 (m/s)
        """
        if self.robot_type == "mec":
            return self._motion.move_distance_mecanum(distance, 0, speed, 0)
        else:
            return self._motion.move_distance_ackermann_diff(distance, speed)
    
    def turn(self, angle, speed=0.5):
        """
        旋转指定角度
        
        参数：
        - angle: 角度 (度)
        - speed: 角速度 (rad/s)
        """
        return self._motion.rotate_angle(angle, speed)
    
    # ==================== 实用工具方法 ====================
    
    def get_status(self):
        """
        获取机器人完整状态
        
        返回值：
        - dict: 包含所有状态信息的字典
        """
        status = {
            "initialized": self.is_initialized,
            "robot_type": self.robot_type,
            "battery_voltage": self._system.get_battery_voltage(),
            "pose": self._motion.get_robot_pose(),
            "imu": self._motion.get_imu_data(),
        }
        
        return status
    
    def print_status(self):
        """打印机器人状态"""
        status = self.get_status()
        
        print("\n" + "="*50)
        print("机器人状态")
        print("="*50)
        print(f"初始化状态: {'✅ 已初始化' if status['initialized'] else '❌ 未初始化'}")
        print(f"车型: {status['robot_type']}")
        print(f"电池电压: {status['battery_voltage']:.2f}V")
        
        if status['pose']:
            pose = status['pose']
            import math
            print(f"位置: ({pose['x']:.2f}, {pose['y']:.2f})m")
            print(f"朝向: {math.degrees(pose['theta']):.1f}°")
        
        if status['imu']:
            imu = status['imu']
            print(f"角速度 Z: {imu['gyro_z']:.3f} rad/s")
        
        print("="*50 + "\n")
    
    def __enter__(self):
        """支持 with 语句"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """with 语句结束时自动关闭"""
        self.shutdown()
        return False
    
    def __del__(self):
        """析构函数：确保资源释放"""
        if self.is_initialized:
            self.shutdown()


# 使用示例
if __name__ == "__main__":
    import time
    
    print("=" * 60)
    print("完整机器人控制库测试")
    print("=" * 60)
    
    # 方法 1：常规使用
    robot = Robot()
    
    try:
        # 初始化
        print("\n[测试] 初始化机器人...")
        if robot.initialize("mec"):
            print("✅ 初始化成功")
            
            # 打印状态
            robot.print_status()
            
            # 前进
            print("\n[测试] 前进 1 米...")
            robot.forward(1.0, 0.3)
            
            # 旋转
            print("\n[测试] 旋转 90 度...")
            robot.turn(90)
            
            # 打印最终状态
            robot.print_status()
    
    finally:
        robot.shutdown()
    
    # 方法 2：使用 with 语句（推荐）
    print("\n\n使用 with 语句测试...")
    
    with Robot() as robot:
        robot.initialize("mec")
        robot.print_status()
        # ... 使用机器人 ...
        # 自动调用 shutdown()
    
    print("\n✅ 所有测试完成！")
