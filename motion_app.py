#!/usr/bin/env python3
# 文件名: motion_app.py
# 功能：底盘运动控制功能示例应用
# 说明：演示如何使用 robot_lib_motion.py 中的运动控制函数

from robot_lib_motion import RobotMotion
from robot_lib_system import RobotSystem
import time
import math
import sys

def example_basic_velocity_control():
    """
    示例1：基本速度控制
    
    演示如何控制机器人以指定速度移动
    """
    print("\n" + "="*60)
    print("示例1：基本速度控制")
    print("="*60)
    
    # 先初始化系统
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        print("系统初始化失败")
        return
    
    try:
        motion = RobotMotion("mec")
        
        # 测试1：前进
        print("\n1. 前进 0.3m/s，持续 3 秒...")
        motion.set_velocity(0.3, 0.0, 0.0)
        time.sleep(3)
        motion.stop()
        print("   停止")
        time.sleep(1)
        
        # 测试2：后退
        print("\n2. 后退 0.2m/s，持续 2 秒...")
        motion.set_velocity(-0.2, 0.0, 0.0)
        time.sleep(2)
        motion.stop()
        print("   停止")
        time.sleep(1)
        
        # 测试3：左移（仅麦轮）
        print("\n3. 左移 0.2m/s，持续 2 秒...")
        motion.set_velocity(0.0, 0.2, 0.0)
        time.sleep(2)
        motion.stop()
        print("   停止")
        time.sleep(1)
        
        # 测试4：原地旋转
        print("\n4. 逆时针旋转 0.5rad/s，持续 3 秒...")
        motion.set_velocity(0.0, 0.0, 0.5)
        time.sleep(3)
        motion.stop()
        print("   停止")
        
    finally:
        robot_sys.shutdown()


def example_move_square_mecanum():
    """
    示例2：麦轮车走正方形
    
    演示麦轮车利用全向移动能力走正方形
    """
    print("\n" + "="*60)
    print("示例2：麦轮车走正方形")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        motion = RobotMotion("mec")
        
        side_length = 1.0  # 正方形边长（米）
        speed = 0.2        # 移动速度（m/s）
        
        print(f"\n开始走正方形（边长 {side_length}m）...")
        
        # 前进
        print("\n1. 前进...")
        motion.move_distance_mecanum(side_length, 0.0, speed, 0.0)
        time.sleep(0.5)
        
        # 左移
        print("\n2. 左移...")
        motion.move_distance_mecanum(0.0, side_length, 0.0, speed)
        time.sleep(0.5)
        
        # 后退
        print("\n3. 后退...")
        motion.move_distance_mecanum(-side_length, 0.0, speed, 0.0)
        time.sleep(0.5)
        
        # 右移
        print("\n4. 右移...")
        motion.move_distance_mecanum(0.0, -side_length, 0.0, speed)
        time.sleep(0.5)
        
        print("\n✅ 正方形轨迹完成！")
        
    finally:
        robot_sys.shutdown()


def example_move_distance_diff():
    """
    示例3：差速车前进指定距离
    
    演示差速车精确移动指定距离
    """
    print("\n" + "="*60)
    print("示例3：差速车前进指定距离")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("diff"):
        return
    
    try:
        motion = RobotMotion("diff")
        
        # 直线前进
        print("\n1. 直线前进 2 米...")
        motion.move_distance_ackermann_diff(2.0, 0.3)
        time.sleep(1)
        
        # 后退
        print("\n2. 后退 1 米...")
        motion.move_distance_ackermann_diff(-1.0, 0.3)
        time.sleep(1)
        
        print("\n✅ 移动完成！")
        
    finally:
        robot_sys.shutdown()


def example_rotate_angles():
    """
    示例4：旋转指定角度
    
    演示机器人旋转到指定角度
    """
    print("\n" + "="*60)
    print("示例4：旋转指定角度")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("diff"):
        return
    
    try:
        motion = RobotMotion("diff")
        
        # 逆时针旋转 90 度
        print("\n1. 逆时针旋转 90°...")
        motion.rotate_angle(90)
        time.sleep(1)
        
        # 再逆时针旋转 90 度
        print("\n2. 再逆时针旋转 90°...")
        motion.rotate_angle(90)
        time.sleep(1)
        
        # 顺时针旋转 180 度回到起点
        print("\n3. 顺时针旋转 180°...")
        motion.rotate_angle(-180)
        time.sleep(1)
        
        print("\n✅ 旋转完成！")
        
    finally:
        robot_sys.shutdown()


def example_get_sensor_data():
    """
    示例5：获取传感器数据
    
    演示如何获取位姿、IMU、车轮速度等传感器数据
    """
    print("\n" + "="*60)
    print("示例5：获取传感器数据")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        motion = RobotMotion("mec")
        
        print("\n开始采集传感器数据（持续 10 秒）...")
        print("按 Ctrl+C 提前停止\n")
        
        for i in range(10):
            print(f"\n--- 第 {i+1} 次采集 ---")
            
            # 获取位姿
            pose = motion.get_robot_pose()
            if pose:
                print(f"位姿: X={pose['x']:.3f}m, Y={pose['y']:.3f}m, "
                      f"朝向={math.degrees(pose['theta']):.1f}°")
            
            # 获取 IMU 数据
            imu = motion.get_imu_data()
            print(f"IMU: 角速度Z={imu['gyro_z']:.3f}rad/s, "
                  f"加速度Z={imu['accel_z']:.3f}m/s²")
            
            # 获取车轮速度
            wheels = motion.get_wheel_speeds()
            print(f"车轮速度: FL={wheels[0]:.2f}, FR={wheels[1]:.2f}, "
                  f"RL={wheels[2]:.2f}, RR={wheels[3]:.2f}")
            
            time.sleep(1)
        
        print("\n✅ 数据采集完成！")
        
    except KeyboardInterrupt:
        print("\n\n数据采集被中断")
    
    finally:
        robot_sys.shutdown()


def example_figure_eight():
    """
    示例6：走8字形轨迹
    
    演示机器人走复杂的8字形轨迹
    """
    print("\n" + "="*60)
    print("示例6：走8字形轨迹")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("diff"):
        return
    
    try:
        motion = RobotMotion("diff")
        
        print("\n开始走8字形...")
        
        # 第一个圆（逆时针）
        print("\n1. 第一个圆（逆时针）...")
        for _ in range(4):
            motion.move_distance_ackermann_diff(1.57, 0.3, 0.5)  # π/2 * r
            time.sleep(0.2)
        
        # 第二个圆（顺时针）
        print("\n2. 第二个圆（顺时针）...")
        for _ in range(4):
            motion.move_distance_ackermann_diff(1.57, 0.3, -0.5)
            time.sleep(0.2)
        
        print("\n✅ 8字形轨迹完成！")
        
    finally:
        robot_sys.shutdown()


def example_direct_wheel_control():
    """
    示例7：直接控制车轮速度
    
    演示如何直接控制四个车轮的转速
    """
    print("\n" + "="*60)
    print("示例7：直接控制车轮速度")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        motion = RobotMotion("mec")
        
        print("\n注意：该功能需要底盘支持直接电机控制")
        
        # 四轮同速前进
        print("\n1. 四轮同速前进（50 rad/s）...")
        motion.set_wheel_speeds(50, 50, 50, 50)
        time.sleep(2)
        motion.stop()
        time.sleep(1)
        
        # 原地旋转（左侧后转，右侧前转）
        print("\n2. 原地逆时针旋转...")
        motion.set_wheel_speeds(-30, 30, -30, 30)
        time.sleep(2)
        motion.stop()
        time.sleep(1)
        
        # 横向移动（仅麦轮）
        print("\n3. 横向左移...")
        motion.set_wheel_speeds(30, -30, -30, 30)
        time.sleep(2)
        motion.stop()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_ackermann_steering():
    """
    示例8：阿克曼转向控制
    
    演示阿克曼车型的转向角度控制
    """
    print("\n" + "="*60)
    print("示例8：阿克曼转向控制")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("akm"):
        return
    
    try:
        motion = RobotMotion("akm")
        
        print("\n注意：该功能需要阿克曼车型支持")
        
        # 设置前进速度
        print("\n1. 设置基础速度 0.3m/s...")
        motion.set_velocity(0.3, 0.0, 0.0)
        
        # 左转 30 度
        print("\n2. 左转 30°...")
        motion.set_ackermann_angle(30)
        time.sleep(3)
        
        # 回正
        print("\n3. 回正...")
        motion.set_ackermann_angle(0)
        time.sleep(2)
        
        # 右转 30 度
        print("\n4. 右转 30°...")
        motion.set_ackermann_angle(-30)
        time.sleep(3)
        
        # 停止
        motion.stop()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_continuous_monitoring():
    """
    示例9：持续监控运动状态
    
    演示在运动过程中持续监控传感器数据
    """
    print("\n" + "="*60)
    print("示例9：持续监控运动状态")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("diff"):
        return
    
    try:
        motion = RobotMotion("diff")
        
        print("\n机器人将前进并持续显示状态...")
        print("按 Ctrl+C 停止\n")
        
        # 设置前进速度
        motion.set_velocity(0.2, 0.0, 0.0)
        
        start_time = time.time()
        
        while True:
            # 获取当前状态
            pose = motion.get_robot_pose()
            imu = motion.get_imu_data()
            
            # 计算运行时间
            elapsed = time.time() - start_time
            
            if pose:
                # 显示状态
                print(f"[{elapsed:.1f}s] "
                      f"位置:({pose['x']:.2f}, {pose['y']:.2f}m) "
                      f"朝向:{math.degrees(pose['theta']):.0f}° "
                      f"角速度:{imu['gyro_z']:.2f}rad/s")
            
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\n\n停止运动...")
        motion.stop()
    
    finally:
        robot_sys.shutdown()


def main():
    """
    主函数：提供交互式菜单选择不同的示例
    """
    examples = {
        "1": ("基本速度控制", example_basic_velocity_control),
        "2": ("麦轮车走正方形", example_move_square_mecanum),
        "3": ("差速车前进指定距离", example_move_distance_diff),
        "4": ("旋转指定角度", example_rotate_angles),
        "5": ("获取传感器数据", example_get_sensor_data),
        "6": ("走8字形轨迹", example_figure_eight),
        "7": ("直接控制车轮速度", example_direct_wheel_control),
        "8": ("阿克曼转向控制", example_ackermann_steering),
        "9": ("持续监控运动状态", example_continuous_monitoring),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 底盘运动控制功能示例应用")
        print("="*60)
        print("\n请选择要运行的示例：")
        print()
        
        for key, (name, _) in examples.items():
            print(f"  {key}. {name}")
        
        print("  0. 退出程序")
        print()
        
        choice = input("请输入选项 (0-9): ").strip()
        
        if choice == "0":
            print("\n再见！")
            break
        
        if choice in examples:
            name, func = examples[choice]
            try:
                func()
                input("\n按 Enter 键继续...")
            except KeyboardInterrupt:
                print("\n\n示例被中断")
                input("\n按 Enter 键继续...")
            except Exception as e:
                print(f"\n❌ 发生错误: {e}")
                import traceback
                traceback.print_exc()
                input("\n按 Enter 键继续...")
        else:
            print("\n❌ 无效的选项，请重新选择")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序已退出")
        sys.exit(0)
