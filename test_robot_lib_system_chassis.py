#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试文件：test_robot_lib_system_chassis.py
测试内容：系统管理 + 底盘运动控制功能
适用车型：麦轮车（mec）
"""

from robot_lib_system_chassis import Robot
import time

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("底盘运动控制功能测试菜单")
    print("="*50)
    print("1. 测试基础速度控制")
    print("2. 测试直线移动 (move_distance)")
    print("3. 测试二维移动 (move_distance_xy) - 麦轮专用")
    print("4. 测试原地旋转 (rotate_angle)")
    print("5. 测试获取车轮速度")
    print("6. 测试获取IMU数据")
    print("7. 测试获取机器人位姿")
    print("8. 运行综合运动演示")
    print("9. 运行所有测试")
    print("0. 退出")
    print("="*50)

def test_velocity_control(robot):
    """测试1：基础速度控制"""
    print("\n【测试1】基础速度控制")
    print("-" * 40)
    
    print("测试前进...")
    robot.set_velocity(0.2, 0, 0)  # 前进 0.2 m/s
    print("✅ 机器人正在前进，速度 0.2 m/s")
    time.sleep(2)
    
    print("\n测试左平移（麦轮特性）...")
    robot.set_velocity(0, 0.2, 0)  # 左平移 0.2 m/s
    print("✅ 机器人正在左平移，速度 0.2 m/s")
    time.sleep(2)
    
    print("\n测试右平移（麦轮特性）...")
    robot.set_velocity(0, -0.2, 0)  # 右平移 0.2 m/s
    print("✅ 机器人正在右平移，速度 0.2 m/s")
    time.sleep(2)
    
    print("\n测试旋转...")
    robot.set_velocity(0, 0, 0.5)  # 逆时针旋转
    print("✅ 机器人正在旋转，角速度 0.5 rad/s")
    time.sleep(2)
    
    print("\n停止运动...")
    robot.set_velocity(0, 0, 0)
    print("✅ 机器人已停止")
    
    return True

def test_move_distance(robot):
    """测试2：直线移动"""
    print("\n【测试2】直线移动测试")
    print("-" * 40)
    
    distance = 0.3  # 移动30cm
    speed = 0.2     # 速度 0.2 m/s
    
    print(f"准备前进 {distance} 米，速度 {speed} m/s")
    print("⚠️  请确保前方有足够空间")
    input("按回车键开始...")
    
    print("开始移动...")
    robot.move_distance(distance, speed)
    print(f"✅ 已完成 {distance} 米移动")
    
    return True

def test_move_distance_xy(robot):
    """测试3：二维移动（麦轮专用）"""
    print("\n【测试3】二维移动测试（麦轮专用）")
    print("-" * 40)
    
    distance_x = 0.3  # X方向30cm
    distance_y = 0.3  # Y方向30cm
    speed_x = 0.2
    speed_y = 0.2
    
    print(f"准备移动：X={distance_x}m, Y={distance_y}m")
    print("麦轮车将斜向移动")
    print("⚠️  请确保周围有足够空间")
    input("按回车键开始...")
    
    print("开始移动...")
    robot.move_distance_xy(distance_x, distance_y, speed_x, speed_y)
    print(f"✅ 已完成二维移动")
    
    return True

def test_rotate_angle(robot):
    """测试4：原地旋转"""
    print("\n【测试4】原地旋转测试")
    print("-" * 40)
    
    angle = 90  # 旋转90度
    speed = 0.5  # 角速度
    
    print(f"准备旋转 {angle} 度，角速度 {speed} rad/s")
    input("按回车键开始...")
    
    print("开始旋转...")
    robot.rotate_angle(angle, speed)
    print(f"✅ 已完成 {angle} 度旋转")
    
    return True

def test_get_wheel_speeds(robot):
    """测试5：获取车轮速度"""
    print("\n【测试5】获取车轮速度")
    print("-" * 40)
    
    print("正在让机器人缓慢前进...")
    robot.set_velocity(0.1, 0, 0)
    time.sleep(1)
    
    print("读取车轮速度...")
    speeds = robot.get_wheel_speeds()
    print(f"📊 车轮速度: {speeds}")
    print(f"   左前轮: {speeds.get('front_left', 'N/A')}")
    print(f"   右前轮: {speeds.get('front_right', 'N/A')}")
    print(f"   左后轮: {speeds.get('rear_left', 'N/A')}")
    print(f"   右后轮: {speeds.get('rear_right', 'N/A')}")
    
    robot.set_velocity(0, 0, 0)
    print("✅ 车轮速度读取完成")
    
    return True

def test_get_imu_data(robot):
    """测试6：获取IMU数据"""
    print("\n【测试6】获取IMU数据")
    print("-" * 40)
    
    print("读取IMU数据...")
    imu = robot.get_imu_data()
    print(f"📊 IMU六轴数据:")
    print(f"   加速度 - X: {imu.get('accel_x', 'N/A'):.3f}, Y: {imu.get('accel_y', 'N/A'):.3f}, Z: {imu.get('accel_z', 'N/A'):.3f}")
    print(f"   角速度 - X: {imu.get('gyro_x', 'N/A'):.3f}, Y: {imu.get('gyro_y', 'N/A'):.3f}, Z: {imu.get('gyro_z', 'N/A'):.3f}")
    print("✅ IMU数据读取完成")
    
    return True

def test_get_robot_pose(robot):
    """测试7：获取机器人位姿"""
    print("\n【测试7】获取机器人位姿")
    print("-" * 40)
    
    print("读取机器人位姿...")
    pose = robot.get_robot_pose()
    print(f"📊 机器人位姿:")
    print(f"   位置 X: {pose.get('x', 'N/A'):.3f} m")
    print(f"   位置 Y: {pose.get('y', 'N/A'):.3f} m")
    print(f"   朝向 Yaw: {pose.get('yaw', 'N/A'):.3f} rad ({pose.get('yaw', 0) * 57.3:.1f}°)")
    print("✅ 位姿数据读取完成")
    
    return True

def test_comprehensive_demo(robot):
    """测试8：综合运动演示"""
    print("\n【测试8】综合运动演示")
    print("-" * 40)
    print("演示内容：麦轮车的全向移动能力")
    print("1. 前进 → 2. 左平移 → 3. 后退 → 4. 右平移 → 5. 旋转")
    print("⚠️  请确保周围有足够空间（至少2m×2m）")
    input("按回车键开始...")
    
    speed = 0.15  # 较慢速度，安全
    duration = 1.5
    
    print("\n1️⃣  前进...")
    robot.set_velocity(speed, 0, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("2️⃣  左平移（麦轮特性）...")
    robot.set_velocity(0, speed, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("3️⃣  后退...")
    robot.set_velocity(-speed, 0, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("4️⃣  右平移（麦轮特性）...")
    robot.set_velocity(0, -speed, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("5️⃣  原地旋转...")
    robot.set_velocity(0, 0, 0.5)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    
    print("✅ 综合演示完成")
    print("麦轮车的全向移动能力已展示")
    
    return True

def run_all_tests(robot):
    """运行所有测试"""
    print("\n" + "="*50)
    print("开始运行所有测试")
    print("="*50)
    
    tests = [
        ("基础速度控制", test_velocity_control),
        ("获取车轮速度", test_get_wheel_speeds),
        ("获取IMU数据", test_get_imu_data),
        ("获取机器人位姿", test_get_robot_pose),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func(robot)
            results.append((name, result))
            time.sleep(1)
        except Exception as e:
            print(f"❌ 测试失败: {e}")
            results.append((name, False))
    
    # 打印测试总结
    print("\n" + "="*50)
    print("测试总结")
    print("="*50)
    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{name}: {status}")
    print("="*50)

def main():
    """主函数"""
    print("\n" + "="*50)
    print("底盘运动控制功能测试程序")
    print("车型: 麦轮车 (mec)")
    print("="*50)
    print("⚠️  安全提示：")
    print("  - 测试前请确保周围有足够空间")
    print("  - 建议先将机器人轮子架空测试")
    print("  - 可随时按 Ctrl+C 紧急停止")
    print("="*50)
    
    robot = Robot()
    
    try:
        # 初始化机器人
        print("\n正在初始化机器人...")
        success = robot.initialize("mec")
        if not success:
            print("❌ 初始化失败，程序退出")
            return
        print("✅ 初始化成功\n")
        
        while True:
            print_menu()
            choice = input("\n请选择测试项 (0-9): ").strip()
            
            if choice == '0':
                print("\n退出测试程序")
                break
            elif choice == '1':
                test_velocity_control(robot)
            elif choice == '2':
                test_move_distance(robot)
            elif choice == '3':
                test_move_distance_xy(robot)
            elif choice == '4':
                test_rotate_angle(robot)
            elif choice == '5':
                test_get_wheel_speeds(robot)
            elif choice == '6':
                test_get_imu_data(robot)
            elif choice == '7':
                test_get_robot_pose(robot)
            elif choice == '8':
                test_comprehensive_demo(robot)
            elif choice == '9':
                run_all_tests(robot)
            else:
                print("❌ 无效选择，请重试")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        print("正在紧急停止...")
        robot.emergency_stop()
    
    finally:
        print("\n正在安全关闭...")
        robot.shutdown()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()
