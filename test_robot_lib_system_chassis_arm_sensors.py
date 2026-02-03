#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试文件：test_robot_lib_system_chassis_arm_sensors.py
测试内容：系统管理 + 底盘运动 + 机械臂 + 感知功能
适用车型：麦轮车（mec）
"""

from robot_lib_system_chassis_arm_sensors import Robot
import time

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("感知功能测试菜单")
    print("="*50)
    print("1. 测试启动雷达")
    print("2. 测试停止雷达")
    print("3. 测试启动相机")
    print("4. 测试停止相机")
    print("5. 测试获取雷达距离数据")
    print("6. 测试视觉跟随")
    print("7. 测试视觉巡线")
    print("8. 测试雷达跟随")
    print("9. 测试停止应用")
    print("0. 退出")
    print("="*50)

def test_launch_lidar(robot):
    """测试1：启动雷达"""
    print("\n【测试1】启动雷达")
    print("-" * 40)
    
    print("正在启动雷达...")
    robot.launch_lidar()
    print("✅ 雷达已启动")
    print("提示：雷达数据将发布到 /scan 话题")
    
    return True

def test_stop_lidar(robot):
    """测试2：停止雷达"""
    print("\n【测试2】停止雷达")
    print("-" * 40)
    
    print("正在停止雷达...")
    robot.stop_lidar()
    print("✅ 雷达已停止")
    
    return True

def test_launch_camera(robot):
    """测试3：启动相机"""
    print("\n【测试3】启动相机")
    print("-" * 40)
    
    print("正在启动相机...")
    robot.launch_camera()
    print("✅ 相机已启动")
    print("提示：相机图像将发布到 /image_raw 话题")
    
    return True

def test_stop_camera(robot):
    """测试4：停止相机"""
    print("\n【测试4】停止相机")
    print("-" * 40)
    
    print("正在停止相机...")
    robot.stop_camera()
    print("✅ 相机已停止")
    
    return True

def test_get_lidar_distance(robot):
    """测试5：获取雷达距离数据"""
    print("\n【测试5】获取雷达距离数据")
    print("-" * 40)
    
    print("读取雷达数据...")
    distances = robot.get_lidar_distance()
    
    if distances:
        print(f"📊 雷达数据摘要:")
        print(f"   前方: {distances.get('front', 'N/A'):.2f} m")
        print(f"   左侧: {distances.get('left', 'N/A'):.2f} m")
        print(f"   右侧: {distances.get('right', 'N/A'):.2f} m")
        print(f"   后方: {distances.get('rear', 'N/A'):.2f} m")
        print(f"   最小距离: {distances.get('min', 'N/A'):.2f} m")
        print("✅ 雷达数据读取完成")
    else:
        print("⚠️  未能读取雷达数据，请确保雷达已启动")
    
    return True

def test_visual_follow(robot):
    """测试6：视觉跟随"""
    print("\n【测试6】视觉跟随")
    print("-" * 40)
    
    print("⚠️  视觉跟随功能将启动")
    print("机器人将跟随检测到的目标（如人）")
    print("按 Ctrl+C 停止跟随")
    input("按回车键开始...")
    
    print("启动视觉跟随...")
    robot.start_visual_follow()
    print("✅ 视觉跟随已启动")
    print("提示：机器人将自动跟随目标移动")
    
    return True

def test_line_tracking(robot):
    """测试7：视觉巡线"""
    print("\n【测试7】视觉巡线")
    print("-" * 40)
    
    print("⚠️  视觉巡线功能将启动")
    print("机器人将沿着地面的线条移动")
    print("按 Ctrl+C 停止巡线")
    input("按回车键开始...")
    
    print("启动视觉巡线...")
    robot.start_line_tracking()
    print("✅ 视觉巡线已启动")
    print("提示：机器人将自动沿线移动")
    
    return True

def test_lidar_follow(robot):
    """测试8：雷达跟随"""
    print("\n【测试8】雷达跟随")
    print("-" * 40)
    
    print("⚠️  雷达跟随功能将启动")
    print("机器人将跟随最近的障碍物")
    print("按 Ctrl+C 停止跟随")
    input("按回车键开始...")
    
    print("启动雷达跟随...")
    robot.start_lidar_follow()
    print("✅ 雷达跟随已启动")
    print("提示：机器人将自动跟随最近的障碍物")
    
    return True

def test_stop_application(robot):
    """测试9：停止应用"""
    print("\n【测试9】停止应用")
    print("-" * 40)
    
    print("正在停止当前运行的应用...")
    robot.stop_application()
    print("✅ 应用已停止")
    
    return True

def test_sensor_comprehensive(robot):
    """综合测试：传感器启动和数据读取"""
    print("\n【综合测试】传感器启动和数据读取")
    print("-" * 40)
    
    print("1️⃣  启动雷达...")
    robot.launch_lidar()
    time.sleep(2)
    
    print("\n2️⃣  启动相机...")
    robot.launch_camera()
    time.sleep(2)
    
    print("\n3️⃣  读取雷达数据...")
    distances = robot.get_lidar_distance()
    if distances:
        print(f"   前方距离: {distances.get('front', 'N/A'):.2f} m")
        print(f"   最小距离: {distances.get('min', 'N/A'):.2f} m")
    
    time.sleep(1)
    
    print("\n4️⃣  停止传感器...")
    robot.stop_lidar()
    robot.stop_camera()
    
    print("✅ 综合测试完成")
    
    return True

def main():
    """主函数"""
    print("\n" + "="*50)
    print("感知功能测试程序")
    print("车型: 麦轮车 (mec)")
    print("="*50)
    print("⚠️  安全提示：")
    print("  - 确保雷达和相机硬件已正确连接")
    print("  - 某些功能需要相应的硬件支持")
    print("  - 跟随功能会自动控制机器人运动，请注意安全")
    print("  - 可随时按 Ctrl+C 停止")
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
                test_launch_lidar(robot)
            elif choice == '2':
                test_stop_lidar(robot)
            elif choice == '3':
                test_launch_camera(robot)
            elif choice == '4':
                test_stop_camera(robot)
            elif choice == '5':
                test_get_lidar_distance(robot)
            elif choice == '6':
                test_visual_follow(robot)
            elif choice == '7':
                test_line_tracking(robot)
            elif choice == '8':
                test_lidar_follow(robot)
            elif choice == '9':
                test_stop_application(robot)
            else:
                print("❌ 无效选择，请重试")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        print("正在停止应用...")
        robot.stop_application()
        robot.emergency_stop()
    
    finally:
        print("\n正在安全关闭...")
        robot.shutdown()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()
