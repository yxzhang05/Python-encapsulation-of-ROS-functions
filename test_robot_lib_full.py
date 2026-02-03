#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试文件：test_robot_lib_full.py
测试内容：完整功能测试 - 系统 + 底盘 + 机械臂 + 感知 + 建图导航
适用车型：麦轮车（mec）
"""

from robot_lib_full import Robot
import time
import os

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("完整功能测试菜单")
    print("="*50)
    print("【建图导航功能】")
    print("1. 测试启动建图 (SLAM)")
    print("2. 测试保存地图")
    print("3. 测试启动键盘控制")
    print("4. 测试加载地图并启动导航")
    print("5. 测试导航到目标点")
    print("6. 测试取消导航")
    print("")
    print("【综合演示】")
    print("7. 运行建图完整流程演示")
    print("8. 运行导航完整流程演示")
    print("9. 运行麦轮车全功能演示")
    print("")
    print("0. 退出")
    print("="*50)

def test_start_mapping(robot):
    """测试1：启动建图"""
    print("\n【测试1】启动建图 (SLAM)")
    print("-" * 40)
    
    print("⚠️  建图功能将启动")
    print("建议使用键盘控制机器人移动，绘制地图")
    input("按回车键开始...")
    
    print("正在启动SLAM建图...")
    robot.start_mapping()
    print("✅ 建图已启动")
    print("提示：")
    print("  - 可以在另一个终端运行键盘控制")
    print("  - 或使用 rviz2 查看建图过程")
    print("  - 完成后使用'保存地图'功能保存")
    
    return True

def test_save_map(robot):
    """测试2：保存地图"""
    print("\n【测试2】保存地图")
    print("-" * 40)
    
    # 默认地图路径
    default_path = os.path.expanduser("~/wheeltec_robot/maps")
    map_name = input(f"请输入地图名称 (默认: test_map): ").strip()
    if not map_name:
        map_name = "test_map"
    
    map_path = os.path.join(default_path, map_name)
    
    print(f"正在保存地图到: {map_path}")
    robot.save_map(map_path)
    print("✅ 地图已保存")
    print(f"地图文件: {map_path}.yaml 和 {map_path}.pgm")
    
    return True

def test_start_keyboard_control(robot):
    """测试3：启动键盘控制"""
    print("\n【测试3】启动键盘控制")
    print("-" * 40)
    
    print("⚠️  键盘控制将启动")
    print("使用方向键控制机器人移动")
    print("按 Ctrl+C 停止键盘控制")
    input("按回车键开始...")
    
    print("启动键盘控制...")
    robot.start_keyboard_control()
    print("✅ 键盘控制已启动")
    
    return True

def test_load_map_and_navigate(robot):
    """测试4：加载地图并启动导航"""
    print("\n【测试4】加载地图并启动导航")
    print("-" * 40)
    
    default_path = os.path.expanduser("~/wheeltec_robot/maps")
    map_name = input(f"请输入要加载的地图名称 (默认: test_map): ").strip()
    if not map_name:
        map_name = "test_map"
    
    map_path = os.path.join(default_path, map_name)
    
    print(f"正在加载地图: {map_path}")
    robot.load_map_and_start_navigation(map_path)
    print("✅ 地图已加载，导航系统已启动")
    print("提示：现在可以设置导航目标点")
    
    return True

def test_move_to_goal(robot):
    """测试5：导航到目标点"""
    print("\n【测试5】导航到目标点")
    print("-" * 40)
    
    print("请输入目标点坐标（单位：米）")
    try:
        x = float(input("X 坐标: ").strip())
        y = float(input("Y 坐标: ").strip())
        yaw = float(input("朝向角度 (度，可选，回车跳过): ").strip() or "0")
    except ValueError:
        print("❌ 输入格式错误")
        return False
    
    print(f"正在导航到目标点: ({x:.2f}, {y:.2f}, {yaw:.2f}°)")
    robot.move_to_goal(x, y, yaw)
    print("✅ 导航任务已发送")
    print("提示：机器人将自动规划路径并移动到目标点")
    
    return True

def test_cancel_navigation(robot):
    """测试6：取消导航"""
    print("\n【测试6】取消导航")
    print("-" * 40)
    
    print("正在取消当前导航任务...")
    robot.cancel_navigation()
    print("✅ 导航已取消")
    
    return True

def test_mapping_workflow(robot):
    """测试7：建图完整流程演示"""
    print("\n【测试7】建图完整流程演示")
    print("-" * 40)
    print("演示内容：")
    print("1. 启动建图")
    print("2. 使用键盘控制移动（需手动操作）")
    print("3. 保存地图")
    print("⚠️  这是一个交互式流程")
    input("按回车键开始...")
    
    print("\n步骤1: 启动建图")
    robot.start_mapping()
    print("✅ 建图已启动")
    time.sleep(2)
    
    print("\n步骤2: 手动控制机器人")
    print("请使用键盘或手动推动机器人移动")
    print("探索环境以建立地图")
    input("完成建图后按回车继续...")
    
    print("\n步骤3: 保存地图")
    map_path = os.path.expanduser("~/wheeltec_robot/maps/demo_map")
    robot.save_map(map_path)
    print(f"✅ 地图已保存到: {map_path}")
    
    print("\n✅ 建图流程演示完成")
    
    return True

def test_navigation_workflow(robot):
    """测试8：导航完整流程演示"""
    print("\n【测试8】导航完整流程演示")
    print("-" * 40)
    print("演示内容：")
    print("1. 加载已保存的地图")
    print("2. 启动导航系统")
    print("3. 设置目标点并导航")
    print("⚠️  需要先完成建图流程")
    input("按回车键开始...")
    
    print("\n步骤1: 加载地图")
    map_path = os.path.expanduser("~/wheeltec_robot/maps/demo_map")
    if not os.path.exists(f"{map_path}.yaml"):
        print(f"⚠️  地图文件不存在: {map_path}.yaml")
        print("请先运行建图流程")
        return False
    
    robot.load_map_and_start_navigation(map_path)
    print("✅ 地图已加载，导航系统已启动")
    time.sleep(3)
    
    print("\n步骤2: 设置目标点")
    print("示例：导航到 (1.0, 1.0)")
    robot.move_to_goal(1.0, 1.0, 0.0)
    print("✅ 导航任务已发送")
    
    print("\n提示：机器人正在自动导航")
    print("可以按 Ctrl+C 取消导航")
    
    print("\n✅ 导航流程演示完成")
    
    return True

def test_comprehensive_demo(robot):
    """测试9：麦轮车全功能演示"""
    print("\n【测试9】麦轮车全功能综合演示")
    print("-" * 40)
    print("演示内容：")
    print("1. 系统初始化")
    print("2. 传感器启动（雷达、相机）")
    print("3. 麦轮全向移动演示")
    print("4. 传感器数据读取")
    print("5. 系统关闭")
    print("⚠️  请确保周围有足够空间")
    input("按回车键开始...")
    
    print("\n=== 第1部分：传感器启动 ===")
    print("启动雷达...")
    robot.launch_lidar()
    time.sleep(2)
    
    print("启动相机...")
    robot.launch_camera()
    time.sleep(2)
    
    print("\n=== 第2部分：麦轮全向移动 ===")
    speed = 0.15
    duration = 1.5
    
    print("1. 前进...")
    robot.set_velocity(speed, 0, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("2. 左平移（麦轮特性）...")
    robot.set_velocity(0, speed, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("3. 斜向移动（麦轮特性）...")
    robot.set_velocity(speed*0.7, speed*0.7, 0)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    time.sleep(0.5)
    
    print("4. 原地旋转...")
    robot.set_velocity(0, 0, 0.5)
    time.sleep(duration)
    robot.set_velocity(0, 0, 0)
    
    print("\n=== 第3部分：传感器数据读取 ===")
    print("读取雷达数据...")
    distances = robot.get_lidar_distance()
    if distances:
        print(f"  前方: {distances.get('front', 'N/A'):.2f} m")
        print(f"  最小: {distances.get('min', 'N/A'):.2f} m")
    
    print("\n读取IMU数据...")
    imu = robot.get_imu_data()
    print(f"  加速度Z: {imu.get('accel_z', 'N/A'):.3f}")
    print(f"  角速度Z: {imu.get('gyro_z', 'N/A'):.3f}")
    
    print("\n读取位姿...")
    pose = robot.get_robot_pose()
    print(f"  位置: ({pose.get('x', 'N/A'):.2f}, {pose.get('y', 'N/A'):.2f})")
    
    print("\n=== 第4部分：传感器关闭 ===")
    robot.stop_lidar()
    robot.stop_camera()
    
    print("\n✅ 全功能演示完成")
    print("麦轮车的全向移动能力和传感器功能已展示")
    
    return True

def main():
    """主函数"""
    print("\n" + "="*50)
    print("完整功能测试程序")
    print("车型: 麦轮车 (mec)")
    print("功能: 系统 + 底盘 + 机械臂 + 感知 + 建图导航")
    print("="*50)
    print("⚠️  安全提示：")
    print("  - 建图和导航需要足够的空间")
    print("  - 确保所有传感器正常工作")
    print("  - 某些功能需要特定硬件支持")
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
                test_start_mapping(robot)
            elif choice == '2':
                test_save_map(robot)
            elif choice == '3':
                test_start_keyboard_control(robot)
            elif choice == '4':
                test_load_map_and_navigate(robot)
            elif choice == '5':
                test_move_to_goal(robot)
            elif choice == '6':
                test_cancel_navigation(robot)
            elif choice == '7':
                test_mapping_workflow(robot)
            elif choice == '8':
                test_navigation_workflow(robot)
            elif choice == '9':
                test_comprehensive_demo(robot)
            else:
                print("❌ 无效选择，请重试")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        print("正在停止所有任务...")
        robot.cancel_navigation()
        robot.stop_application()
        robot.emergency_stop()
    
    finally:
        print("\n正在安全关闭...")
        robot.shutdown()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()
