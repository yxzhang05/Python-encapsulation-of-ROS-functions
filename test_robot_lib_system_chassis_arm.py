#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试文件：test_robot_lib_system_chassis_arm.py
测试内容：系统管理 + 底盘运动 + 机械臂控制功能
适用车型：麦轮车（mec）
注意：需要机器人配备机械臂
"""

from robot_lib_system_chassis_arm import Robot
import time

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("机械臂控制功能测试菜单")
    print("="*50)
    print("1. 测试机械臂归零")
    print("2. 测试设置关节角度")
    print("3. 测试设置云台角度")
    print("4. 测试设置机械臂末端位置")
    print("5. 测试夹爪控制")
    print("6. 测试获取机械臂末端坐标")
    print("7. 测试获取关节状态")
    print("8. 测试PWM控制")
    print("9. 运行机械臂综合演示")
    print("0. 退出")
    print("="*50)

def test_arm_home(robot):
    """测试1：机械臂归零"""
    print("\n【测试1】机械臂归零")
    print("-" * 40)
    
    print("⚠️  机械臂将移动到归零位置")
    print("请确保机械臂周围无障碍物")
    input("按回车键开始...")
    
    print("执行归零...")
    robot.arm_home()
    print("✅ 机械臂已归零")
    
    return True

def test_set_joint_angles(robot):
    """测试2：设置关节角度"""
    print("\n【测试2】设置关节角度")
    print("-" * 40)
    
    print("⚠️  将设置各关节角度")
    print("请确保机械臂周围无障碍物")
    input("按回车键开始...")
    
    # 示例：设置各关节角度（单位：度）
    angles = {
        'joint1': 0,
        'joint2': 30,
        'joint3': -30,
        'joint4': 0
    }
    
    print(f"设置关节角度: {angles}")
    robot.set_joint_angles(angles)
    print("✅ 关节角度已设置")
    time.sleep(2)
    
    # 恢复到初始位置
    print("恢复到初始位置...")
    robot.arm_home()
    
    return True

def test_set_yaw_angle(robot):
    """测试3：设置云台角度"""
    print("\n【测试3】设置云台角度")
    print("-" * 40)
    
    print("测试云台左右转动...")
    
    print("云台向左转30度...")
    robot.set_yaw_angle(30)
    time.sleep(1)
    
    print("云台回中...")
    robot.set_yaw_angle(0)
    time.sleep(1)
    
    print("云台向右转30度...")
    robot.set_yaw_angle(-30)
    time.sleep(1)
    
    print("云台回中...")
    robot.set_yaw_angle(0)
    
    print("✅ 云台角度测试完成")
    
    return True

def test_set_arm_position(robot):
    """测试4：设置机械臂末端位置"""
    print("\n【测试4】设置机械臂末端位置")
    print("-" * 40)
    
    print("⚠️  将通过逆运动学设置末端位置")
    print("请确保机械臂周围无障碍物")
    input("按回车键开始...")
    
    # 示例：设置末端位置（单位：米）
    x, y, z = 0.2, 0.0, 0.15
    
    print(f"设置末端位置: X={x}m, Y={y}m, Z={z}m")
    robot.set_arm_position(x, y, z)
    print("✅ 末端位置已设置")
    time.sleep(2)
    
    # 恢复到初始位置
    print("恢复到初始位置...")
    robot.arm_home()
    
    return True

def test_set_gripper(robot):
    """测试5：夹爪控制"""
    print("\n【测试5】夹爪控制")
    print("-" * 40)
    
    print("测试夹爪开合...")
    
    print("打开夹爪...")
    robot.set_gripper(1.0)  # 完全打开
    print("✅ 夹爪已打开")
    time.sleep(2)
    
    print("半开夹爪...")
    robot.set_gripper(0.5)  # 半开
    print("✅ 夹爪半开")
    time.sleep(2)
    
    print("关闭夹爪...")
    robot.set_gripper(0.0)  # 完全关闭
    print("✅ 夹爪已关闭")
    time.sleep(2)
    
    print("打开夹爪...")
    robot.set_gripper(1.0)
    print("✅ 夹爪控制测试完成")
    
    return True

def test_get_arm_pose_xy(robot):
    """测试6：获取机械臂末端坐标"""
    print("\n【测试6】获取机械臂末端坐标")
    print("-" * 40)
    
    print("读取机械臂末端坐标...")
    pose = robot.get_arm_pose_xy()
    print(f"📊 机械臂末端位置:")
    print(f"   X: {pose.get('x', 'N/A'):.3f} m")
    print(f"   Y: {pose.get('y', 'N/A'):.3f} m")
    print(f"   Z: {pose.get('z', 'N/A'):.3f} m")
    print("✅ 末端坐标读取完成")
    
    return True

def test_get_joint_states(robot):
    """测试7：获取关节状态"""
    print("\n【测试7】获取关节状态")
    print("-" * 40)
    
    print("读取所有关节状态...")
    states = robot.get_joint_states()
    print(f"📊 关节状态:")
    for joint_name, angle in states.items():
        print(f"   {joint_name}: {angle:.2f}°")
    print("✅ 关节状态读取完成")
    
    return True

def test_set_pwm(robot):
    """测试8：PWM控制"""
    print("\n【测试8】PWM控制")
    print("-" * 40)
    
    print("⚠️  PWM控制为底层接口，请谨慎使用")
    print("将测试一个关节的PWM输出")
    input("按回车键开始...")
    
    joint_id = 1
    pwm_value = 1500  # 中位值
    
    print(f"设置关节{joint_id}的PWM为{pwm_value}")
    robot.set_pwm(joint_id, pwm_value)
    print("✅ PWM已设置")
    time.sleep(2)
    
    print("恢复到归零位置...")
    robot.arm_home()
    
    return True

def test_comprehensive_demo(robot):
    """测试9：机械臂综合演示"""
    print("\n【测试9】机械臂综合演示")
    print("-" * 40)
    print("演示内容：机械臂抓取动作模拟")
    print("1. 归零 → 2. 移动到目标 → 3. 抓取 → 4. 提升 → 5. 放置 → 6. 归零")
    print("⚠️  请确保机械臂周围无障碍物")
    input("按回车键开始...")
    
    print("\n1️⃣  机械臂归零...")
    robot.arm_home()
    time.sleep(2)
    
    print("2️⃣  打开夹爪...")
    robot.set_gripper(1.0)
    time.sleep(1)
    
    print("3️⃣  移动到抓取位置...")
    robot.set_arm_position(0.2, 0.0, 0.05)
    time.sleep(2)
    
    print("4️⃣  关闭夹爪（抓取）...")
    robot.set_gripper(0.0)
    time.sleep(2)
    
    print("5️⃣  提升物体...")
    robot.set_arm_position(0.2, 0.0, 0.15)
    time.sleep(2)
    
    print("6️⃣  移动到放置位置...")
    robot.set_arm_position(0.15, 0.1, 0.15)
    time.sleep(2)
    
    print("7️⃣  放下物体...")
    robot.set_arm_position(0.15, 0.1, 0.05)
    time.sleep(2)
    
    print("8️⃣  打开夹爪（释放）...")
    robot.set_gripper(1.0)
    time.sleep(1)
    
    print("9️⃣  机械臂归零...")
    robot.arm_home()
    time.sleep(2)
    
    print("✅ 综合演示完成")
    print("机械臂抓取流程已展示")
    
    return True

def main():
    """主函数"""
    print("\n" + "="*50)
    print("机械臂控制功能测试程序")
    print("车型: 麦轮车 (mec) + 机械臂")
    print("="*50)
    print("⚠️  安全提示：")
    print("  - 测试前请确保机械臂周围无障碍物")
    print("  - 确保机械臂安装正确且供电正常")
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
                test_arm_home(robot)
            elif choice == '2':
                test_set_joint_angles(robot)
            elif choice == '3':
                test_set_yaw_angle(robot)
            elif choice == '4':
                test_set_arm_position(robot)
            elif choice == '5':
                test_set_gripper(robot)
            elif choice == '6':
                test_get_arm_pose_xy(robot)
            elif choice == '7':
                test_get_joint_states(robot)
            elif choice == '8':
                test_set_pwm(robot)
            elif choice == '9':
                test_comprehensive_demo(robot)
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
