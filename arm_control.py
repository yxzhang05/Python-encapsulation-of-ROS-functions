#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂控制示例程序
演示如何使用 robot_lib 进行机械臂控制
注意：此示例需要实际的机械臂硬件和ROS驱动支持
"""

from robot_lib import Robot
import time

def demo_basic_control(robot):
    """演示基本的机械臂控制"""
    print("\n" + "=" * 50)
    print("机械臂基本控制演示")
    print("=" * 50)
    
    # 1. 机械臂复位
    print("\n>>> 步骤1: 机械臂复位")
    if robot.arm_home():
        print("✓ 机械臂已复位到初始姿态")
    else:
        print("× 机械臂复位失败（可能需要机械臂驱动节点）")
    
    time.sleep(2)
    
    # 2. 设置关节角度
    print("\n>>> 步骤2: 设置关节角度")
    print("设置大臂=45°, 小臂=90°")
    robot.set_joint_angles(45, 90)
    time.sleep(3)
    
    print("设置大臂=90°, 小臂=45°")
    robot.set_joint_angles(90, 45)
    time.sleep(3)
    
    # 3. 设置云台角度
    print("\n>>> 步骤3: 设置云台角度")
    print("云台旋转到 45°")
    robot.set_yaw_angle(45)
    time.sleep(2)
    
    print("云台旋转到 -45°")
    robot.set_yaw_angle(-45)
    time.sleep(2)
    
    # 4. 回到初始位置
    print("\n>>> 步骤4: 回到初始位置")
    robot.arm_home()

def demo_position_control(robot):
    """演示位置控制（逆运动学）"""
    print("\n" + "=" * 50)
    print("机械臂位置控制演示（逆运动学）")
    print("=" * 50)
    
    # 定义一些目标位置
    positions = [
        (200, 100),   # 位置1
        (150, 150),   # 位置2
        (100, 200),   # 位置3
        (200, 0)      # 位置4
    ]
    
    print("\n机械臂将依次移动到以下位置（单位：mm）:")
    for i, (x, y) in enumerate(positions, 1):
        print(f"  {i}. X={x}mm, Y={y}mm")
    
    input("\n按回车键开始...")
    
    for i, (x, y) in enumerate(positions, 1):
        print(f"\n>>> 移动到位置 {i}: X={x}mm, Y={y}mm")
        robot.set_arm_position(x, y)
        time.sleep(3)
        
        # 获取当前位置确认
        pose = robot.get_arm_pose_xy()
        if pose:
            print(f"当前位置: {pose}")
    
    print("\n✓ 位置控制演示完成")

def demo_gripper_control(robot):
    """演示夹爪控制"""
    print("\n" + "=" * 50)
    print("夹爪控制演示")
    print("=" * 50)
    
    print("\n>>> 夹爪操作序列:")
    
    # 完全张开
    print("1. 完全张开夹爪 (10/10)")
    robot.set_gripper(10)
    time.sleep(2)
    
    # 半开
    print("2. 半开夹爪 (5/10)")
    robot.set_gripper(5)
    time.sleep(2)
    
    # 完全闭合
    print("3. 完全闭合夹爪 (0/10)")
    robot.set_gripper(0)
    time.sleep(2)
    
    # 张开
    print("4. 再次张开夹爪 (10/10)")
    robot.set_gripper(10)
    time.sleep(2)
    
    print("\n✓ 夹爪控制演示完成")

def demo_pick_and_place(robot):
    """演示抓取和放置流程"""
    print("\n" + "=" * 50)
    print("抓取和放置演示")
    print("=" * 50)
    
    print("\n模拟物体抓取和放置流程:")
    
    # 1. 复位
    print("\n>>> 步骤1: 机械臂复位")
    robot.arm_home()
    time.sleep(2)
    
    # 2. 张开夹爪
    print("\n>>> 步骤2: 张开夹爪")
    robot.set_gripper(10)
    time.sleep(1)
    
    # 3. 移动到抓取位置
    print("\n>>> 步骤3: 移动到物体位置 (150mm, 100mm)")
    robot.set_arm_position(150, 100)
    time.sleep(3)
    
    # 4. 闭合夹爪抓取
    print("\n>>> 步骤4: 闭合夹爪抓取物体")
    robot.set_gripper(0)
    time.sleep(2)
    
    # 5. 抬起
    print("\n>>> 步骤5: 抬起物体 (150mm, 200mm)")
    robot.set_arm_position(150, 200)
    time.sleep(3)
    
    # 6. 移动到放置位置
    print("\n>>> 步骤6: 移动到放置位置 (250mm, 150mm)")
    robot.set_arm_position(250, 150)
    time.sleep(3)
    
    # 7. 放置物体
    print("\n>>> 步骤7: 张开夹爪放置物体")
    robot.set_gripper(10)
    time.sleep(2)
    
    # 8. 回到初始位置
    print("\n>>> 步骤8: 回到初始位置")
    robot.arm_home()
    time.sleep(2)
    
    print("\n✓ 抓取和放置演示完成")

def demo_joint_states(robot):
    """演示关节状态读取"""
    print("\n" + "=" * 50)
    print("关节状态读取演示")
    print("=" * 50)
    
    print("\n>>> 获取当前关节状态:")
    states = robot.get_joint_states()
    if states:
        print(f"关节状态: {states}")
    else:
        print("× 无法获取关节状态（需要关节状态话题支持）")
    
    print("\n>>> 获取末端位置:")
    pose = robot.get_arm_pose_xy()
    if pose:
        print(f"末端位置: X={pose[0]}mm, Y={pose[1]}mm")
    else:
        print("× 无法获取末端位置（需要位置反馈支持）")

def main():
    """主函数"""
    print("=" * 50)
    print("机械臂控制示例程序")
    print("=" * 50)
    print("\n注意：此程序需要机械臂硬件和ROS驱动支持")
    print("如果没有实际硬件，部分功能将无法执行")
    
    # 创建机器人对象
    robot = Robot()
    
    try:
        # 初始化机器人底盘（机械臂通常也需要底盘初始化）
        print("\n>>> 初始化机器人")
        if not robot.initialize("mec"):  # 根据实际车型修改
            print("× 初始化失败，程序退出")
            return
        
        print("\n请选择演示功能:")
        print("1. 基本控制（关节角度控制）")
        print("2. 位置控制（逆运动学）")
        print("3. 夹爪控制")
        print("4. 抓取和放置流程")
        print("5. 读取关节状态")
        print("6. 全部演示（按顺序执行）")
        
        choice = input("\n请输入选项 (1-6): ").strip()
        
        if choice == '1':
            demo_basic_control(robot)
        elif choice == '2':
            demo_position_control(robot)
        elif choice == '3':
            demo_gripper_control(robot)
        elif choice == '4':
            demo_pick_and_place(robot)
        elif choice == '5':
            demo_joint_states(robot)
        elif choice == '6':
            # 全部演示
            demo_basic_control(robot)
            time.sleep(2)
            demo_position_control(robot)
            time.sleep(2)
            demo_gripper_control(robot)
            time.sleep(2)
            demo_pick_and_place(robot)
            time.sleep(2)
            demo_joint_states(robot)
        else:
            print("× 无效的选项")
        
        print("\n" + "=" * 50)
        print("✓ 机械臂控制示例完成！")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序执行出错: {e}")
    finally:
        # 安全关闭
        print("\n正在安全关闭系统...")
        robot.shutdown()
        print("程序结束")

if __name__ == "__main__":
    main()
