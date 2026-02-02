#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
建图与导航示例程序
演示如何使用 robot_lib 进行SLAM建图和自主导航
"""

from robot_lib import Robot
import time

def demo_mapping(robot):
    """演示建图功能"""
    print("\n" + "=" * 50)
    print("SLAM建图演示")
    print("=" * 50)
    
    # 启动建图
    print("\n>>> 步骤1: 选择建图算法")
    print("可用算法: gmapping, cartographer, slam_toolbox")
    method = input("请输入建图算法 (默认: slam_toolbox): ").strip()
    if not method:
        method = 'slam_toolbox'
    
    print(f"\n>>> 步骤2: 启动 {method} 建图")
    if not robot.start_mapping(method):
        print("× 建图启动失败")
        return False
    
    print("✓ 建图已启动")
    
    # 提示用户移动机器人
    print("\n>>> 步骤3: 移动机器人进行建图")
    print("现在需要移动机器人来建立地图")
    print("选项:")
    print("  1. 使用键盘控制 (手动)")
    print("  2. 使用自动移动 (示例)")
    
    choice = input("请选择 (1/2): ").strip()
    
    if choice == '1':
        # 键盘控制
        print("\n启动键盘控制...")
        print("使用键盘控制机器人移动，建立地图")
        print("按 Ctrl+C 结束建图")
        try:
            robot.start_keyboard_control()
        except KeyboardInterrupt:
            print("\n键盘控制已结束")
    
    elif choice == '2':
        # 自动移动示例
        print("\n执行自动移动建图...")
        print("机器人将执行简单的移动模式（前进-旋转-前进）")
        
        # 简单的移动模式
        for i in range(3):
            print(f"\n第 {i+1} 轮移动:")
            
            # 前进
            print("  前进 1米...")
            robot.move_distance(1.0, 0.3)
            time.sleep(1)
            
            # 旋转
            print("  旋转 90度...")
            robot.rotate_angle(90)
            time.sleep(1)
        
        print("\n✓ 自动移动完成")
    
    # 保存地图
    print("\n>>> 步骤4: 保存地图")
    map_name = input("请输入地图名称 (默认: my_map): ").strip()
    if not map_name:
        map_name = 'my_map'
    
    if robot.save_map(map_name):
        print(f"✓ 地图已保存: {map_name}.pgm 和 {map_name}.yaml")
        return map_name
    else:
        print("× 地图保存失败")
        return None

def demo_navigation(robot, map_name=None):
    """演示导航功能"""
    print("\n" + "=" * 50)
    print("自主导航演示")
    print("=" * 50)
    
    # 如果没有地图名，询问用户
    if not map_name:
        map_name = input("\n请输入地图文件名 (不含扩展名): ").strip()
        if not map_name:
            print("× 未指定地图文件")
            return
    
    map_file = f"{map_name}.yaml"
    
    # 启动导航
    print(f"\n>>> 步骤1: 启动导航 (地图: {map_file})")
    if not robot.start_navigation(map_file):
        print("× 导航启动失败")
        return
    
    print("✓ 导航已启动")
    print("提示: 在RViz中使用 '2D Pose Estimate' 设置机器人初始位置")
    
    input("\n按回车键继续发送导航目标...")
    
    # 发送导航目标
    print("\n>>> 步骤2: 发送导航目标点")
    
    # 示例目标点
    goals = [
        (2.0, 0.0, 0.0),    # 前方2米
        (2.0, 2.0, 90.0),   # 右前方
        (0.0, 2.0, 180.0),  # 右侧
        (0.0, 0.0, -90.0)   # 回到起点
    ]
    
    print("将依次导航到以下目标点:")
    for i, (x, y, theta) in enumerate(goals, 1):
        print(f"  {i}. X={x}m, Y={y}m, 朝向={theta}°")
    
    input("\n按回车键开始导航...")
    
    for i, (x, y, theta) in enumerate(goals, 1):
        print(f"\n>>> 导航到目标点 {i}/{len(goals)}")
        if robot.move_to_goal(x, y, theta):
            print(f"✓ 目标点已发送: X={x}m, Y={y}m")
            print("等待机器人到达目标...")
            time.sleep(10)  # 等待导航完成
        else:
            print("× 发送目标点失败")
    
    print("\n✓ 所有导航任务完成")

def demo_auto_mapping_and_navigation(robot):
    """演示完整的建图+导航流程"""
    print("\n" + "=" * 50)
    print("完整流程演示: 建图 -> 保存 -> 导航")
    print("=" * 50)
    
    # 1. 建图
    print("\n### 第一阶段: 建图 ###")
    map_name = demo_mapping(robot)
    
    if not map_name:
        print("× 建图失败，无法继续导航")
        return
    
    # 等待用户准备
    input("\n建图完成！按回车键继续启动导航...")
    
    # 2. 导航
    print("\n### 第二阶段: 导航 ###")
    demo_navigation(robot, map_name)
    
    print("\n✓ 完整流程演示完成！")

def main():
    """主函数"""
    print("=" * 50)
    print("建图与导航示例程序")
    print("=" * 50)
    
    # 创建机器人对象
    robot = Robot()
    
    try:
        # 初始化机器人底盘
        print("\n>>> 初始化机器人底盘")
        if not robot.initialize("mec"):  # 根据实际车型修改
            print("× 初始化失败，程序退出")
            return
        
        # 启动雷达（建图和导航都需要）
        print("\n>>> 启动雷达")
        if not robot.launch_lidar():
            print("× 雷达启动失败，程序退出")
            return
        
        print("\n请选择功能:")
        print("1. 仅建图")
        print("2. 仅导航")
        print("3. 完整流程 (建图 + 导航)")
        
        choice = input("\n请输入选项 (1-3): ").strip()
        
        if choice == '1':
            demo_mapping(robot)
        elif choice == '2':
            demo_navigation(robot)
        elif choice == '3':
            demo_auto_mapping_and_navigation(robot)
        else:
            print("× 无效的选项")
        
        print("\n" + "=" * 50)
        print("✓ 程序执行完成！")
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
