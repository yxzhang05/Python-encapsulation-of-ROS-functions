#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器应用示例程序
演示如何使用雷达和相机，以及视觉/雷达跟随功能
"""

from robot_lib import Robot
import time

def demo_lidar(robot):
    """演示雷达功能"""
    print("\n" + "=" * 50)
    print("雷达功能演示")
    print("=" * 50)
    
    # 启动雷达
    print("\n>>> 步骤1: 启动雷达")
    if robot.launch_lidar():
        print("✓ 雷达已启动，数据发布在 /scan 话题")
        
        # 等待雷达稳定
        print("等待雷达数据稳定...")
        time.sleep(3)
        
        # 获取雷达数据
        print("\n>>> 步骤2: 获取雷达数据")
        distance = robot.get_lidar_distance(0)  # 获取正前方距离
        if distance:
            print("✓ 雷达数据获取成功")
        
        # 演示雷达跟随
        print("\n>>> 步骤3: 启动雷达跟随（5秒后自动停止）")
        print("请在机器人前方移动，机器人会跟随您")
        if robot.start_lidar_follow(target_dist=0.8):
            print("✓ 雷达跟随已启动，保持距离: 0.8米")
            time.sleep(5)
            
            # 停止跟随
            print("\n停止雷达跟随...")
            robot.stop_application()
            print("✓ 雷达跟随已停止")
        
        # 关闭雷达
        print("\n>>> 步骤4: 关闭雷达")
        robot.stop_lidar()
        print("✓ 雷达已关闭")
    else:
        print("× 雷达启动失败")

def demo_camera(robot):
    """演示相机功能"""
    print("\n" + "=" * 50)
    print("相机功能演示")
    print("=" * 50)
    
    # 启动相机
    print("\n>>> 步骤1: 启动相机")
    if robot.launch_camera():
        print("✓ 相机已启动，图像发布在 /camera/image_raw 话题")
        
        # 等待相机稳定
        print("等待相机稳定...")
        time.sleep(3)
        
        # 拍照
        print("\n>>> 步骤2: 拍照")
        frame = robot.capture_camera_frame()
        print("✓ 拍照功能调用（需要额外的图像保存节点）")
        
        # 关闭相机
        print("\n>>> 步骤3: 关闭相机")
        robot.stop_camera()
        print("✓ 相机已关闭")
    else:
        print("× 相机启动失败")

def demo_visual_follow(robot):
    """演示视觉跟随功能"""
    print("\n" + "=" * 50)
    print("视觉跟随功能演示")
    print("=" * 50)
    
    # 确保相机已启动
    print("\n>>> 步骤1: 启动相机")
    if not robot.launch_camera():
        print("× 相机启动失败，无法进行视觉跟随")
        return
    
    time.sleep(3)
    
    # 启动视觉跟随
    print("\n>>> 步骤2: 启动视觉跟随（红色目标）")
    print("请在机器人前方放置红色物体")
    if robot.start_visual_follow('red'):
        print("✓ 视觉跟随已启动，目标颜色: 红色")
        print("机器人将跟随红色物体（10秒后自动停止）")
        time.sleep(10)
        
        # 停止跟随
        print("\n>>> 步骤3: 停止视觉跟随")
        robot.stop_application()
        print("✓ 视觉跟随已停止")
    else:
        print("× 视觉跟随启动失败")
    
    # 关闭相机
    print("\n>>> 步骤4: 关闭相机")
    robot.stop_camera()

def demo_line_tracking(robot):
    """演示视觉巡线功能"""
    print("\n" + "=" * 50)
    print("视觉巡线功能演示")
    print("=" * 50)
    
    # 确保相机已启动
    print("\n>>> 步骤1: 启动相机")
    if not robot.launch_camera():
        print("× 相机启动失败，无法进行视觉巡线")
        return
    
    time.sleep(3)
    
    # 启动视觉巡线
    print("\n>>> 步骤2: 启动视觉巡线（黑线）")
    print("请确保地面有黑色线条")
    if robot.start_line_tracking('black'):
        print("✓ 视觉巡线已启动，线条颜色: 黑色")
        print("机器人将沿着黑线行驶（10秒后自动停止）")
        time.sleep(10)
        
        # 停止巡线
        print("\n>>> 步骤3: 停止视觉巡线")
        robot.stop_application()
        print("✓ 视觉巡线已停止")
    else:
        print("× 视觉巡线启动失败")
    
    # 关闭相机
    print("\n>>> 步骤4: 关闭相机")
    robot.stop_camera()

def main():
    """主函数"""
    print("=" * 50)
    print("传感器应用示例程序")
    print("=" * 50)
    
    # 创建机器人对象
    robot = Robot()
    
    try:
        # 初始化机器人底盘
        print("\n>>> 初始化机器人底盘")
        if not robot.initialize("mec"):  # 根据实际车型修改
            print("× 初始化失败，程序退出")
            return
        
        print("\n请选择要演示的功能:")
        print("1. 雷达功能演示")
        print("2. 相机功能演示")
        print("3. 视觉跟随演示")
        print("4. 视觉巡线演示")
        print("5. 全部演示（按顺序执行）")
        
        choice = input("\n请输入选项 (1-5): ").strip()
        
        if choice == '1':
            demo_lidar(robot)
        elif choice == '2':
            demo_camera(robot)
        elif choice == '3':
            demo_visual_follow(robot)
        elif choice == '4':
            demo_line_tracking(robot)
        elif choice == '5':
            # 全部演示
            demo_lidar(robot)
            time.sleep(2)
            demo_camera(robot)
            time.sleep(2)
            demo_visual_follow(robot)
            time.sleep(2)
            demo_line_tracking(robot)
        else:
            print("× 无效的选项")
        
        print("\n" + "=" * 50)
        print("✓ 传感器应用示例完成！")
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
