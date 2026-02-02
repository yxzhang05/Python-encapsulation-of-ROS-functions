#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘运动控制示例程序
演示如何使用 robot_lib 进行底盘运动控制
"""

from robot_lib import Robot
import time

def main():
    """主函数"""
    print("=" * 50)
    print("底盘运动控制示例")
    print("=" * 50)
    
    # 1. 创建机器人对象
    robot = Robot()
    
    try:
        # 2. 初始化机器人（根据实际车型修改：akm/diff/mec）
        print("\n>>> 步骤1: 初始化机器人底盘")
        robot_type = "mec"  # 麦轮车型，可改为 "akm" 或 "diff"
        if not robot.initialize(robot_type):
            print("初始化失败，程序退出")
            return
        
        # 3. 检查电池电压
        print("\n>>> 步骤2: 检查电池电压")
        voltage = robot.get_battery_voltage()
        print(f"当前电池电压: {voltage:.2f}V")
        if voltage < 10.5 and voltage > 0:
            print("⚠️  警告：电量过低，请及时充电！")
        
        # 4. 演示基础速度控制
        print("\n>>> 步骤3: 演示速度控制")
        print("机器人将以 0.2m/s 前进 2秒...")
        robot.set_velocity(0.2, 0.0, 0.0)
        time.sleep(2)
        robot.set_velocity(0.0, 0.0, 0.0)  # 停止
        time.sleep(1)
        
        # 5. 演示旋转控制
        print("\n>>> 步骤4: 演示原地旋转")
        print("机器人将原地旋转 90度...")
        robot.rotate_angle(90, speed=0.5)
        time.sleep(1)
        
        # 6. 演示距离控制（仅当里程计可用时）
        print("\n>>> 步骤5: 演示距离移动")
        print("机器人将前进 0.5米...")
        result = robot.move_distance(0.5, 0.3)
        if result:
            print("✓ 距离移动完成")
        else:
            print("× 距离移动失败（可能里程计不可用）")
        time.sleep(1)
        
        # 7. 演示麦轮横向移动（仅麦轮有效）
        if robot_type == "mec":
            print("\n>>> 步骤6: 演示麦轮横向移动")
            print("麦轮机器人将横向移动...")
            robot.set_velocity(0.0, 0.2, 0.0)  # 横向速度
            time.sleep(2)
            robot.set_velocity(0.0, 0.0, 0.0)
            time.sleep(1)
        
        # 8. 获取机器人位姿信息
        print("\n>>> 步骤7: 获取机器人位姿")
        pose = robot.get_robot_pose()
        if pose:
            x, y, yaw = pose
            print(f"当前位置: X={x:.3f}m, Y={y:.3f}m")
            print(f"当前朝向: {yaw:.3f}弧度")
        
        # 9. 演示急停功能
        print("\n>>> 步骤8: 演示急停功能")
        print("发送急停指令...")
        robot.emergency_stop()
        
        print("\n" + "=" * 50)
        print("✓ 底盘控制示例完成！")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序执行出错: {e}")
    finally:
        # 10. 安全关闭
        print("\n正在安全关闭系统...")
        robot.shutdown()
        print("程序结束")

if __name__ == "__main__":
    main()
