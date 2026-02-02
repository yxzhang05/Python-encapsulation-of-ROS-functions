#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速入门示例 - 演示最基本的机器人控制
适合初次使用者快速体验
"""

from robot_lib import Robot
import time

def main():
    """快速入门示例"""
    
    print("=" * 50)
    print("ROS2机器人Python封装 - 快速入门")
    print("=" * 50)
    
    # 步骤1: 创建机器人对象
    print("\n[1] 创建机器人对象...")
    robot = Robot()
    
    try:
        # 步骤2: 初始化（根据你的车型修改）
        print("[2] 初始化机器人...")
        print("    提示: 车型选项 'akm'(阿克曼) 'diff'(差速) 'mec'(麦轮)")
        
        # 这里使用麦轮，请根据实际车型修改
        robot_type = "mec"
        
        if not robot.initialize(robot_type):
            print("    ✗ 初始化失败！请检查:")
            print("      - ROS2环境是否正确配置")
            print("      - 机器人硬件是否连接")
            print("      - 设备权限是否正确")
            return
        
        print(f"    ✓ {robot_type} 车型初始化成功！")
        
        # 步骤3: 检查电压
        print("\n[3] 检查电池电压...")
        voltage = robot.get_battery_voltage()
        if voltage > 0:
            print(f"    ✓ 当前电压: {voltage:.2f}V")
            if voltage < 10.5:
                print("    ⚠️  警告: 电量低，请充电！")
        else:
            print("    ℹ️  无法获取电压信息（话题可能不存在）")
        
        # 步骤4: 基本移动测试
        print("\n[4] 基本移动测试...")
        print("    测试1: 设置速度（前进2秒）")
        robot.set_velocity(0.2, 0, 0)  # 以0.2m/s前进
        time.sleep(2)
        robot.set_velocity(0, 0, 0)    # 停止
        print("    ✓ 速度控制测试完成")
        
        time.sleep(1)
        
        print("\n    测试2: 原地旋转（45度）")
        robot.rotate_angle(45, speed=0.3)
        print("    ✓ 旋转控制测试完成")
        
        # 步骤5: 完成
        print("\n[5] 测试完成！")
        print("=" * 50)
        print("✓ 快速入门示例运行成功！")
        print("\n接下来可以尝试:")
        print("  • python3 chassis_control.py  - 完整底盘控制")
        print("  • python3 sensor_app.py       - 传感器应用")
        print("  • python3 mapping_app.py      - 建图导航")
        print("  • python3 arm_control.py      - 机械臂控制")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n\n用户中断程序")
    except Exception as e:
        print(f"\n\n✗ 程序执行出错: {e}")
        print("请检查:")
        print("  1. ROS2环境是否正确配置")
        print("  2. 机器人是否正确连接")
        print("  3. launch文件是否存在")
    finally:
        # 步骤6: 安全关闭
        print("\n[6] 安全关闭系统...")
        robot.shutdown()
        print("✓ 程序结束\n")

if __name__ == "__main__":
    main()
