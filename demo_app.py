#!/usr/bin/env python3
# 文件名: demo_app.py
# 功能：综合演示应用
# 说明：展示如何组合使用多个功能模块完成复杂任务

from robot_lib_full import Robot
import time
import sys

def demo_complete_workflow():
    """
    完整工作流程演示
    
    展示一个完整的机器人应用场景：
    1. 系统初始化
    2. 传感器启动
    3. 建图
    4. 导航
    5. 视觉应用
    6. 机械臂操作
    """
    print("\n" + "="*60)
    print("完整工作流程演示")
    print("="*60)
    
    robot = Robot()
    
    try:
        # 1. 初始化
        print("\n【步骤 1】初始化机器人系统...")
        if not robot.initialize("mec"):
            print("初始化失败，退出演示")
            return
        
        robot.print_status()
        
        # 2. 启动传感器
        print("\n【步骤 2】启动传感器...")
        print("  启动雷达...")
        robot.sensors.launch_lidar(visualize=True)
        time.sleep(2)
        
        print("  启动相机...")
        robot.sensors.launch_camera(visualize=True)
        time.sleep(2)
        
        # 3. 环境感知测试
        print("\n【步骤 3】环境感知测试...")
        for i in range(5):
            dist_front = robot.sensors.get_lidar_distance(0)
            dist_left = robot.sensors.get_lidar_distance(90)
            dist_right = robot.sensors.get_lidar_distance(-90)
            
            voltage = robot.get_battery_voltage()
            
            print(f"  [{i+1}] 前:{dist_front:.2f}m 左:{dist_left:.2f}m "
                  f"右:{dist_right:.2f}m 电压:{voltage:.2f}V")
            
            time.sleep(1)
        
        # 4. 简单移动测试
        print("\n【步骤 4】简单移动测试...")
        print("  前进 0.5 米...")
        robot.forward(0.5, 0.2)
        time.sleep(1)
        
        print("  左转 90 度...")
        robot.turn(90, 0.3)
        time.sleep(1)
        
        print("  后退 0.5 米...")
        robot.forward(-0.5, 0.2)
        time.sleep(1)
        
        print("  右转 90 度...")
        robot.turn(-90, 0.3)
        
        # 5. 机械臂测试（如果有）
        print("\n【步骤 5】机械臂测试...")
        try:
            print("  机械臂复位...")
            robot.arm.arm_home()
            time.sleep(2)
            
            print("  移动到测试位置...")
            robot.arm.set_joint_angles(30, -20)
            time.sleep(2)
            
            print("  测试夹爪...")
            robot.arm.set_gripper(0)   # 闭合
            time.sleep(1)
            robot.arm.set_gripper(10)  # 打开
            time.sleep(1)
            
            print("  复位...")
            robot.arm.arm_home()
        except Exception as e:
            print(f"  机械臂测试跳过: {e}")
        
        # 6. 最终状态
        print("\n【步骤 6】最终状态检查...")
        robot.print_status()
        
        print("\n✅ 完整工作流程演示完成！")
        
    except KeyboardInterrupt:
        print("\n\n用户中断演示")
    
    finally:
        robot.shutdown()


def demo_autonomous_exploration():
    """
    自主探索演示
    
    机器人自动探索环境并避障
    """
    print("\n" + "="*60)
    print("自主探索演示")
    print("="*60)
    
    robot = Robot()
    
    try:
        # 初始化
        if not robot.initialize("diff"):
            return
        
        # 启动雷达
        robot.sensors.launch_lidar(visualize=True)
        time.sleep(3)
        
        print("\n开始自主探索（按 Ctrl+C 停止）...\n")
        
        step = 0
        while step < 20:  # 最多探索 20 步
            step += 1
            
            # 感知环境
            dist_front = robot.sensors.get_lidar_distance(0)
            dist_left = robot.sensors.get_lidar_distance(45)
            dist_right = robot.sensors.get_lidar_distance(-45)
            
            print(f"[步骤 {step:2d}] 前:{dist_front:.2f}m 左:{dist_left:.2f}m 右:{dist_right:.2f}m", end=" ")
            
            # 决策
            if dist_front > 1.5:
                # 前方安全，前进
                print("→ 前进")
                robot.move(0.3, 0, 0)
                time.sleep(2)
            elif dist_front > 0.8:
                # 接近障碍，减速前进
                print("→ 减速")
                robot.move(0.1, 0, 0)
                time.sleep(1)
            else:
                # 太近，选择转向
                if dist_left > dist_right:
                    print("↶ 左转")
                    robot.turn(45, 0.5)
                else:
                    print("↷ 右转")
                    robot.turn(-45, 0.5)
                time.sleep(1)
            
            robot.stop()
            time.sleep(0.5)
        
        print("\n✅ 探索完成！")
        
    except KeyboardInterrupt:
        print("\n\n探索被中断")
        robot.stop()
    
    finally:
        robot.shutdown()


def demo_visual_tracking_with_lidar():
    """
    视觉跟随 + 雷达避障演示
    
    结合视觉跟随和雷达避障功能
    """
    print("\n" + "="*60)
    print("视觉跟随 + 雷达避障演示")
    print("="*60)
    
    robot = Robot()
    
    try:
        # 初始化
        if not robot.initialize("mec"):
            return
        
        # 启动传感器
        print("\n启动传感器...")
        robot.sensors.launch_camera(visualize=True)
        robot.sensors.launch_lidar(visualize=False)
        time.sleep(3)
        
        # 选择跟随颜色
        colors = ['red', 'blue', 'green', 'yellow']
        print("\n可用颜色：")
        for i, color in enumerate(colors, 1):
            print(f"  {i}. {color}")
        
        choice = input("\n选择跟随颜色 (1-4): ").strip()
        if choice not in ['1', '2', '3', '4']:
            print("无效选择")
            return
        
        color = colors[int(choice) - 1]
        
        # 自定义跟随逻辑（结合雷达避障）
        print(f"\n开始跟随 {color} 颜色物体（带避障）...")
        print("按 Ctrl+C 停止\n")
        
        # 这里我们手动控制而不是让应用自动控制
        # 这样可以加入避障逻辑
        robot.sensors.start_visual_follow(
            color,
            control_robot=False  # 不自动控制
        )
        
        while True:
            # 检查前方是否有障碍
            dist = robot.sensors.get_lidar_distance(0)
            
            if dist < 0.5:
                # 太近，停止
                print(f"⚠️  前方障碍物！距离: {dist:.2f}m")
                robot.stop()
            else:
                # 安全，可以移动
                # 实际应用中应该从视觉跟随获取目标信息
                # 这里简化为持续前进
                print(f"✅ 安全距离: {dist:.2f}m")
            
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\n\n停止跟随")
        robot.sensors.stop_application()
        robot.stop()
    
    finally:
        robot.shutdown()


def demo_mapping_and_navigation():
    """
    建图和导航演示
    
    先建图，然后在地图中导航
    """
    print("\n" + "="*60)
    print("建图和导航演示")
    print("="*60)
    
    robot = Robot()
    
    try:
        # 初始化
        if not robot.initialize("mec"):
            return
        
        # 询问用户选择
        print("\n请选择操作：")
        print("  1. 建图")
        print("  2. 导航")
        
        choice = input("\n请输入选项 (1-2): ").strip()
        
        if choice == "1":
            # 建图模式
            print("\n【建图模式】")
            
            robot.sensors.launch_lidar(visualize=False)
            time.sleep(2)
            
            robot.navigation.start_mapping("gmapping", visualize=True)
            time.sleep(5)
            
            print("\n开始键盘控制建图...")
            print("建图完成后按 Ctrl+C 继续")
            
            try:
                robot.navigation.start_keyboard_control()
            except KeyboardInterrupt:
                pass
            
            # 保存地图
            map_name = input("\n输入地图名称（默认: demo_map）: ").strip()
            if not map_name:
                map_name = "demo_map"
            
            robot.navigation.save_map(map_name)
            print(f"\n✅ 地图已保存: maps/{map_name}")
            
        elif choice == "2":
            # 导航模式
            print("\n【导航模式】")
            
            import os
            maps_dir = "maps"
            if os.path.exists(maps_dir):
                maps = [f.replace(".yaml", "") for f in os.listdir(maps_dir) if f.endswith(".yaml")]
                if maps:
                    print("\n可用地图：")
                    for i, m in enumerate(maps, 1):
                        print(f"  {i}. {m}")
                    
                    choice = input("\n选择地图编号: ").strip()
                    if choice.isdigit() and 1 <= int(choice) <= len(maps):
                        map_name = maps[int(choice) - 1]
                    else:
                        print("无效选择")
                        return
                else:
                    print("没有可用的地图，请先建图")
                    return
            else:
                print("地图目录不存在，请先建图")
                return
            
            # 加载地图
            if robot.navigation.load_map_and_start_navigation(map_name):
                print("\n导航系统已启动")
                print("可以在 RViz 中使用 2D Goal Pose 工具设置目标")
                print("或输入坐标导航")
                
                try:
                    while True:
                        x = float(input("\n目标 X 坐标（输入 q 退出）: "))
                        y = float(input("目标 Y 坐标: "))
                        
                        robot.navigation.move_to_goal(x, y, 0)
                        
                        print("导航中...")
                        time.sleep(15)
                        
                except ValueError:
                    print("退出导航")
        
        else:
            print("无效选项")
        
    except KeyboardInterrupt:
        print("\n\n演示被中断")
    
    finally:
        robot.shutdown()


def main():
    """主菜单"""
    demos = {
        "1": ("完整工作流程演示", demo_complete_workflow),
        "2": ("自主探索演示", demo_autonomous_exploration),
        "3": ("视觉跟随+避障演示", demo_visual_tracking_with_lidar),
        "4": ("建图和导航演示", demo_mapping_and_navigation),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 功能 Python 封装 - 综合演示")
        print("="*60)
        print("\n请选择演示：")
        print()
        
        for key, (name, _) in demos.items():
            print(f"  {key}. {name}")
        
        print("  0. 退出程序")
        print()
        
        choice = input("请输入选项 (0-4): ").strip()
        
        if choice == "0":
            print("\n再见！")
            break
        
        if choice in demos:
            name, func = demos[choice]
            try:
                func()
                input("\n按 Enter 键继续...")
            except KeyboardInterrupt:
                print("\n\n演示被中断")
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
