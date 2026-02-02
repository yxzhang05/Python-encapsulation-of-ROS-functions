#!/usr/bin/env python3
# 文件名: navigation_app.py
# 功能：建图与导航功能示例应用
# 说明：演示如何使用 robot_lib_navigation.py 中的建图导航功能

from robot_lib_navigation import RobotNavigation
from robot_lib_system import RobotSystem
from robot_lib_motion import RobotMotion
from robot_lib_sensors import RobotSensors
import time
import sys

def example_mapping_basic():
    """
    示例1：基本建图流程
    
    演示如何进行 SLAM 建图并保存地图
    """
    print("\n" + "="*60)
    print("示例1：基本建图流程")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        nav = RobotNavigation()
        sensors = RobotSensors()
        
        # 1. 启动雷达（建图需要）
        print("\n1. 启动雷达...")
        sensors.launch_lidar(visualize=False)
        time.sleep(3)
        
        # 2. 启动建图
        print("\n2. 启动 gmapping 建图...")
        if not nav.start_mapping("gmapping", visualize=True):
            return
        
        # 3. 键盘控制建图
        print("\n3. 现在可以用键盘控制机器人移动来建图")
        print("   建图完成后按 Ctrl+C 继续...")
        
        try:
            nav.start_keyboard_control()
        except KeyboardInterrupt:
            pass
        
        # 4. 保存地图
        print("\n4. 保存地图...")
        map_name = input("   输入地图名称（默认: test_map）: ").strip()
        if not map_name:
            map_name = "test_map"
        
        if nav.save_map(map_name):
            print(f"\n✅ 地图已保存: maps/{map_name}.pgm/.yaml")
        
        # 5. 清理
        nav.shutdown()
        sensors.stop_lidar()
        
    finally:
        robot_sys.shutdown()


def example_auto_mapping():
    """
    示例2：自动建图
    
    演示如何让机器人自动移动并建图（无需手动控制）
    """
    print("\n" + "="*60)
    print("示例2：自动建图")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        nav = RobotNavigation()
        sensors = RobotSensors()
        motion = RobotMotion("mec")
        
        # 启动传感器和建图
        print("\n启动雷达和建图...")
        sensors.launch_lidar(visualize=False)
        time.sleep(2)
        nav.start_mapping("gmapping", visualize=True)
        time.sleep(5)
        
        print("\n开始自动建图（机器人将自动移动）...")
        print("按 Ctrl+C 停止\n")
        
        # 简单的自动探索策略：走正方形
        try:
            for lap in range(2):  # 走两圈
                print(f"\n第 {lap+1} 圈...")
                
                # 前进
                print("  前进...")
                motion.move_distance_mecanum(2.0, 0.0, 0.3, 0.0)
                time.sleep(1)
                
                # 左转90度
                motion.rotate_angle(90, 0.5)
                time.sleep(1)
                
                # 前进
                print("  前进...")
                motion.move_distance_mecanum(2.0, 0.0, 0.3, 0.0)
                time.sleep(1)
                
                # 左转90度
                motion.rotate_angle(90, 0.5)
                time.sleep(1)
                
                # 前进
                print("  前进...")
                motion.move_distance_mecanum(2.0, 0.0, 0.3, 0.0)
                time.sleep(1)
                
                # 左转90度
                motion.rotate_angle(90, 0.5)
                time.sleep(1)
                
                # 前进
                print("  前进...")
                motion.move_distance_mecanum(2.0, 0.0, 0.3, 0.0)
                time.sleep(1)
                
                # 左转90度
                motion.rotate_angle(90, 0.5)
                time.sleep(1)
            
            print("\n自动建图完成！")
            
            # 保存地图
            print("\n保存地图...")
            nav.save_map("auto_map")
            
        except KeyboardInterrupt:
            print("\n\n建图被中断")
        
        # 停止
        motion.stop()
        nav.shutdown()
        sensors.stop_lidar()
        
    finally:
        robot_sys.shutdown()


def example_navigation_basic():
    """
    示例3：基本导航
    
    演示如何加载地图并导航到目标点
    """
    print("\n" + "="*60)
    print("示例3：基本导航")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        nav = RobotNavigation()
        
        # 1. 列出可用地图
        import os
        maps_dir = "maps"
        if os.path.exists(maps_dir):
            maps = [f.replace(".yaml", "") for f in os.listdir(maps_dir) if f.endswith(".yaml")]
            if maps:
                print("\n可用地图：")
                for i, m in enumerate(maps, 1):
                    print(f"  {i}. {m}")
            else:
                print("\n[错误] 没有可用的地图")
                print("[提示] 请先运行建图示例创建地图")
                return
        else:
            print("\n[错误] 地图目录不存在")
            return
        
        # 2. 选择地图
        choice = input("\n选择地图编号（或输入地图名称）: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(maps):
            map_name = maps[int(choice) - 1]
        else:
            map_name = choice
        
        # 3. 加载地图并启动导航
        print(f"\n加载地图: {map_name}")
        if not nav.load_map_and_start_navigation(map_name, visualize=True):
            return
        
        # 4. 发送导航目标
        print("\n发送导航目标...")
        print("（也可以在 RViz 中使用 2D Goal Pose 工具设置目标）")
        
        targets = [
            (2.0, 1.0, 0),
            (3.0, -1.0, 90),
            (0.0, 0.0, 180),
        ]
        
        for i, (x, y, theta) in enumerate(targets, 1):
            print(f"\n目标 {i}: ({x}, {y}, {theta}°)")
            nav.move_to_goal(x, y, theta)
            
            # 等待到达（简单等待，实际应该监控状态）
            print("  等待机器人到达...")
            time.sleep(20)
            
            status = nav.get_navigation_status()
            print(f"  状态: {status}")
        
        print("\n✅ 所有导航目标完成！")
        
        # 清理
        nav.shutdown()
        
    finally:
        robot_sys.shutdown()


def example_navigation_with_monitoring():
    """
    示例4：带监控的导航
    
    演示如何在导航过程中监控机器人状态
    """
    print("\n" + "="*60)
    print("示例4：带监控的导航")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        nav = RobotNavigation()
        motion = RobotMotion("mec")
        sensors = RobotSensors()
        
        # 加载地图
        map_name = input("输入地图名称（默认: test_map）: ").strip()
        if not map_name:
            map_name = "test_map"
        
        # 启动雷达（用于监控）
        sensors.launch_lidar(visualize=False)
        time.sleep(2)
        
        # 启动导航
        if not nav.load_map_and_start_navigation(map_name):
            return
        
        # 发送目标
        x = float(input("目标 X 坐标: "))
        y = float(input("目标 Y 坐标: "))
        
        print(f"\n导航到 ({x}, {y})...")
        nav.move_to_goal(x, y)
        
        # 持续监控
        print("\n持续监控导航状态...")
        print("按 Ctrl+C 取消导航\n")
        
        try:
            while nav.get_navigation_status() == "navigating":
                # 获取位姿
                pose = motion.get_robot_pose()
                if pose:
                    # 计算距离目标的距离
                    import math
                    dist_to_goal = math.sqrt((x - pose['x'])**2 + (y - pose['y'])**2)
                    
                    # 获取前方障碍物距离
                    dist_front = sensors.get_lidar_distance(0)
                    
                    # 显示状态
                    print(f"位置: ({pose['x']:.2f}, {pose['y']:.2f}) | "
                          f"距目标: {dist_to_goal:.2f}m | "
                          f"前方: {dist_front:.2f}m")
                    
                    # 如果接近目标
                    if dist_to_goal < 0.2:
                        print("\n✅ 已到达目标！")
                        break
                
                time.sleep(1)
            
        except KeyboardInterrupt:
            print("\n\n取消导航...")
            nav.cancel_navigation()
        
        # 清理
        nav.shutdown()
        sensors.stop_lidar()
        
    finally:
        robot_sys.shutdown()


def example_waypoint_navigation():
    """
    示例5：多点巡航
    
    演示机器人按顺序访问多个目标点
    """
    print("\n" + "="*60)
    print("示例5：多点巡航")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        nav = RobotNavigation()
        
        # 加载地图
        map_name = input("输入地图名称（默认: test_map）: ").strip()
        if not map_name:
            map_name = "test_map"
        
        if not nav.load_map_and_start_navigation(map_name):
            return
        
        # 定义巡航路径
        waypoints = [
            (1.0, 1.0, 0, "点1"),
            (2.0, 1.0, 90, "点2"),
            (2.0, 2.0, 180, "点3"),
            (1.0, 2.0, 270, "点4"),
            (0.0, 0.0, 0, "原点"),
        ]
        
        print(f"\n开始多点巡航（共 {len(waypoints)} 个点）...")
        
        for i, (x, y, theta, name) in enumerate(waypoints, 1):
            print(f"\n[{i}/{len(waypoints)}] 前往 {name}: ({x}, {y}, {theta}°)")
            
            if nav.move_to_goal(x, y, theta):
                # 等待到达
                print("  导航中...", end="", flush=True)
                for _ in range(15):  # 最多等待15秒
                    time.sleep(1)
                    print(".", end="", flush=True)
                print()
                
                print(f"  ✅ 到达 {name}")
                time.sleep(2)  # 在每个点停留2秒
            else:
                print(f"  ❌ 无法到达 {name}")
        
        print("\n✅ 多点巡航完成！")
        
        # 清理
        nav.shutdown()
        
    finally:
        robot_sys.shutdown()


def example_exploration_mapping():
    """
    示例6：探索式建图
    
    演示结合雷达数据的智能探索建图
    """
    print("\n" + "="*60)
    print("示例6：探索式建图")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("diff"):
        return
    
    try:
        nav = RobotNavigation()
        sensors = RobotSensors()
        motion = RobotMotion("diff")
        
        # 启动建图
        sensors.launch_lidar(visualize=False)
        time.sleep(2)
        nav.start_mapping("gmapping", visualize=True)
        time.sleep(5)
        
        print("\n开始智能探索建图...")
        print("机器人将自动避开障碍物并探索环境")
        print("按 Ctrl+C 停止\n")
        
        try:
            step = 0
            while step < 50:  # 最多50步
                step += 1
                
                # 检测周围环境
                dist_front = sensors.get_lidar_distance(0)
                dist_left = sensors.get_lidar_distance(45)
                dist_right = sensors.get_lidar_distance(-45)
                
                print(f"[步骤 {step}] 前:{dist_front:.2f}m 左:{dist_left:.2f}m 右:{dist_right:.2f}m")
                
                # 决策：前进还是转向
                if dist_front > 1.0:
                    # 前方安全，前进
                    print("  → 前进")
                    motion.set_velocity(0.3, 0.0, 0.0)
                    time.sleep(2)
                else:
                    # 前方有障碍，选择转向
                    if dist_left > dist_right:
                        print("  ↶ 左转")
                        motion.rotate_angle(45, 0.5)
                    else:
                        print("  ↷ 右转")
                        motion.rotate_angle(-45, 0.5)
                    time.sleep(1)
                
                motion.stop()
                time.sleep(0.5)
            
            print("\n探索完成！")
            
            # 保存地图
            print("\n保存地图...")
            nav.save_map("explored_map")
            
        except KeyboardInterrupt:
            print("\n\n探索被中断")
        
        # 停止
        motion.stop()
        nav.shutdown()
        sensors.stop_lidar()
        
    finally:
        robot_sys.shutdown()


def main():
    """
    主函数：提供交互式菜单选择不同的示例
    """
    examples = {
        "1": ("基本建图流程", example_mapping_basic),
        "2": ("自动建图", example_auto_mapping),
        "3": ("基本导航", example_navigation_basic),
        "4": ("带监控的导航", example_navigation_with_monitoring),
        "5": ("多点巡航", example_waypoint_navigation),
        "6": ("探索式建图", example_exploration_mapping),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 建图与导航功能示例应用")
        print("="*60)
        print("\n请选择要运行的示例：")
        print()
        
        for key, (name, _) in examples.items():
            print(f"  {key}. {name}")
        
        print("  0. 退出程序")
        print()
        
        choice = input("请输入选项 (0-6): ").strip()
        
        if choice == "0":
            print("\n再见！")
            break
        
        if choice in examples:
            name, func = examples[choice]
            try:
                func()
                input("\n按 Enter 键继续...")
            except KeyboardInterrupt:
                print("\n\n示例被中断")
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
