#!/usr/bin/env python3
"""
导航示例程序
演示使用已有地图进行自主导航
"""

from robot_lib import Robot
import time


def simple_navigation_demo(bot, map_name):
    """
    简单导航示例：导航到单个目标点
    """
    print("\n" + "="*50)
    print("简单导航示例")
    print("="*50)
    
    # 1. 启动导航系统
    print(f"\n1. 加载地图并启动导航: {map_name}")
    success = bot.start_navigation(map_name)
    if not success:
        return False
    
    print("导航系统已启动，等待初始化...")
    time.sleep(5)
    
    # 2. 设置目标点
    print("\n2. 发送导航目标")
    target_x = 2.0
    target_y = 1.5
    target_theta = 90  # 朝向北方
    
    print(f"目标: x={target_x}m, y={target_y}m, theta={target_theta}°")
    bot.move_to_goal(target_x, target_y, target_theta)
    
    # 3. 监控导航状态
    print("\n3. 监控导航进度")
    timeout = 60  # 60秒超时
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        status = bot.get_navigation_status()
        pose = bot.get_pose()
        
        if pose:
            print(f"当前位置: x={pose['x']:.2f}m, y={pose['y']:.2f}m")
        
        print(f"导航状态: {status}")
        
        if status == "reached":
            print("\n目标到达！")
            break
        elif status == "failed":
            print("\n导航失败！")
            break
        
        time.sleep(2)
    
    if (time.time() - start_time) >= timeout:
        print("\n导航超时！")
        bot.cancel_navigation()
    
    return True


def multi_point_navigation_demo(bot, map_name):
    """
    多点导航示例：按顺序导航到多个目标点
    """
    print("\n" + "="*50)
    print("多点导航示例")
    print("="*50)
    
    # 1. 启动导航系统
    print(f"\n1. 加载地图并启动导航: {map_name}")
    success = bot.start_navigation(map_name)
    if not success:
        return False
    
    time.sleep(5)
    
    # 2. 定义巡航路径
    waypoints = [
        (1.0, 1.0, 0),      # 点1
        (2.0, 1.0, 90),     # 点2
        (2.0, 2.0, 180),    # 点3
        (1.0, 2.0, 270),    # 点4
        (1.0, 1.0, 0)       # 回到起点
    ]
    
    print(f"\n2. 开始巡航，共{len(waypoints)}个点")
    
    for i, (x, y, theta) in enumerate(waypoints):
        print(f"\n--- 导航到点{i+1}/{len(waypoints)} ---")
        print(f"目标: x={x}m, y={y}m, theta={theta}°")
        
        bot.move_to_goal(x, y, theta)
        
        # 等待到达
        timeout = 30
        start_time = time.time()
        reached = False
        
        while (time.time() - start_time) < timeout:
            status = bot.get_navigation_status()
            
            if status == "reached":
                print(f"点{i+1}到达！")
                reached = True
                break
            elif status == "failed":
                print(f"点{i+1}导航失败，跳过")
                break
            
            time.sleep(1)
        
        if not reached:
            print(f"点{i+1}超时，继续下一个点")
        
        # 在每个点停留2秒
        time.sleep(2)
    
    print("\n巡航完成！")
    return True


def interactive_navigation_demo(bot, map_name):
    """
    交互式导航示例：用户输入目标点进行导航
    """
    print("\n" + "="*50)
    print("交互式导航示例")
    print("="*50)
    
    # 1. 启动导航系统
    print(f"\n1. 加载地图并启动导航: {map_name}")
    success = bot.start_navigation(map_name)
    if not success:
        return False
    
    time.sleep(5)
    
    print("\n2. 交互式导航")
    print("可以输入目标点坐标，机器人将自动导航到该点")
    print("输入 'quit' 退出")
    
    # 预定义一些常用位置
    named_locations = {
        "home": (0.0, 0.0, 0),
        "kitchen": (3.0, 2.0, 90),
        "bedroom": (5.0, 4.0, 180),
        "door": (2.0, 0.0, 0)
    }
    
    print("\n预定义位置:")
    for name, (x, y, theta) in named_locations.items():
        print(f"  {name}: ({x}, {y}, {theta}°)")
    
    # 模拟用户输入（实际使用时可用input()）
    # 这里演示导航到预定义位置
    demo_commands = ["kitchen", "bedroom", "home"]
    
    for cmd in demo_commands:
        print(f"\n>>> 导航到: {cmd}")
        
        if cmd in named_locations:
            x, y, theta = named_locations[cmd]
            print(f"目标: {cmd} ({x}, {y}, {theta}°)")
            
            bot.move_to_goal(x, y, theta)
            
            # 监控导航
            timeout = 30
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                status = bot.get_navigation_status()
                
                if status == "reached":
                    print(f"已到达 {cmd}")
                    break
                elif status == "failed":
                    print(f"导航到 {cmd} 失败")
                    break
                
                time.sleep(1)
            
            time.sleep(2)
    
    return True


def navigation_with_obstacle_avoidance_demo(bot, map_name):
    """
    带动态避障的导航示例
    """
    print("\n" + "="*50)
    print("动态避障导航示例")
    print("="*50)
    
    # 1. 启动雷达
    print("\n1. 启动雷达用于动态避障")
    bot.launch_lidar()
    time.sleep(2)
    
    # 2. 启动导航系统
    print(f"\n2. 加载地图并启动导航: {map_name}")
    success = bot.start_navigation(map_name)
    if not success:
        bot.stop_lidar()
        return False
    
    time.sleep(5)
    
    # 3. 导航到目标，同时监控动态障碍
    print("\n3. 导航到目标点")
    target_x = 3.0
    target_y = 2.0
    target_theta = 0
    
    print(f"目标: x={target_x}m, y={target_y}m, theta={target_theta}°")
    bot.move_to_goal(target_x, target_y, target_theta)
    
    # 4. 监控导航过程
    print("\n4. 监控导航过程和障碍物")
    timeout = 60
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        status = bot.get_navigation_status()
        
        # 检查前方障碍
        dist_front = bot.get_lidar_distance(0)
        if dist_front > 0 and dist_front < 0.3:
            print(f"警告！检测到近距离障碍物: {dist_front:.2f}m")
            # 导航系统会自动避障，这里只是提醒
        
        pose = bot.get_pose()
        if pose:
            print(f"位置: ({pose['x']:.2f}, {pose['y']:.2f}), 状态: {status}")
        
        if status == "reached":
            print("\n目标到达！")
            break
        elif status == "failed":
            print("\n导航失败！可能遇到无法避开的障碍")
            break
        
        time.sleep(2)
    
    if (time.time() - start_time) >= timeout:
        print("\n导航超时！")
        bot.cancel_navigation()
    
    # 5. 清理
    bot.stop_lidar()
    
    return True


def main():
    # 创建机器人对象
    bot = Robot()

    try:
        # 初始化机器人
        print("="*50)
        print("自主导航示例程序")
        print("="*50)
        
        success = bot.initialize("diff")  # 差速车型
        
        if not success:
            print("初始化失败，程序退出")
            return

        # 检查电池
        voltage = bot.get_battery_voltage()
        print(f"\n当前电池电压: {voltage:.2f} V")

        # 使用之前保存的地图
        # 实际使用时，用户需要提供真实的地图名称
        map_name = "my_map"  # 替换为实际的地图名称
        
        print(f"\n将使用地图: {map_name}")
        print("请确保该地图文件存在于 ~/maps/ 目录下")
        
        # 演示不同的导航模式
        print("\n" + "="*50)
        print("导航模式选择")
        print("="*50)
        print("1. 简单导航 (单点)")
        print("2. 多点巡航")
        print("3. 交互式导航")
        print("4. 动态避障导航")
        
        # 这里演示简单导航和多点巡航
        print("\n=== 演示: 简单导航 ===")
        simple_navigation_demo(bot, map_name)
        time.sleep(2)
        
        print("\n=== 演示: 多点巡航 ===")
        multi_point_navigation_demo(bot, map_name)
        time.sleep(2)
        
        print("\n=== 演示: 交互式导航 ===")
        interactive_navigation_demo(bot, map_name)
        time.sleep(2)
        
        print("\n=== 演示: 动态避障导航 ===")
        navigation_with_obstacle_avoidance_demo(bot, map_name)
        
        print("\n" + "="*50)
        print("所有导航演示完成！")
        print("="*50)

    except KeyboardInterrupt:
        print("\n用户中断")
        bot.cancel_navigation()
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        # 安全关闭
        bot.shutdown()


if __name__ == "__main__":
    main()
