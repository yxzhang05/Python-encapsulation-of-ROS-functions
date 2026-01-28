#!/usr/bin/env python3
"""
建图示例程序
演示SLAM建图功能，支持自动建图和手动建图
"""

from robot_lib import Robot
import time


def manual_mapping_demo(bot):
    """
    手动建图示例：使用键盘控制建图
    """
    print("\n" + "="*50)
    print("手动建图模式")
    print("="*50)
    
    # 1. 启动建图
    print("\n1. 启动gmapping建图算法")
    success = bot.start_mapping("gmapping")
    if not success:
        return False
    
    print("\n建图已启动！")
    print("接下来将启动键盘控制，请使用键盘移动机器人完成建图")
    print("建议路线：尽可能覆盖整个区域，确保回环")
    time.sleep(2)
    
    # 2. 使用键盘控制移动建图
    print("\n2. 启动键盘控制")
    print("使用WASD或方向键控制机器人移动")
    print("按Ctrl+C结束建图")
    
    try:
        bot.start_keyboard_control()
    except KeyboardInterrupt:
        print("\n键盘控制结束")
    
    # 3. 保存地图
    print("\n3. 保存地图")
    map_name = f"manual_map_{int(time.time())}"
    success = bot.save_map(map_name)
    if success:
        print(f"地图保存成功: {map_name}")
    
    return True


def auto_mapping_demo(bot):
    """
    自动建图示例：使用程序控制自动建图
    这展示了如何在建图的同时使用其他功能
    """
    print("\n" + "="*50)
    print("自动建图模式")
    print("="*50)
    
    # 1. 启动雷达
    print("\n1. 启动雷达")
    bot.launch_lidar()
    time.sleep(2)
    
    # 2. 启动建图（非阻塞）
    print("\n2. 启动cartographer建图算法")
    success = bot.start_mapping("cartographer")
    if not success:
        bot.stop_lidar()
        return False
    
    print("\n建图已启动，开始自动探索...")
    
    # 3. 自动探索策略：简单的正方形路径
    print("\n3. 执行自动探索路径")
    
    try:
        # 正方形路径，每边2米
        side_length = 2.0
        speed = 0.2
        
        for i in range(4):
            print(f"\n第{i+1}边: 前进{side_length}米")
            
            # 前进过程中监控雷达
            start_time = time.time()
            while (time.time() - start_time) < (side_length / speed):
                # 检查前方障碍
                distance = bot.get_lidar_distance(0)
                if distance > 0 and distance < 0.3:
                    print("检测到障碍物，提前停止")
                    bot.emergency_stop()
                    break
                
                bot.set_velocity(speed, 0.0, 0.0)
                time.sleep(0.1)
            
            bot.emergency_stop()
            time.sleep(1)
            
            print(f"左转90度")
            bot.rotate_angle(90, 0.5)
            time.sleep(1)
        
        # 额外的探索：螺旋形扩展
        print("\n执行螺旋形探索")
        for r in range(1, 4):
            radius = 0.5 * r
            print(f"弧线运动，半径={radius}m")
            bot.move_distance_with_radius(2.0, 0.2, radius=radius)
            time.sleep(1)
        
        print("\n自动探索完成！")
        
    except KeyboardInterrupt:
        print("\n用户中断自动建图")
        bot.emergency_stop()
    
    # 4. 保存地图
    print("\n4. 保存地图")
    map_name = f"auto_map_{int(time.time())}"
    success = bot.save_map(map_name)
    if success:
        print(f"地图保存成功: {map_name}")
    
    # 5. 清理
    bot.stop_lidar()
    
    return True


def smart_mapping_demo(bot):
    """
    智能建图示例：使用雷达数据智能选择路径
    """
    print("\n" + "="*50)
    print("智能建图模式")
    print("="*50)
    
    # 1. 启动雷达和建图
    print("\n1. 启动系统")
    bot.launch_lidar()
    time.sleep(2)
    
    success = bot.start_mapping("gmapping")
    if not success:
        bot.stop_lidar()
        return False
    
    print("\n2. 开始智能探索")
    print("策略：沿墙前进，遇到开放空间则探索")
    
    try:
        explored_time = 0
        max_time = 60  # 最多探索60秒
        
        while explored_time < max_time:
            # 获取周围距离信息
            dist_front = bot.get_lidar_distance(0)
            dist_left = bot.get_lidar_distance(90)
            dist_right = bot.get_lidar_distance(-90)
            
            print(f"\n距离 - 前:{dist_front:.2f}m 左:{dist_left:.2f}m 右:{dist_right:.2f}m")
            
            # 决策逻辑
            if dist_front > 1.0:
                # 前方开阔，前进
                print("前方开阔，前进")
                bot.set_velocity(0.2, 0.0, 0.0)
                time.sleep(2)
                
            elif dist_left > dist_right and dist_left > 0.5:
                # 左侧空间更大，左转
                print("左转探索")
                bot.rotate_angle(45, 0.5)
                time.sleep(1)
                
            elif dist_right > 0.5:
                # 右侧有空间，右转
                print("右转探索")
                bot.rotate_angle(-45, 0.5)
                time.sleep(1)
                
            else:
                # 周围都是障碍，后退并旋转
                print("周围受限，后退并旋转")
                bot.move_distance(-0.3, 0.2)
                time.sleep(1)
                bot.rotate_angle(90, 0.5)
                time.sleep(1)
            
            explored_time += 3
        
        bot.emergency_stop()
        print("\n智能探索完成！")
        
    except KeyboardInterrupt:
        print("\n用户中断智能建图")
        bot.emergency_stop()
    
    # 3. 保存地图
    print("\n3. 保存地图")
    map_name = f"smart_map_{int(time.time())}"
    success = bot.save_map(map_name)
    if success:
        print(f"地图保存成功: {map_name}")
    
    # 4. 清理
    bot.stop_lidar()
    
    return True


def main():
    # 创建机器人对象
    bot = Robot()

    try:
        # 初始化机器人
        print("="*50)
        print("SLAM建图示例程序")
        print("="*50)
        
        success = bot.initialize("diff")  # 差速车型
        
        if not success:
            print("初始化失败，程序退出")
            return

        # 检查电池
        voltage = bot.get_battery_voltage()
        print(f"\n当前电池电压: {voltage:.2f} V")
        if voltage < 10.5:
            print("【警告】电量过低，建议充电后再建图")
            return

        # 选择建图模式
        print("\n请选择建图模式:")
        print("1. 手动建图 (键盘控制)")
        print("2. 自动建图 (预设路径)")
        print("3. 智能建图 (基于雷达的智能探索)")
        
        # 这里简化为自动运行三种模式的演示
        # 实际使用时可以通过input()让用户选择
        
        print("\n=== 演示模式1: 手动建图 ===")
        print("(实际使用时会启动键盘控制)")
        print("跳过演示...\n")
        # manual_mapping_demo(bot)  # 需要用户交互，演示时跳过
        
        print("\n=== 演示模式2: 自动建图 ===")
        auto_mapping_demo(bot)
        
        print("\n=== 演示模式3: 智能建图 ===")
        # smart_mapping_demo(bot)  # 可选运行
        
        print("\n" + "="*50)
        print("所有建图演示完成！")
        print("="*50)

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        # 安全关闭
        bot.shutdown()


if __name__ == "__main__":
    main()
