#!/usr/bin/env python3
"""
传感器与感知应用示例程序
演示雷达、相机、视觉跟随、巡线等功能
"""

from robot_lib import Robot
import time


def main():
    # 创建机器人对象
    bot = Robot()

    try:
        # 1. 初始化机器人
        print("="*50)
        print("传感器与感知应用示例程序")
        print("="*50)
        
        success = bot.initialize("diff")  # 差速车型
        
        if not success:
            print("初始化失败，程序退出")
            return

        # ========== 雷达功能测试 ==========
        print("\n" + "="*50)
        print("雷达功能测试")
        print("="*50)
        
        print("\n1. 启动雷达")
        bot.launch_lidar()
        time.sleep(3)

        print("\n2. 获取雷达距离信息")
        # 获取正前方(0度)的距离
        distance_front = bot.get_lidar_distance(0)
        print(f"正前方距离: {distance_front:.2f}m")
        
        # 获取左侧(90度)的距离
        distance_left = bot.get_lidar_distance(90)
        print(f"左侧距离: {distance_left:.2f}m")
        
        # 获取右侧(-90度)的距离
        distance_right = bot.get_lidar_distance(-90)
        print(f"右侧距离: {distance_right:.2f}m")

        print("\n3. 关闭雷达")
        bot.stop_lidar()
        time.sleep(1)

        # ========== 相机功能测试 ==========
        print("\n" + "="*50)
        print("相机功能测试")
        print("="*50)
        
        print("\n1. 启动相机")
        bot.launch_camera()
        time.sleep(3)

        print("\n2. 捕获图像")
        image = bot.capture_image()
        if image is not None:
            print("图像捕获成功")
        else:
            print("图像捕获失败或功能未实现")

        print("\n3. 关闭相机")
        bot.stop_camera()
        time.sleep(1)

        # ========== 视觉跟随测试 ==========
        print("\n" + "="*50)
        print("视觉跟随测试")
        print("="*50)
        
        # 重新启动相机用于视觉跟随
        print("\n1. 启动相机")
        bot.launch_camera()
        time.sleep(2)

        print("\n2. 启动红色物体跟随 (自动控制)")
        bot.start_visual_follow("red", control_enabled=True)
        
        print("跟随运行中...持续10秒")
        for i in range(10):
            # 获取目标信息
            target_info = bot.get_visual_target_info()
            if target_info['detected']:
                print(f"目标检测到: 位置=({target_info['x']}, {target_info['y']}), 面积={target_info['area']}")
            else:
                print("未检测到目标")
            time.sleep(1)
        
        print("\n3. 停止视觉跟随")
        bot.stop_application()
        time.sleep(1)

        # 使用自定义颜色的视觉跟随
        print("\n4. 自定义HSV颜色跟随示例")
        print("使用自定义橙色 HSV范围")
        hsv_lower = (10, 100, 100)  # 橙色下界
        hsv_upper = (20, 255, 255)  # 橙色上界
        bot.start_visual_follow_custom(hsv_lower, hsv_upper, control_enabled=True)
        time.sleep(5)
        bot.stop_application()
        time.sleep(1)

        # 仅检测不控制的模式
        print("\n5. 视觉检测模式 (不控制底盘)")
        bot.start_visual_follow("blue", control_enabled=False)
        
        print("检测运行中...持续5秒")
        for i in range(5):
            target_info = bot.get_visual_target_info()
            if target_info['detected']:
                print(f"检测到蓝色目标")
            time.sleep(1)
        
        bot.stop_application()

        # ========== 视觉巡线测试 ==========
        print("\n" + "="*50)
        print("视觉巡线测试")
        print("="*50)
        
        print("\n1. 启动黑线巡线 (自动控制)")
        bot.start_line_tracking("black", control_enabled=True)
        
        print("巡线运行中...持续10秒")
        for i in range(10):
            # 获取线条信息
            line_info = bot.get_line_info()
            if line_info['detected']:
                print(f"检测到线条: 偏移={line_info['offset']:.2f}, 角度={line_info['angle']:.2f}°")
            else:
                print("未检测到线条")
            time.sleep(1)
        
        print("\n2. 停止巡线")
        bot.stop_application()
        time.sleep(1)

        # 使用自定义颜色的巡线
        print("\n3. 自定义颜色巡线示例")
        print("使用自定义黄色线条")
        hsv_lower = (20, 100, 100)  # 黄色下界
        hsv_upper = (30, 255, 255)  # 黄色上界
        bot.start_line_tracking_custom(hsv_lower, hsv_upper, control_enabled=True)
        time.sleep(5)
        bot.stop_application()
        time.sleep(1)

        # ========== 雷达跟随测试 ==========
        print("\n" + "="*50)
        print("雷达跟随测试")
        print("="*50)
        
        print("\n1. 启动雷达")
        bot.launch_lidar()
        time.sleep(2)

        print("\n2. 启动雷达跟随 (保持1米距离)")
        bot.start_lidar_follow(1.0)
        
        print("跟随运行中...持续10秒")
        time.sleep(10)
        
        print("\n3. 停止雷达跟随")
        bot.stop_application()
        
        print("\n4. 关闭雷达")
        bot.stop_lidar()

        # ========== 综合应用示例 ==========
        print("\n" + "="*50)
        print("综合应用示例：避障前进")
        print("="*50)
        
        print("\n启动雷达")
        bot.launch_lidar()
        time.sleep(2)

        print("\n开始避障前进...")
        print("前进过程中检测障碍物，距离<0.5m时停止")
        
        for i in range(20):
            # 检测前方距离
            distance = bot.get_lidar_distance(0)
            
            if distance > 0 and distance < 0.5:
                print(f"检测到障碍物！距离: {distance:.2f}m")
                bot.emergency_stop()
                print("停止运动")
                break
            else:
                print(f"前方安全，距离: {distance:.2f}m，继续前进")
                bot.set_velocity(0.1, 0.0, 0.0)
            
            time.sleep(0.5)
        
        bot.emergency_stop()
        bot.stop_lidar()

        print("\n所有测试完成！")

    except KeyboardInterrupt:
        print("\n用户中断")
        bot.stop_application()
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        # 安全关闭
        bot.shutdown()


if __name__ == "__main__":
    main()
