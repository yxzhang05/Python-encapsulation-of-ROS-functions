#!/usr/bin/env python3
# 文件名: sensors_app.py
# 功能：感知与功能示例应用
# 说明：演示如何使用 robot_lib_sensors.py 中的感知功能

from robot_lib_sensors import RobotSensors
from robot_lib_system import RobotSystem
import time
import sys

def example_lidar_basic():
    """
    示例1：基本雷达操作
    
    演示雷达的启动、数据获取和停止
    """
    print("\n" + "="*60)
    print("示例1：基本雷达操作")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 启动雷达
        print("\n1. 启动雷达（带可视化）...")
        sensors.launch_lidar(visualize=True)
        time.sleep(3)
        
        # 获取不同方向的距离
        print("\n2. 获取各方向距离...")
        angles = [0, 45, 90, -45, -90, 180]
        for angle in angles:
            dist = sensors.get_lidar_distance(angle)
            if dist > 0:
                print(f"   {angle:4d}°: {dist:5.2f}m")
            else:
                print(f"   {angle:4d}°: 无效")
        
        # 持续监控前方距离
        print("\n3. 持续监控前方距离（5秒）...")
        for i in range(5):
            dist = sensors.get_lidar_distance(0)
            print(f"   前方: {dist:.2f}m")
            time.sleep(1)
        
        # 停止雷达
        print("\n4. 停止雷达...")
        sensors.stop_lidar()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_camera_basic():
    """
    示例2：基本相机操作
    
    演示相机的启动、拍照和停止
    """
    print("\n" + "="*60)
    print("示例2：基本相机操作")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 启动相机
        print("\n1. 启动相机（带可视化）...")
        sensors.launch_camera(visualize=True)
        time.sleep(3)
        
        # 拍照
        print("\n2. 拍摄照片...")
        for i in range(3):
            filename = f"photo_{i+1}.jpg"
            print(f"   拍摄第 {i+1} 张照片...")
            sensors.get_camera_frame(filename)
            time.sleep(2)
        
        # 停止相机
        print("\n3. 停止相机...")
        sensors.stop_camera()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_visual_follow():
    """
    示例3：视觉跟随
    
    演示机器人跟随指定颜色的物体
    """
    print("\n" + "="*60)
    print("示例3：视觉跟随")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 选择跟随颜色
        colors = ['red', 'blue', 'green', 'yellow']
        print("\n可用颜色：")
        for i, color in enumerate(colors, 1):
            print(f"  {i}. {color}")
        
        choice = input("\n选择跟随颜色 (1-4): ").strip()
        
        if choice in ['1', '2', '3', '4']:
            color = colors[int(choice) - 1]
            
            print(f"\n启动视觉跟随（颜色: {color}）...")
            print("机器人将自动跟随该颜色的物体")
            print("按 Ctrl+C 停止\n")
            
            # 启动视觉跟随
            if sensors.start_visual_follow(color, control_robot=True):
                # 持续运行
                while True:
                    time.sleep(1)
        else:
            print("无效选择")
            
    except KeyboardInterrupt:
        print("\n\n停止跟随...")
        sensors.stop_application()
    
    finally:
        robot_sys.shutdown()


def example_line_tracking():
    """
    示例4：视觉巡线
    
    演示机器人沿线行驶
    """
    print("\n" + "="*60)
    print("示例4：视觉巡线")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 选择线条颜色
        colors = ['black', 'red', 'yellow', 'white']
        print("\n可用线条颜色：")
        for i, color in enumerate(colors, 1):
            print(f"  {i}. {color}")
        
        choice = input("\n选择线条颜色 (1-4): ").strip()
        
        if choice in ['1', '2', '3', '4']:
            color = colors[int(choice) - 1]
            
            print(f"\n启动视觉巡线（颜色: {color}）...")
            print("机器人将自动沿线行驶")
            print("按 Ctrl+C 停止\n")
            
            # 启动巡线
            if sensors.start_line_tracking(color, control_robot=True):
                # 持续运行
                while True:
                    time.sleep(1)
        else:
            print("无效选择")
            
    except KeyboardInterrupt:
        print("\n\n停止巡线...")
        sensors.stop_application()
    
    finally:
        robot_sys.shutdown()


def example_lidar_follow():
    """
    示例5：雷达跟随
    
    演示机器人跟随前方的人或物体
    """
    print("\n" + "="*60)
    print("示例5：雷达跟随")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 设置跟随距离
        distance = input("\n输入跟随距离（米，默认0.8）: ").strip()
        target_dist = float(distance) if distance else 0.8
        
        print(f"\n启动雷达跟随（保持距离: {target_dist}m）...")
        print("请在机器人前方移动")
        print("按 Ctrl+C 停止\n")
        
        # 启动雷达跟随
        if sensors.start_lidar_follow(target_dist=target_dist):
            # 持续运行
            while True:
                time.sleep(1)
        
    except KeyboardInterrupt:
        print("\n\n停止跟随...")
        sensors.stop_application()
    
    finally:
        robot_sys.shutdown()


def example_obstacle_detection():
    """
    示例6：障碍物检测
    
    演示使用雷达进行360度障碍物检测
    """
    print("\n" + "="*60)
    print("示例6：障碍物检测")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 启动雷达
        sensors.launch_lidar(visualize=False)
        time.sleep(3)
        
        print("\n开始360度障碍物扫描...")
        print("安全距离阈值: 0.5米\n")
        
        # 定义检测区域
        regions = {
            "正前方": 0,
            "右前方": -45,
            "右侧": -90,
            "后方": 180,
            "左侧": 90,
            "左前方": 45
        }
        
        safe_distance = 0.5  # 米
        
        # 持续监控
        try:
            while True:
                print("\n" + "-"*50)
                obstacles_detected = False
                
                for region, angle in regions.items():
                    dist = sensors.get_lidar_distance(angle)
                    
                    if dist > 0:
                        if dist < safe_distance:
                            print(f"⚠️  {region:8s} ({angle:4d}°): {dist:.2f}m - 障碍物！")
                            obstacles_detected = True
                        else:
                            print(f"✅ {region:8s} ({angle:4d}°): {dist:.2f}m - 安全")
                    else:
                        print(f"   {region:8s} ({angle:4d}°): 无数据")
                
                if obstacles_detected:
                    print("\n⚠️  检测到障碍物！")
                else:
                    print("\n✅ 周围环境安全")
                
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n\n停止检测")
        
        sensors.stop_lidar()
        
    finally:
        robot_sys.shutdown()


def example_combined_sensors():
    """
    示例7：综合传感器应用
    
    演示同时使用雷达和相机
    """
    print("\n" + "="*60)
    print("示例7：综合传感器应用")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 同时启动雷达和相机
        print("\n1. 启动雷达和相机...")
        sensors.launch_lidar(visualize=True)
        time.sleep(2)
        sensors.launch_camera(visualize=True)
        time.sleep(2)
        
        print("\n2. 综合监控（10秒）...")
        print("同时显示雷达距离和相机图像\n")
        
        for i in range(10):
            # 获取雷达数据
            dist_front = sensors.get_lidar_distance(0)
            dist_left = sensors.get_lidar_distance(90)
            dist_right = sensors.get_lidar_distance(-90)
            
            # 显示
            print(f"[{i+1:2d}s] 前:{dist_front:.2f}m | "
                  f"左:{dist_left:.2f}m | 右:{dist_right:.2f}m")
            
            # 如果前方有障碍物，拍照记录
            if dist_front > 0 and dist_front < 0.8:
                print("     ⚠️  前方障碍物，拍照记录...")
                sensors.get_camera_frame(f"obstacle_{i}.jpg")
            
            time.sleep(1)
        
        print("\n3. 关闭传感器...")
        sensors.stop_camera()
        sensors.stop_lidar()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_visual_follow_no_control():
    """
    示例8：视觉检测（不控制机器人）
    
    演示只检测目标但不控制机器人运动
    """
    print("\n" + "="*60)
    print("示例8：视觉检测（不控制机器人）")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        sensors = RobotSensors()
        
        # 定义回调函数
        def on_target_detected(info):
            if info['detected']:
                print(f"检测到目标: 中心({info['center_x']}, {info['center_y']}), "
                      f"面积:{info['area']}, 距离:{info['distance']:.2f}m")
            else:
                print("未检测到目标")
        
        print("\n启动视觉检测（颜色: red）...")
        print("只检测不控制机器人")
        print("按 Ctrl+C 停止\n")
        
        # 启动视觉检测但不控制
        if sensors.start_visual_follow('red', control_robot=False, callback=on_target_detected):
            while True:
                time.sleep(1)
        
    except KeyboardInterrupt:
        print("\n\n停止检测...")
        sensors.stop_application()
    
    finally:
        robot_sys.shutdown()


def main():
    """
    主函数：提供交互式菜单选择不同的示例
    """
    examples = {
        "1": ("基本雷达操作", example_lidar_basic),
        "2": ("基本相机操作", example_camera_basic),
        "3": ("视觉跟随", example_visual_follow),
        "4": ("视觉巡线", example_line_tracking),
        "5": ("雷达跟随", example_lidar_follow),
        "6": ("障碍物检测", example_obstacle_detection),
        "7": ("综合传感器应用", example_combined_sensors),
        "8": ("视觉检测（不控制）", example_visual_follow_no_control),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 感知与功能示例应用")
        print("="*60)
        print("\n请选择要运行的示例：")
        print()
        
        for key, (name, _) in examples.items():
            print(f"  {key}. {name}")
        
        print("  0. 退出程序")
        print()
        
        choice = input("请输入选项 (0-8): ").strip()
        
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
