#!/usr/bin/env python3
# 文件名: system_app.py
# 功能：系统管理功能示例应用
# 说明：演示如何使用 robot_lib_system.py 中的系统管理函数

from robot_lib_system import RobotSystem
import time
import sys

def example_basic_initialization():
    """
    示例1：基本的系统初始化和关闭
    
    演示如何正确初始化和关闭机器人系统
    """
    print("\n" + "="*60)
    print("示例1：基本的系统初始化和关闭")
    print("="*60)
    
    # 创建机器人系统对象
    robot = RobotSystem()
    
    try:
        # 初始化机器人（麦轮车型）
        print("\n1. 初始化机器人系统...")
        if not robot.initialize("mec"):
            print("❌ 初始化失败")
            return
        
        print("✅ 初始化成功！")
        
        # 等待一段时间，让用户观察系统状态
        print("\n2. 系统运行中，等待 5 秒...")
        time.sleep(5)
        
    finally:
        # 安全关闭系统
        print("\n3. 关闭系统...")
        robot.shutdown()
        print("✅ 系统已安全关闭")


def example_battery_monitoring():
    """
    示例2：电池电压监控
    
    演示如何持续监控电池电压并实现低电量报警
    """
    print("\n" + "="*60)
    print("示例2：电池电压监控")
    print("="*60)
    
    robot = RobotSystem()
    
    try:
        # 初始化系统
        if not robot.initialize("mec"):
            return
        
        print("\n开始监控电池电压（按 Ctrl+C 停止）...\n")
        
        # 定义电压阈值
        LOW_BATTERY_WARNING = 11.0   # 低电量警告阈值（伏特）
        LOW_BATTERY_CRITICAL = 10.5  # 低电量严重阈值（伏特）
        
        # 持续监控
        while True:
            # 获取电压
            voltage = robot.get_battery_voltage()
            
            # 判断电量状态并显示
            if voltage <= 0:
                status = "❌ 无数据"
            elif voltage < LOW_BATTERY_CRITICAL:
                status = "🔴 严重低电量！请立即充电！"
            elif voltage < LOW_BATTERY_WARNING:
                status = "🟡 低电量警告"
            else:
                status = "🟢 电量正常"
            
            # 打印状态
            print(f"电池电压: {voltage:5.2f}V  |  状态: {status}")
            
            # 如果电量严重不足，执行急停并退出
            if voltage > 0 and voltage < LOW_BATTERY_CRITICAL:
                print("\n⚠️  电量过低，执行急停保护...")
                robot.emergency_stop()
                time.sleep(2)
                break
            
            # 每隔 2 秒检查一次
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n\n用户停止监控")
    
    finally:
        robot.shutdown()


def example_emergency_stop():
    """
    示例3：急停功能演示
    
    演示如何使用急停功能立即停止机器人
    """
    print("\n" + "="*60)
    print("示例3：急停功能演示")
    print("="*60)
    
    robot = RobotSystem()
    
    try:
        # 初始化系统
        if not robot.initialize("mec"):
            return
        
        print("\n模拟机器人运行场景...")
        print("（实际应用中，这里可能有运动控制代码）")
        time.sleep(2)
        
        # 模拟检测到危险情况
        print("\n⚠️  检测到危险情况！")
        print("执行急停...")
        robot.emergency_stop()
        print("✅ 机器人已停止")
        
        time.sleep(2)
        
    finally:
        robot.shutdown()


def example_version_check():
    """
    示例4：软件版本查询
    
    演示如何查询底盘固件版本
    """
    print("\n" + "="*60)
    print("示例4：软件版本查询")
    print("="*60)
    
    robot = RobotSystem()
    
    try:
        # 初始化系统
        if not robot.initialize("diff"):  # 使用差速车型
            return
        
        # 获取版本信息
        print("\n查询软件版本...")
        version = robot.get_software_version()
        
        print(f"\n软件版本信息:")
        print(f"  {version}")
        
        time.sleep(2)
        
    finally:
        robot.shutdown()


def example_different_robot_types():
    """
    示例5：不同车型的初始化
    
    演示如何初始化不同类型的机器人
    """
    print("\n" + "="*60)
    print("示例5：不同车型的初始化")
    print("="*60)
    
    robot_types = {
        "mec": "麦克纳姆轮",
        "diff": "差速轮",
        "akm": "阿克曼转向"
    }
    
    for robot_type, description in robot_types.items():
        print(f"\n{'='*40}")
        print(f"测试车型: {description} ({robot_type})")
        print('='*40)
        
        robot = RobotSystem()
        
        try:
            # 初始化指定车型
            if robot.initialize(robot_type):
                print(f"✅ {description} 初始化成功")
                
                # 获取电压
                voltage = robot.get_battery_voltage()
                print(f"   电池电压: {voltage:.2f}V")
                
                time.sleep(2)
            else:
                print(f"❌ {description} 初始化失败")
        
        finally:
            robot.shutdown()
        
        # 等待一下再测试下一个车型
        time.sleep(1)


def example_error_handling():
    """
    示例6：错误处理演示
    
    演示如何正确处理各种错误情况
    """
    print("\n" + "="*60)
    print("示例6：错误处理演示")
    print("="*60)
    
    robot = RobotSystem()
    
    # 测试1：未初始化就使用功能
    print("\n[测试1] 未初始化就获取电压...")
    voltage = robot.get_battery_voltage()
    print(f"结果: {voltage}V (应该显示警告)")
    
    # 测试2：使用无效的车型
    print("\n[测试2] 使用无效的车型...")
    result = robot.initialize("invalid_type")
    print(f"结果: {result} (应该返回 False)")
    
    # 测试3：正常初始化并使用
    print("\n[测试3] 正常初始化...")
    try:
        if robot.initialize("mec"):
            print("✅ 初始化成功")
            
            # 获取电压
            voltage = robot.get_battery_voltage()
            print(f"电压: {voltage:.2f}V")
            
    finally:
        robot.shutdown()


def main():
    """
    主函数：提供交互式菜单选择不同的示例
    """
    examples = {
        "1": ("基本的系统初始化和关闭", example_basic_initialization),
        "2": ("电池电压监控", example_battery_monitoring),
        "3": ("急停功能演示", example_emergency_stop),
        "4": ("软件版本查询", example_version_check),
        "5": ("不同车型的初始化", example_different_robot_types),
        "6": ("错误处理演示", example_error_handling),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 系统管理功能示例应用")
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
