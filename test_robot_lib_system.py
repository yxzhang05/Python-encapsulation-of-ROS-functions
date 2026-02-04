#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试文件：test_robot_lib_system.py
测试内容：系统管理功能
适用车型：麦轮车（mec）
"""

from robot_lib_system import Robot
import time

def print_menu():
    """打印测试菜单"""
    print("\n" + "="*50)
    print("系统管理功能测试菜单")
    print("="*50)
    print("1. 测试初始化和电池电压")
    print("2. 测试软件版本查询")
    print("3. 测试急停功能")
    print("4. 运行所有测试")
    print("0. 退出")
    print("="*50)

def test_initialize_and_voltage(robot):
    """测试1：初始化和电池电压"""
    print("\n【测试1】初始化和电池电压测试")
    print("-" * 40)
    
    # 初始化机器人
    print("正在初始化麦轮车...")
    success = robot.initialize("mec")
    
    if not success:
        print("❌ 初始化失败！")
        return False
    
    print("✅ 初始化成功")
    time.sleep(1)
    
    # 获取电池电压
    print("\n正在读取电池电压...")
    voltage = robot.get_battery_voltage()
    print(f"📊 当前电池电压: {voltage:.2f} V")
    
    if voltage < 10.5:
        print("⚠️  警告：电量过低，请充电！")
    elif voltage < 11.0:
        print("⚠️  提示：电量较低，建议充电")
    else:
        print("✅ 电量充足")
    
    return True

def test_software_version(robot):
    """测试2：软件版本查询"""
    print("\n【测试2】软件版本查询测试")
    print("-" * 40)
    
    print("正在查询软件版本...")
    version = robot.get_software_version()
    print(f"📦 软件版本信息: {version}")
    
    return True

def test_emergency_stop(robot):
    """测试3：急停功能"""
    print("\n【测试3】急停功能测试")
    print("-" * 40)
    
    print("⚠️  即将测试急停功能...")
    print("提示：急停会立即停止所有运动")
    
    input("按回车键继续...")
    
    print("执行急停...")
    robot.emergency_stop()
    print("✅ 急停命令已发送")
    print("机器人应该已经停止所有运动")
    
    return True

def run_all_tests(robot):
    """运行所有测试"""
    print("\n" + "="*50)
    print("开始运行所有测试")
    print("="*50)
    
    tests = [
        ("初始化和电池电压", test_initialize_and_voltage),
        ("软件版本查询", test_software_version),
        ("急停功能", test_emergency_stop)
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func(robot)
            results.append((name, result))
            time.sleep(1)
        except Exception as e:
            print(f"❌ 测试失败: {e}")
            results.append((name, False))
    
    # 打印测试总结
    print("\n" + "="*50)
    print("测试总结")
    print("="*50)
    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{name}: {status}")
    print("="*50)

def main():
    """主函数"""
    print("\n" + "="*50)
    print("系统管理功能测试程序")
    print("车型: 麦轮车 (mec)")
    print("="*50)
    
    robot = Robot()
    
    try:
        while True:
            print_menu()
            choice = input("\n请选择测试项 (0-4): ").strip()
            
            if choice == '0':
                print("\n退出测试程序")
                break
            elif choice == '1':
                test_initialize_and_voltage(robot)
            elif choice == '2':
                test_software_version(robot)
            elif choice == '3':
                test_emergency_stop(robot)
            elif choice == '4':
                run_all_tests(robot)
            else:
                print("❌ 无效选择，请重试")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
    
    finally:
        print("\n正在安全关闭...")
        robot.shutdown()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()
