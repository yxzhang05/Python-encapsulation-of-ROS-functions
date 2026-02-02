#!/usr/bin/env python3
# 文件名: arm_app.py
# 功能：机械臂控制功能示例应用
# 说明：演示如何使用 robot_lib_arm.py 中的机械臂控制函数

from robot_lib_arm import RobotArm
from robot_lib_system import RobotSystem
import time
import math
import sys

def example_basic_arm_control():
    """
    示例1：基本机械臂控制
    
    演示机械臂的基本操作：复位、关节控制、夹爪控制
    """
    print("\n" + "="*60)
    print("示例1：基本机械臂控制")
    print("="*60)
    
    # 初始化系统
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        print("系统初始化失败")
        return
    
    try:
        arm = RobotArm()
        
        # 1. 复位
        print("\n1. 机械臂复位...")
        arm.arm_home()
        time.sleep(2)
        
        # 2. 移动到不同位置
        print("\n2. 移动大臂到 45°...")
        arm.set_joint_angles(45, 0)
        time.sleep(2)
        
        print("\n3. 移动小臂到 -30°...")
        arm.set_joint_angles(45, -30)
        time.sleep(2)
        
        # 3. 控制夹爪
        print("\n4. 测试夹爪：打开-闭合-打开...")
        arm.set_gripper(10)  # 打开
        time.sleep(1)
        arm.set_gripper(0)   # 闭合
        time.sleep(1)
        arm.set_gripper(10)  # 打开
        time.sleep(1)
        
        # 4. 回到初始位置
        print("\n5. 回到初始位置...")
        arm.arm_home()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_cartesian_control():
    """
    示例2：笛卡尔空间控制
    
    演示使用XY坐标控制机械臂末端位置
    """
    print("\n" + "="*60)
    print("示例2：笛卡尔空间控制")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        # 复位
        arm.arm_home()
        time.sleep(2)
        
        # 定义一系列目标点
        targets = [
            (300, 0),     # 正前方
            (250, 100),   # 右前方
            (200, 150),   # 右侧
            (250, -100),  # 左前方
            (200, -150),  # 左侧
        ]
        
        print(f"\n机械臂将移动到 {len(targets)} 个目标点...")
        
        for i, (x, y) in enumerate(targets, 1):
            print(f"\n{i}. 移动到 ({x}mm, {y}mm)...")
            
            if arm.set_arm_position(x, y):
                # 获取当前末端位置
                actual_x, actual_y = arm.get_arm_pose_xy()
                print(f"   实际位置: ({actual_x:.1f}mm, {actual_y:.1f}mm)")
                time.sleep(2)
            else:
                print(f"   目标点不可达！")
        
        # 回到初始位置
        print("\n回到初始位置...")
        arm.arm_home()
        
        print("\n✅ 测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_pick_and_place():
    """
    示例3：抓取和放置
    
    演示一个完整的抓取-移动-放置流程
    """
    print("\n" + "="*60)
    print("示例3：抓取和放置")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        # 定义抓取位置和放置位置
        pick_pos = (250, 50)    # 抓取位置 (mm)
        place_pos = (250, -50)  # 放置位置 (mm)
        safe_height = (200, 0)  # 安全高度（避免碰撞）
        
        print("\n开始抓取和放置流程...")
        
        # 1. 移动到安全高度
        print("\n1. 移动到安全高度...")
        arm.set_arm_position(safe_height[0], safe_height[1])
        arm.set_gripper(10)  # 确保夹爪打开
        time.sleep(2)
        
        # 2. 移动到抓取位置上方
        print("\n2. 移动到抓取位置上方...")
        arm.set_arm_position(pick_pos[0], pick_pos[1] + 50)
        time.sleep(2)
        
        # 3. 下降到抓取位置
        print("\n3. 下降到抓取位置...")
        arm.set_arm_position(pick_pos[0], pick_pos[1])
        time.sleep(2)
        
        # 4. 闭合夹爪（抓取）
        print("\n4. 闭合夹爪（抓取）...")
        arm.set_gripper(0)
        time.sleep(2)
        
        # 5. 提升物体
        print("\n5. 提升物体...")
        arm.set_arm_position(pick_pos[0], pick_pos[1] + 50)
        time.sleep(2)
        
        # 6. 移动到安全高度
        print("\n6. 移动到安全高度...")
        arm.set_arm_position(safe_height[0], safe_height[1])
        time.sleep(2)
        
        # 7. 移动到放置位置上方
        print("\n7. 移动到放置位置上方...")
        arm.set_arm_position(place_pos[0], place_pos[1] + 50)
        time.sleep(2)
        
        # 8. 下降到放置位置
        print("\n8. 下降到放置位置...")
        arm.set_arm_position(place_pos[0], place_pos[1])
        time.sleep(2)
        
        # 9. 打开夹爪（放置）
        print("\n9. 打开夹爪（放置）...")
        arm.set_gripper(10)
        time.sleep(2)
        
        # 10. 提升并回到初始位置
        print("\n10. 回到初始位置...")
        arm.set_arm_position(place_pos[0], place_pos[1] + 50)
        time.sleep(1)
        arm.arm_home()
        
        print("\n✅ 抓取和放置完成！")
        
    finally:
        robot_sys.shutdown()


def example_draw_circle():
    """
    示例4：画圆形轨迹
    
    演示机械臂末端沿圆形轨迹运动
    """
    print("\n" + "="*60)
    print("示例4：画圆形轨迹")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        # 圆心和半径
        center_x = 250  # mm
        center_y = 0    # mm
        radius = 50     # mm
        num_points = 20 # 圆上的点数
        
        print(f"\n绘制圆形轨迹（圆心: ({center_x}, {center_y}), 半径: {radius}mm）...")
        
        # 计算圆上的点
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            print(f"\r进度: {i}/{num_points}", end="")
            
            if arm.set_arm_position(x, y):
                time.sleep(0.3)
            else:
                print(f"\n警告: 点 ({x:.1f}, {y:.1f}) 不可达")
        
        print("\n\n✅ 圆形轨迹完成！")
        
        # 回到初始位置
        arm.arm_home()
        
    finally:
        robot_sys.shutdown()


def example_yaw_control():
    """
    示例5：云台控制
    
    演示云台/yaw轴的旋转控制
    """
    print("\n" + "="*60)
    print("示例5：云台控制")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        # 复位
        arm.arm_home()
        time.sleep(2)
        
        print("\n云台旋转测试...")
        
        # 向左旋转
        print("\n1. 向左旋转 45°...")
        arm.set_yaw_angle(45)
        time.sleep(2)
        
        # 向右旋转
        print("\n2. 向右旋转 -45°...")
        arm.set_yaw_angle(-45)
        time.sleep(2)
        
        # 大角度旋转
        print("\n3. 向左旋转 90°...")
        arm.set_yaw_angle(90)
        time.sleep(2)
        
        # 回正
        print("\n4. 回正...")
        arm.set_yaw_angle(0)
        time.sleep(2)
        
        print("\n✅ 云台测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_gripper_test():
    """
    示例6：夹爪精细控制
    
    演示夹爪的不同开合程度
    """
    print("\n" + "="*60)
    print("示例6：夹爪精细控制")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        print("\n测试夹爪的不同开合程度...")
        
        # 从完全打开到完全闭合
        print("\n1. 逐渐闭合（10 -> 0）...")
        for i in range(10, -1, -2):
            print(f"   夹爪开度: {i}/10")
            arm.set_gripper(i)
            time.sleep(1)
        
        time.sleep(1)
        
        # 从完全闭合到完全打开
        print("\n2. 逐渐打开（0 -> 10）...")
        for i in range(0, 11, 2):
            print(f"   夹爪开度: {i}/10")
            arm.set_gripper(i)
            time.sleep(1)
        
        print("\n✅ 夹爪测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_pwm_control():
    """
    示例7：PWM输出控制
    
    演示控制主控板的PWM输出引脚
    """
    print("\n" + "="*60)
    print("示例7：PWM输出控制")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        print("\n注意：该功能需要底盘支持PWM控制")
        
        # 测试不同占空比
        print("\n测试引脚1的PWM输出...")
        
        duty_cycles = [0.0, 0.25, 0.5, 0.75, 1.0]
        
        for duty in duty_cycles:
            print(f"\n设置占空比: {duty*100:.0f}%")
            arm.set_pwm(1, duty)
            time.sleep(2)
        
        # 关闭PWM
        print("\n关闭PWM...")
        arm.set_pwm(1, 0.0)
        
        print("\n✅ PWM测试完成！")
        
    finally:
        robot_sys.shutdown()


def example_get_joint_states():
    """
    示例8：实时监控关节状态
    
    演示持续获取机械臂关节角度和末端位置
    """
    print("\n" + "="*60)
    print("示例8：实时监控关节状态")
    print("="*60)
    
    robot_sys = RobotSystem()
    if not robot_sys.initialize("mec"):
        return
    
    try:
        arm = RobotArm()
        
        print("\n持续监控机械臂状态（10秒）...")
        print("按 Ctrl+C 提前停止\n")
        
        # 让机械臂运动
        arm.set_joint_angles(30, -20)
        
        for i in range(10):
            # 获取关节状态
            j1, j2, yaw = arm.get_joint_states()
            
            # 获取末端位置
            x, y = arm.get_arm_pose_xy()
            
            # 显示
            print(f"[{i+1:2d}s] 关节: J1={j1:6.1f}°, J2={j2:6.1f}°, Yaw={yaw:6.1f}° | "
                  f"末端: X={x:6.1f}mm, Y={y:6.1f}mm")
            
            time.sleep(1)
        
        print("\n✅ 监控完成！")
        
        # 回到初始位置
        arm.arm_home()
        
    except KeyboardInterrupt:
        print("\n\n监控被中断")
    
    finally:
        robot_sys.shutdown()


def main():
    """
    主函数：提供交互式菜单选择不同的示例
    """
    examples = {
        "1": ("基本机械臂控制", example_basic_arm_control),
        "2": ("笛卡尔空间控制", example_cartesian_control),
        "3": ("抓取和放置", example_pick_and_place),
        "4": ("画圆形轨迹", example_draw_circle),
        "5": ("云台控制", example_yaw_control),
        "6": ("夹爪精细控制", example_gripper_test),
        "7": ("PWM输出控制", example_pwm_control),
        "8": ("实时监控关节状态", example_get_joint_states),
    }
    
    while True:
        print("\n" + "="*60)
        print("ROS2 机械臂控制功能示例应用")
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
