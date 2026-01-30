#!/usr/bin/env python3
"""
机械臂控制示例程序
演示机械臂各种控制功能
"""

from robot_lib import Robot
import time


def main():
    # 创建机器人对象
    bot = Robot()

    try:
        # 1. 初始化机器人
        print("="*50)
        print("机械臂控制示例程序")
        print("="*50)
        
        success = bot.initialize("mec")  # 初始化底盘
        
        if not success:
            print("初始化失败，程序退出")
            return

        # 2. 机械臂复位
        print("\n--- 机械臂复位 ---")
        print("机械臂回到初始位置")
        bot.arm_home()
        time.sleep(3)

        # 3. 获取当前关节状态
        print("\n--- 获取关节状态 ---")
        joint_states = bot.get_joint_states()
        if joint_states:
            print(f"关节1角度: {joint_states[0]:.1f}°")
            print(f"关节2角度: {joint_states[1]:.1f}°")
            print(f"云台角度: {joint_states[2]:.1f}°")

        # 4. 设置关节角度
        print("\n--- 设置关节角度 ---")
        print("设置关节1=45°, 关节2=30°")
        bot.set_joint_angles(45, 30)
        time.sleep(2)

        print("设置关节1=90°, 关节2=60°")
        bot.set_joint_angles(90, 60)
        time.sleep(2)

        # 5. 设置云台角度
        print("\n--- 设置云台角度 ---")
        print("云台左转45°")
        bot.set_yaw_angle(45)
        time.sleep(2)

        print("云台右转45°")
        bot.set_yaw_angle(-45)
        time.sleep(2)

        print("云台回正")
        bot.set_yaw_angle(0)
        time.sleep(1)

        # 6. 末端位置控制 (逆运动学)
        print("\n--- 末端位置控制 ---")
        print("移动末端到位置 (200mm, 150mm)")
        bot.set_arm_position(200, 150)
        time.sleep(3)

        print("移动末端到位置 (250mm, 100mm)")
        bot.set_arm_position(250, 100)
        time.sleep(3)

        # 7. 获取末端位置
        print("\n--- 获取末端位置 ---")
        pose_xy = bot.get_arm_pose_xy()
        if pose_xy:
            print(f"当前末端位置: x={pose_xy[0]:.1f}mm, y={pose_xy[1]:.1f}mm")

        # 8. 夹爪控制
        # 夹爪值从0到10，0表示完全闭合，10表示完全张开
        print("\n--- 夹爪控制 ---")
        print("夹爪完全张开")
        bot.set_gripper(10)  # 最大张开，准备抓取
        time.sleep(2)

        print("夹爪半开")
        bot.set_gripper(5)  # 中等开度
        time.sleep(2)

        print("夹爪闭合")
        bot.set_gripper(0)  # 完全闭合，夹紧物体
        time.sleep(2)

        # 9. PWM控制示例
        # PWM可以控制额外的电机、LED灯等外设
        print("\n--- PWM控制示例 ---")
        print("设置引脚3的PWM为50%")
        bot.set_pwm(3, 0.5)  # 占空比0.5 = 50%功率
        time.sleep(1)

        print("设置引脚3的PWM为100%")
        bot.set_pwm(3, 1.0)  # 占空比1.0 = 100%功率
        time.sleep(1)

        print("关闭引脚3的PWM")
        bot.set_pwm(3, 0.0)  # 占空比0 = 关闭

        # 10. 复合动作示例：完整的抓取-搬运-放置流程
        print("\n--- 复合动作示例：模拟抓取 ---")
        
        print("1. 机械臂移动到目标上方")
        # 使用逆运动学控制，直接指定末端位置坐标
        bot.set_arm_position(200, 200)  # x=200mm, y=200mm (上方位置)
        time.sleep(2)
        
        print("2. 张开夹爪")
        bot.set_gripper(10)  # 完全张开，准备接近物体
        time.sleep(1)
        
        print("3. 下降到抓取位置")
        bot.set_arm_position(200, 150)  # 下降到y=150mm (物体高度)
        time.sleep(2)
        
        print("4. 闭合夹爪")
        bot.set_gripper(0)  # 夹紧物体
        time.sleep(1)
        
        print("5. 提升物体")
        bot.set_arm_position(200, 200)  # 提升回上方位置
        time.sleep(2)
        
        print("6. 移动到放置位置")
        bot.set_arm_position(150, 200)  # 横向移动到新位置x=150mm
        time.sleep(2)
        
        print("7. 放下物体")
        bot.set_gripper(10)
        time.sleep(1)

        # 11. 回到初始位置
        print("\n回到初始位置")
        bot.arm_home()
        time.sleep(2)

        print("\n所有测试完成！")

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n程序异常: {e}")
    finally:
        # 安全关闭
        bot.shutdown()


if __name__ == "__main__":
    main()
