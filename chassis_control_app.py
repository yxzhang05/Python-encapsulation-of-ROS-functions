#!/usr/bin/env python3
"""
底盘控制示例程序
演示基本的底盘运动控制功能
"""

from robot_lib import Robot
import time


def main():
    # 创建机器人对象
    bot = Robot()

    try:
        # 1. 初始化机器人 (根据实际车型选择: "akm", "diff", "mec")
        print("="*50)
        print("底盘控制示例程序")
        print("="*50)
        
        robot_type = "mec"  # 麦轮车型，可以改为 "akm" 或 "diff"
        success = bot.initialize(robot_type)
        
        if not success:
            print("初始化失败，程序退出")
            return

        # 2. 检查电池电压
        voltage = bot.get_battery_voltage()
        print(f"\n当前电池电压: {voltage:.2f} V")
        
        if voltage < 10.5:
            print("【警告】电量过低，请充电！")
            return

        # 3. 获取软件版本
        version = bot.get_software_version()
        print(f"下位机版本: {version}")

        # 4. 基本速度控制示例
        print("\n--- 速度控制示例 ---")
        print("前进 0.2m/s，持续2秒")
        bot.set_velocity(0.2, 0.0, 0.0)
        time.sleep(2)
        bot.emergency_stop()
        time.sleep(1)

        # 5. 移动指定距离示例
        print("\n--- 移动指定距离示例 ---")
        print("前进 0.5米")
        bot.move_distance(0.5, 0.2)
        time.sleep(1)

        print("后退 0.5米")
        bot.move_distance(-0.5, 0.2)
        time.sleep(1)

        # 6. 麦轮全向移动示例 (仅麦轮有效)
        if robot_type == "mec":
            print("\n--- 麦轮全向移动示例 ---")
            print("横向移动 0.3米")
            bot.move_distance(0.0, 0.2, lateral_distance=0.3, lateral_speed=0.2)
            time.sleep(1)

        # 7. 旋转示例
        print("\n--- 旋转示例 ---")
        print("原地左转 90度")
        bot.rotate_angle(90, 0.5)
        time.sleep(1)

        print("原地右转 90度")
        bot.rotate_angle(-90, 0.5)
        time.sleep(1)

        # 8. 弧线运动示例 (阿克曼/差速)
        if robot_type in ["akm", "diff"]:
            print("\n--- 弧线运动示例 ---")
            print("沿半径1米的弧线前进 1米")
            bot.move_distance_with_radius(1.0, 0.2, radius=1.0)
            time.sleep(1)

        # 9. 阿克曼转向示例
        if robot_type == "akm":
            print("\n--- 阿克曼转向示例 ---")
            print("设置转向角度为30度")
            bot.set_ackermann_angle(30)
            time.sleep(1)
            print("回正转向")
            bot.set_ackermann_angle(0)

        # 10. 获取车身位姿
        print("\n--- 获取车身位姿 ---")
        pose = bot.get_pose()
        if pose:
            print(f"当前位置: x={pose['x']:.3f}m, y={pose['y']:.3f}m")
            print(f"当前朝向: yaw={pose['yaw']:.3f}rad")

        # 11. 获取IMU数据
        print("\n--- 获取IMU数据 ---")
        imu = bot.get_imu_data()
        if imu:
            print(f"加速度: {imu['accel']}")
            print(f"角速度: {imu['gyro']}")

        # 12. 获取轮速
        print("\n--- 获取四轮速度 ---")
        wheel_speeds = bot.get_wheel_speeds()
        if wheel_speeds:
            print(f"左前轮: {wheel_speeds['fl']}")
            print(f"右前轮: {wheel_speeds['fr']}")
            print(f"左后轮: {wheel_speeds['rl']}")
            print(f"右后轮: {wheel_speeds['rr']}")

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
