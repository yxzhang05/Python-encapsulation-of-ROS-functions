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
        # 演示最底层的速度控制接口，直接设置机器人的线速度和角速度
        print("\n--- 速度控制示例 ---")
        print("前进 0.2m/s，持续2秒")
        bot.set_velocity(0.2, 0.0, 0.0)  # v_x=0.2m/s前进，v_y=0（横向），w_z=0（不旋转）
        time.sleep(2)  # 保持运动2秒
        bot.emergency_stop()  # 紧急停止，确保安全
        time.sleep(1)

        # 5. 移动指定距离示例
        # 使用闭环控制，通过里程计反馈确保精确移动到指定距离
        print("\n--- 移动指定距离示例 ---")
        print("前进 0.5米")
        bot.move_distance(0.5, 0.2)  # 前进0.5米，速度0.2m/s
        time.sleep(1)

        print("后退 0.5米")
        bot.move_distance(-0.5, 0.2)  # 负距离表示后退
        time.sleep(1)

        # 6. 麦轮全向移动示例 (仅麦轮有效)
        # 麦轮车型可以横向移动，无需先旋转
        if robot_type == "mec":
            print("\n--- 麦轮全向移动示例 ---")
            print("横向移动 0.3米")
            # lateral_distance参数仅对麦轮有效，实现横向平移
            bot.move_distance(0.0, 0.2, lateral_distance=0.3, lateral_speed=0.2)
            time.sleep(1)

        # 7. 旋转示例
        # 原地旋转指定角度，使用里程计的朝向信息进行闭环控制
        print("\n--- 旋转示例 ---")
        print("原地左转 90度")
        bot.rotate_angle(90, 0.5)  # 正角度为左转（逆时针），角速度0.5rad/s
        time.sleep(1)

        print("原地右转 90度")
        bot.rotate_angle(-90, 0.5)  # 负角度为右转（顺时针）
        time.sleep(1)

        # 8. 弧线运动示例 (阿克曼/差速)
        # 阿克曼和差速车型可以沿指定半径的弧线移动
        if robot_type in ["akm", "diff"]:
            print("\n--- 弧线运动示例 ---")
            print("沿半径1米的弧线前进 1米")
            # 通过同时施加线速度和角速度实现弧线运动
            bot.move_distance_with_radius(1.0, 0.2, radius=1.0)
            time.sleep(1)

        # 9. 阿克曼转向示例
        # 阿克曼车型（类似汽车）可以直接设置前轮转向角
        if robot_type == "akm":
            print("\n--- 阿克曼转向示例 ---")
            print("设置转向角度为30度")
            bot.set_ackermann_angle(30)  # 设置前轮向左转30度
            time.sleep(1)
            print("回正转向")
            bot.set_ackermann_angle(0)  # 回正前轮

        # 10. 获取车身位姿
        # 从里程计读取机器人当前的位置和朝向
        print("\n--- 获取车身位姿 ---")
        pose = bot.get_pose()
        if pose:
            print(f"当前位置: x={pose['x']:.3f}m, y={pose['y']:.3f}m")
            print(f"当前朝向: yaw={pose['yaw']:.3f}rad")

        # 11. 获取IMU数据
        # IMU（惯性测量单元）提供加速度和角速度信息
        print("\n--- 获取IMU数据 ---")
        imu = bot.get_imu_data()
        if imu:
            print(f"加速度: {imu['accel']}")  # [x, y, z] 单位: m/s²
            print(f"角速度: {imu['gyro']}")   # [x, y, z] 单位: rad/s

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
