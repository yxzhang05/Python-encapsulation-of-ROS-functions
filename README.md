# Python-encapsulation-of-ROS-functions

把ROS2的部分功能封装成Python，让用户使用的时候不需要学习ROS也可以跑起来功能。

## 项目简介

本项目将ROS2机器人的常用功能封装成简单的Python接口，用户可以通过简单的Python代码控制机器人，而无需了解ROS的复杂命令和概念。

**原来的方式：**
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py robot_type:=ackermann
ros2 launch wheeltec_slam gmapping.launch.py
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

**现在的方式：**
```python
from robot_lib import Robot

bot = Robot()
bot.initialize("akm")  # 初始化阿克曼车型
bot.start_mapping("gmapping")  # 启动建图
bot.start_keyboard_control()  # 键盘控制
```

## 项目结构

```
.
├── robot_lib.py              # 核心库文件，包含所有封装的ROS2功能
├── chassis_control_app.py    # 底盘控制示例
├── arm_control_app.py        # 机械臂控制示例
├── sensor_app.py             # 传感器与感知应用示例
├── mapping_app.py            # SLAM建图示例
├── navigation_app.py         # 自主导航示例
├── robot_app.py              # 原始参考示例
└── README.md                 # 本文档
```

## 核心库 (robot_lib.py)

### 功能模块

#### 1. 系统管理
- `initialize(robot_type)` - 初始化机器人系统
- `shutdown()` - 安全关闭机器人
- `get_battery_voltage()` - 获取电池电压
- `emergency_stop()` - 紧急停止
- `get_software_version()` - 获取软件版本

#### 2. 底盘运动控制
- `set_velocity(v_x, v_y, w_z)` - 设置底盘速度
- `move_distance(distance, speed, ...)` - 移动指定距离
- `move_distance_with_radius(distance, speed, radius)` - 弧线运动（阿克曼/差速）
- `rotate_angle(angle, speed)` - 旋转指定角度
- `get_wheel_speeds()` - 获取四轮速度
- `get_imu_data()` - 获取陀螺仪数据
- `get_pose()` - 获取车身位姿
- `set_wheel_speeds(fl, fr, rl, rr)` - 直接设置轮速
- `set_ackermann_angle(angle)` - 设置阿克曼转向角

#### 3. 机械臂控制
- `arm_home()` - 机械臂复位
- `set_joint_angles(joint1, joint2)` - 设置关节角度
- `set_yaw_angle(angle)` - 设置云台角度
- `set_arm_position(x, y)` - 设置末端位置（逆运动学）
- `set_gripper(value)` - 控制夹爪（0-10）
- `get_arm_pose_xy()` - 获取末端位置
- `get_joint_states()` - 获取关节状态
- `set_pwm(pin, duty_cycle)` - 设置PWM输出

#### 4. 传感器与感知
- `launch_lidar()` - 启动雷达
- `stop_lidar()` - 关闭雷达
- `launch_camera()` - 启动相机
- `stop_camera()` - 关闭相机
- `start_visual_follow(color, control_enabled, callback)` - 视觉跟随
- `start_visual_follow_custom(hsv_lower, hsv_upper, ...)` - 自定义颜色跟随
- `get_visual_target_info()` - 获取视觉目标信息
- `start_line_tracking(color, control_enabled, callback)` - 视觉巡线
- `start_line_tracking_custom(hsv_lower, hsv_upper, ...)` - 自定义颜色巡线
- `get_line_info()` - 获取线条信息
- `start_lidar_follow(target_dist)` - 雷达跟随
- `stop_application()` - 停止应用
- `capture_image()` - 拍照
- `get_lidar_distance(angle)` - 获取指定角度的雷达距离

#### 5. 建图与导航
- `start_mapping(method)` - 启动SLAM建图
- `save_map(map_name)` - 保存地图
- `start_keyboard_control()` - 键盘控制
- `start_navigation(map_name)` - 启动导航系统
- `move_to_goal(x, y, theta)` - 导航到目标点
- `cancel_navigation()` - 取消导航
- `get_navigation_status()` - 获取导航状态
- `get_current_map()` - 获取当前地图数据

## 使用示例

### 1. 底盘控制 (chassis_control_app.py)

```python
from robot_lib import Robot
import time

bot = Robot()
bot.initialize("mec")  # 初始化麦轮车型

# 前进0.5米
bot.move_distance(0.5, 0.2)

# 旋转90度
bot.rotate_angle(90, 0.5)

# 麦轮横向移动
bot.move_distance(0.0, 0.2, lateral_distance=0.3, lateral_speed=0.2)

bot.shutdown()
```

### 2. 机械臂控制 (arm_control_app.py)

```python
from robot_lib import Robot

bot = Robot()
bot.initialize("mec")

# 机械臂复位
bot.arm_home()

# 设置关节角度
bot.set_joint_angles(45, 30)

# 移动末端到指定位置
bot.set_arm_position(200, 150)

# 控制夹爪
bot.set_gripper(0)  # 闭合

bot.shutdown()
```

### 3. 传感器应用 (sensor_app.py)

```python
from robot_lib import Robot

bot = Robot()
bot.initialize("diff")

# 启动雷达并获取距离
bot.launch_lidar()
distance = bot.get_lidar_distance(0)  # 前方距离
print(f"前方距离: {distance}m")

# 视觉跟随
bot.launch_camera()
bot.start_visual_follow("red")  # 跟随红色物体
time.sleep(10)
bot.stop_application()

# 视觉巡线
bot.start_line_tracking("black")  # 黑线巡线
time.sleep(10)
bot.stop_application()

bot.shutdown()
```

### 4. 建图 (mapping_app.py)

```python
from robot_lib import Robot

bot = Robot()
bot.initialize("diff")

# 启动建图
bot.start_mapping("gmapping")

# 方式1: 键盘控制建图
bot.start_keyboard_control()

# 方式2: 自动建图
bot.launch_lidar()
# 自动移动探索...
bot.move_distance(2.0, 0.2)
bot.rotate_angle(90, 0.5)

# 保存地图
bot.save_map("my_map")

bot.shutdown()
```

### 5. 导航 (navigation_app.py)

```python
from robot_lib import Robot

bot = Robot()
bot.initialize("diff")

# 加载地图并启动导航
bot.start_navigation("my_map")

# 导航到目标点
bot.move_to_goal(2.0, 1.5, 90)

# 等待到达
while bot.get_navigation_status() != "reached":
    time.sleep(1)

bot.shutdown()
```

## 运行方式

所有示例程序都可以直接使用Python运行：

```bash
# 底盘控制示例
python3 chassis_control_app.py

# 机械臂控制示例
python3 arm_control_app.py

# 传感器应用示例
python3 sensor_app.py

# 建图示例
python3 mapping_app.py

# 导航示例
python3 navigation_app.py
```

## 注意事项

1. **ROS2环境**: 使用前请确保已安装ROS2和相关的Wheeltec机器人包
2. **权限**: 某些操作可能需要sudo权限（如串口访问）
3. **硬件连接**: 确保机器人硬件已正确连接并上电
4. **地图文件**: 导航前需要先建图，地图文件默认保存在 `~/maps/` 目录
5. **参数调整**: 根据实际机器人型号，可能需要调整话题名称和参数

## 车型支持

- `akm` - 阿克曼转向车型
- `diff` - 差速驱动车型  
- `mec` - 麦轮全向车型

## 高级功能

### 非阻塞建图

建图功能是非阻塞的，可以在建图过程中同时读取传感器数据和控制底盘：

```python
bot.start_mapping("gmapping")  # 非阻塞启动建图
bot.launch_lidar()

# 可以同时执行其他操作
while exploring:
    distance = bot.get_lidar_distance(0)
    if distance < 0.5:
        bot.rotate_angle(90, 0.5)
    else:
        bot.set_velocity(0.2, 0.0, 0.0)

bot.save_map("auto_map")
```

### 自定义颜色识别

视觉跟随和巡线支持自定义HSV颜色范围：

```python
# 自定义橙色范围
hsv_lower = (10, 100, 100)
hsv_upper = (20, 255, 255)
bot.start_visual_follow_custom(hsv_lower, hsv_upper)
```

### 回调函数支持

视觉功能支持回调函数获取实时信息：

```python
def on_target_detected(info):
    print(f"检测到目标: {info}")

bot.start_visual_follow("red", control_enabled=False, callback=on_target_detected)
```

## 待完善功能

以下功能需要根据实际硬件的串口协议进行适配：

- 部分传感器数据的详细解析
- 特定硬件的自定义功能
- 某些功能的回调机制优化

## 技术支持

如有问题，请查看：
1. ROS2日志: `ros2 topic list`, `ros2 topic echo`
2. 硬件连接: `ls /dev/ttyUSB*`
3. 权限问题: `sudo chmod 666 /dev/ttyUSB0`

## 更新日志

- v1.0 (2026-01-28): 初始版本，完成所有基础功能封装
