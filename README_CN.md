# ROS2功能Python封装使用说明

## 简介

本项目将ROS2机器人的复杂功能封装成简单易用的Python接口，让您无需学习ROS2也能轻松控制机器人。

### 主要特点

- ✅ **简单易用**：将 `ros2 launch` 等复杂命令封装为简单的Python函数
- ✅ **统一接口**：支持阿克曼、差速、麦轮三种车型
- ✅ **功能完整**：涵盖底盘控制、传感器、建图导航、机械臂等功能
- ✅ **丰富示例**：提供多个完整的应用示例程序

### 快速对比

**使用封装前：**
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch wheeltec_slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f my_map
```

**使用封装后：**
```python
from robot_lib import Robot

robot = Robot()
robot.initialize("mec")              # 初始化机器人
robot.launch_lidar()                 # 启动雷达
robot.start_mapping("slam_toolbox")  # 启动建图
robot.save_map("my_map")             # 保存地图
robot.shutdown()                     # 关闭系统
```

---

## 快速开始

### 1. 环境要求

- Ubuntu 22.04 或更高版本
- ROS2 Humble 或更高版本
- Python 3.8+
- Wheeltec机器人硬件（或兼容的ROS2机器人）

### 2. 安装

```bash
# 克隆仓库
cd ~/wheeltec_ros2
git clone https://github.com/yxzhang05/Python-encapsulation-of-ROS-functions.git

# 进入项目目录
cd Python-encapsulation-of-ROS-functions

# 给示例程序添加执行权限
chmod +x *.py
```

### 3. 第一个程序

创建文件 `test.py`：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from robot_lib import Robot
import time

# 创建机器人对象
robot = Robot()

try:
    # 初始化（根据你的车型选择: akm/diff/mec）
    print("正在初始化机器人...")
    if robot.initialize("mec"):  # 麦轮车型
        print("初始化成功！")
        
        # 检查电压
        voltage = robot.get_battery_voltage()
        print(f"当前电压: {voltage:.2f}V")
        
        # 前进0.5米
        print("前进0.5米...")
        robot.move_distance(0.5, 0.2)
        
        # 旋转90度
        print("旋转90度...")
        robot.rotate_angle(90)
        
        print("测试完成！")
    else:
        print("初始化失败")
        
except KeyboardInterrupt:
    print("用户中断")
finally:
    # 安全关闭
    robot.shutdown()
```

运行：
```bash
python3 test.py
```

---

## 功能概览

### 系统管理

| 函数 | 说明 | 示例 |
|------|------|------|
| `initialize(robot_type)` | 初始化机器人 | `robot.initialize("mec")` |
| `shutdown()` | 关闭系统 | `robot.shutdown()` |
| `emergency_stop()` | 紧急停止 | `robot.emergency_stop()` |
| `get_battery_voltage()` | 获取电压 | `voltage = robot.get_battery_voltage()` |

### 底盘控制

| 函数 | 说明 | 示例 |
|------|------|------|
| `set_velocity(vx, vy, wz)` | 设置速度 | `robot.set_velocity(0.5, 0, 0)` |
| `move_distance(dist, speed)` | 移动距离 | `robot.move_distance(1.0, 0.3)` |
| `rotate_angle(angle, speed)` | 旋转角度 | `robot.rotate_angle(90, 0.5)` |
| `get_robot_pose()` | 获取位姿 | `x, y, yaw = robot.get_robot_pose()` |

### 传感器控制

| 函数 | 说明 | 示例 |
|------|------|------|
| `launch_lidar()` | 启动雷达 | `robot.launch_lidar()` |
| `stop_lidar()` | 关闭雷达 | `robot.stop_lidar()` |
| `launch_camera()` | 启动相机 | `robot.launch_camera()` |
| `stop_camera()` | 关闭相机 | `robot.stop_camera()` |

### 应用功能

| 函数 | 说明 | 示例 |
|------|------|------|
| `start_visual_follow(color)` | 视觉跟随 | `robot.start_visual_follow("red")` |
| `start_line_tracking(color)` | 视觉巡线 | `robot.start_line_tracking("black")` |
| `start_lidar_follow(dist)` | 雷达跟随 | `robot.start_lidar_follow(0.8)` |
| `stop_application()` | 停止应用 | `robot.stop_application()` |

### 建图与导航

| 函数 | 说明 | 示例 |
|------|------|------|
| `start_mapping(method)` | 启动建图 | `robot.start_mapping("slam_toolbox")` |
| `save_map(name)` | 保存地图 | `robot.save_map("my_map")` |
| `start_navigation(map)` | 启动导航 | `robot.start_navigation("map.yaml")` |
| `move_to_goal(x, y, theta)` | 导航到目标 | `robot.move_to_goal(2.0, 1.0, 90)` |

### 机械臂控制

| 函数 | 说明 | 示例 |
|------|------|------|
| `arm_home()` | 机械臂复位 | `robot.arm_home()` |
| `set_joint_angles(j1, j2)` | 设置关节角度 | `robot.set_joint_angles(45, 90)` |
| `set_gripper(value)` | 控制夹爪 | `robot.set_gripper(5)` |
| `set_arm_position(x, y)` | 设置末端位置 | `robot.set_arm_position(200, 100)` |

---

## 使用示例

### 示例1：底盘控制

```bash
# 运行底盘控制示例
python3 chassis_control.py
```

**功能：**
- 初始化机器人
- 速度控制
- 距离移动
- 旋转控制
- 位姿获取

### 示例2：传感器应用

```bash
# 运行传感器示例
python3 sensor_app.py
```

**功能：**
- 雷达控制和数据获取
- 相机控制和图像采集
- 视觉跟随演示
- 视觉巡线演示
- 雷达跟随演示

### 示例3：建图与导航

```bash
# 运行建图导航示例
python3 mapping_app.py
```

**功能：**
- SLAM建图（支持多种算法）
- 地图保存
- 自主导航
- 多点导航

### 示例4：机械臂控制

```bash
# 运行机械臂示例
python3 arm_control.py
```

**功能：**
- 关节控制
- 位置控制（逆运动学）
- 夹爪控制
- 抓取和放置流程

---

## 实用案例

### 案例1：自动巡线小车

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

try:
    # 初始化
    robot.initialize("diff")
    
    # 启动相机
    robot.launch_camera()
    time.sleep(3)
    
    # 启动视觉巡线（黑线）
    robot.start_line_tracking("black")
    
    print("巡线运行中，按Ctrl+C停止...")
    while True:
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n停止巡线")
    robot.stop_application()
finally:
    robot.shutdown()
```

### 案例2：雷达避障探索

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

try:
    robot.initialize("mec")
    robot.launch_lidar()
    time.sleep(3)
    
    # 简单的避障探索算法
    for i in range(10):
        # 前进
        robot.move_distance(0.5, 0.3)
        
        # 获取雷达数据检查前方
        # 如果前方有障碍，转向
        robot.rotate_angle(45)
        
finally:
    robot.shutdown()
```

### 案例3：自动建图

```python
#!/usr/bin/env python3
from robot_lib import Robot

robot = Robot()

try:
    robot.initialize("mec")
    robot.launch_lidar()
    
    # 启动建图
    robot.start_mapping("slam_toolbox")
    
    # 执行方形路径建图
    for i in range(4):
        print(f"第{i+1}边...")
        robot.move_distance(2.0, 0.3)  # 前进2米
        robot.rotate_angle(90)          # 转90度
    
    # 保存地图
    robot.save_map("auto_map")
    print("地图已保存: auto_map.pgm")
    
finally:
    robot.shutdown()
```

### 案例4：定点配送

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

try:
    robot.initialize("mec")
    robot.launch_lidar()
    
    # 启动导航
    robot.start_navigation("warehouse.yaml")
    
    # 定义配送点
    delivery_points = [
        (2.0, 1.0, 0),    # A点
        (3.0, 3.0, 90),   # B点
        (1.0, 4.0, 180),  # C点
        (0.0, 0.0, -90)   # 返回起点
    ]
    
    for i, (x, y, theta) in enumerate(delivery_points, 1):
        print(f"前往配送点{i}...")
        robot.move_to_goal(x, y, theta)
        time.sleep(20)  # 等待到达
        print(f"已到达配送点{i}")
        time.sleep(5)   # 等待卸货
    
    print("配送完成！")
    
finally:
    robot.shutdown()
```

---

## 支持的车型

### 1. 阿克曼车型（akm）

**特点：**
- 类似汽车的转向方式
- 适合高速移动
- 转弯半径较大

**控制：**
```python
robot.initialize("akm")
robot.set_velocity(0.5, 0, 0.1)  # vx, 0, wz
```

### 2. 差速车型（diff）

**特点：**
- 左右轮独立控制
- 可原地旋转
- 适合室内环境

**控制：**
```python
robot.initialize("diff")
robot.set_velocity(0.5, 0, 0.5)  # vx, 0, wz
```

### 3. 麦轮车型（mec）

**特点：**
- 全向移动
- 可横向移动
- 适合狭窄空间

**控制：**
```python
robot.initialize("mec")
robot.set_velocity(0.5, 0.3, 0.2)  # vx, vy, wz（全向）
```

---

## 常见问题

### Q1: 程序报错"无法找到ROS2"

**A:** 确保已安装ROS2并source环境：
```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
```

建议添加到 `~/.bashrc`：
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/wheeltec_ros2/install/setup.bash" >> ~/.bashrc
```

### Q2: 初始化失败

**A:** 检查：
1. 硬件是否连接（串口、USB等）
2. ROS2包是否编译：`colcon build`
3. 设备权限：`sudo chmod 777 /dev/ttyUSB*`

### Q3: 速度控制无效

**A:** 确认：
1. 底盘是否正确初始化
2. 话题是否正常：`ros2 topic list | grep cmd_vel`
3. 电机是否使能

### Q4: 建图效果不好

**A:** 建议：
1. 缓慢移动机器人
2. 避免快速旋转
3. 确保雷达数据稳定
4. 选择特征丰富的环境

### Q5: 导航无法到达目标

**A:** 检查：
1. 是否正确设置初始位置（RViz 2D Pose Estimate）
2. 目标点是否在地图内
3. 路径是否被障碍物阻挡
4. 导航参数是否合适

### Q6: 如何查看机器人状态？

**A:** 使用ROS2命令：
```bash
# 查看所有话题
ros2 topic list

# 查看速度指令
ros2 topic echo /cmd_vel

# 查看里程计
ros2 topic echo /odom

# 查看雷达数据
ros2 topic echo /scan
```

### Q7: 如何可视化？

**A:** 使用RViz2：
```bash
# 启动RViz2
rviz2

# 或使用预配置文件
rviz2 -d ~/wheeltec_ros2/src/wheeltec_rviz2/rviz/navigate.rviz
```

### Q8: 程序异常退出后机器人还在动

**A:** 手动停止：
```bash
# 发送停止指令
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 或杀掉所有ROS2进程
pkill -9 ros2
```

---

## 进阶使用

### 自定义配置

创建 `config.json`：
```json
{
    "robot_type": "mec",
    "max_speed": 0.5,
    "max_angular_speed": 1.0,
    "target_distance": 0.8
}
```

在代码中使用：
```python
import json

with open('config.json') as f:
    config = json.load(f)

robot.initialize(config['robot_type'])
```

### 日志记录

添加日志功能：
```python
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='robot.log'
)

logging.info("机器人初始化成功")
```

### 错误恢复

添加错误恢复机制：
```python
def safe_move(robot, distance):
    max_retries = 3
    for i in range(max_retries):
        try:
            return robot.move_distance(distance, 0.3)
        except Exception as e:
            print(f"尝试 {i+1}/{max_retries} 失败: {e}")
            time.sleep(1)
    return False
```

---

## 文件说明

- `robot_lib.py` - 核心库文件，包含所有封装功能
- `robot_app.py` - 原始示例（保持不变）
- `chassis_control.py` - 底盘控制示例
- `sensor_app.py` - 传感器应用示例
- `mapping_app.py` - 建图导航示例
- `arm_control.py` - 机械臂控制示例
- `ENCAPSULATION_GUIDE.md` - 详细封装说明文档
- `README_CN.md` - 本文档

---

## 更新日志

### v1.0.0 (2024)
- ✅ 完成基础框架
- ✅ 实现系统管理功能
- ✅ 实现底盘控制功能
- ✅ 实现传感器控制功能
- ✅ 实现建图导航功能
- ✅ 实现机械臂控制框架
- ✅ 提供完整示例程序
- ✅ 完成中文文档

---

## 贡献

欢迎提交问题和改进建议！

## 许可证

本项目遵循原仓库的许可证。

## 联系方式

- 项目地址：https://github.com/yxzhang05/Python-encapsulation-of-ROS-functions
- 问题反馈：在GitHub上提交Issue

---

## 致谢

感谢Wheeltec团队提供的ROS2机器人平台和功能包。
