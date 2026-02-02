# ROS2 功能 Python 封装项目

[English](#english) | [中文](#chinese)

<a name="chinese"></a>

## 📖 项目简介

本项目将 ROS2 (Robot Operating System 2) 的复杂功能封装成简单易用的 Python 接口，让用户**无需学习 ROS2** 即可控制机器人。

### 🎯 核心特性

- ✅ **零 ROS2 知识要求**：无需了解话题、服务、action 等 ROS2 概念
- ✅ **完整功能覆盖**：系统管理、运动控制、机械臂、传感器、建图导航
- ✅ **详细中文注释**：每个函数都有完整的说明和使用示例
- ✅ **模块化设计**：按功能独立划分，可单独使用或组合使用
- ✅ **丰富示例**：提供多个完整的应用示例
- ✅ **支持多车型**：阿克曼、差速、麦轮三种车型

### 🤖 支持的机器人功能

| 功能模块 | 说明 | 主要函数 |
|---------|------|---------|
| **系统管理** | 初始化、关闭、电压、急停 | `initialize()`, `shutdown()`, `get_battery_voltage()` |
| **运动控制** | 速度控制、精确移动、位姿获取 | `set_velocity()`, `move_distance()`, `rotate_angle()` |
| **机械臂** | 关节控制、末端控制、夹爪 | `set_joint_angles()`, `set_arm_position()`, `set_gripper()` |
| **感知** | 雷达、相机、视觉应用 | `launch_lidar()`, `start_visual_follow()`, `start_line_tracking()` |
| **导航** | SLAM建图、自主导航 | `start_mapping()`, `save_map()`, `move_to_goal()` |

---

## 🚀 快速开始

### 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Wheeltec 机器人及配套功能包

### 安装

```bash
# 1. Clone 项目
git clone https://github.com/yxzhang05/Python-encapsulation-of-ROS-functions.git
cd Python-encapsulation-of-ROS-functions

# 2. 确保 ROS2 环境已配置
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash

# 3. 运行示例
python3 demo_app.py
```

### 最简单的示例

```python
from robot_lib_system import RobotSystem

# 创建机器人对象
robot = RobotSystem()

# 初始化（麦轮车型）
robot.initialize("mec")

# 获取电池电压
voltage = robot.get_battery_voltage()
print(f"电池电压: {voltage}V")

# 关闭
robot.shutdown()
```

### 5 分钟上手

```python
from robot_lib_full import Robot
import time

# 使用完整库（推荐）
with Robot() as robot:
    # 初始化
    robot.initialize("mec")
    
    # 前进 1 米
    robot.forward(1.0, speed=0.3)
    
    # 旋转 90 度
    robot.turn(90)
    
    # 获取状态
    robot.print_status()
    
# 自动关闭
```

---

## 📚 文档

### 核心文档

| 文档 | 说明 |
|------|------|
| [📖 使用说明.md](使用说明.md) | 完整的使用教程和 API 参考 |
| [🔧 ROS2封装实现原理.md](ROS2封装实现原理.md) | 详细的实现原理和技术细节 |
| [🌟 ROS2功能Python封装.md](ROS2功能Python封装.md) | 原始需求文档 |

### 模块文档

| 模块 | 文件 | 说明 |
|------|------|------|
| 系统管理 | `robot_lib_system.py` | 初始化、关闭、电压监控 |
| 运动控制 | `robot_lib_motion.py` | 底盘运动、传感器数据 |
| 机械臂 | `robot_lib_arm.py` | 关节控制、末端控制 |
| 感知功能 | `robot_lib_sensors.py` | 雷达、相机、视觉应用 |
| 建图导航 | `robot_lib_navigation.py` | SLAM、路径规划 |
| 完整库 | `robot_lib_full.py` | 整合所有功能 |

---

## 💡 使用示例

### 示例应用

项目提供了多个完整的示例应用：

```bash
# 系统管理示例
python3 system_app.py

# 运动控制示例
python3 motion_app.py

# 机械臂控制示例
python3 arm_app.py

# 感知功能示例
python3 sensors_app.py

# 建图导航示例
python3 navigation_app.py

# 综合演示
python3 demo_app.py
```

### 代码示例

#### 1. 基本运动控制

```python
from robot_lib_motion import RobotMotion

motion = RobotMotion("mec")

# 前进
motion.set_velocity(0.3, 0.0, 0.0)
time.sleep(2)

# 左移（仅麦轮）
motion.set_velocity(0.0, 0.2, 0.0)
time.sleep(2)

# 停止
motion.stop()
```

#### 2. 视觉跟随

```python
from robot_lib_sensors import RobotSensors

sensors = RobotSensors()
sensors.launch_camera()

# 跟随红色物体
sensors.start_visual_follow('red', control_robot=True)

# 按 Ctrl+C 停止
time.sleep(30)
sensors.stop_application()
```

#### 3. SLAM 建图

```python
from robot_lib_navigation import RobotNavigation

nav = RobotNavigation()

# 启动建图
nav.start_mapping("gmapping", visualize=True)

# 键盘控制建图
nav.start_keyboard_control()

# 保存地图
nav.save_map("my_map")
```

#### 4. 自主导航

```python
# 加载地图
nav.load_map_and_start_navigation("my_map")

# 导航到目标点
nav.move_to_goal(x=2.0, y=1.5, theta=90)

# 等待到达
while nav.get_navigation_status() == "navigating":
    time.sleep(1)
```

---

## 🗂️ 项目结构

```
Python-encapsulation-of-ROS-functions/
├── robot_lib_system.py          # 系统管理模块
├── robot_lib_motion.py          # 运动控制模块
├── robot_lib_arm.py             # 机械臂控制模块
├── robot_lib_sensors.py         # 感知功能模块
├── robot_lib_navigation.py      # 建图导航模块
├── robot_lib_full.py            # 完整库（整合所有模块）
│
├── system_app.py                # 系统管理示例
├── motion_app.py                # 运动控制示例
├── arm_app.py                   # 机械臂示例
├── sensors_app.py               # 感知功能示例
├── navigation_app.py            # 建图导航示例
├── demo_app.py                  # 综合演示
│
├── ROS2功能Python封装.md        # 需求文档
├── ROS2封装实现原理.md          # 实现原理文档
├── 使用说明.md                  # 使用教程
├── README.md                    # 本文件
│
├── robot_lib.py                 # 原始参考文件
├── robot_app.py                 # 原始参考文件
│
└── src/                         # ROS2 源代码（不修改）
    ├── turn_on_wheeltec_robot/
    ├── wheeltec_robot_slam/
    ├── wheeltec_robot_nav2/
    └── ...
```

---

## 🎓 学习路径

### 初学者

1. 阅读 [使用说明.md](使用说明.md) 的"快速开始"部分
2. 运行 `system_app.py` 了解基本操作
3. 运行 `motion_app.py` 学习运动控制
4. 尝试修改示例代码

### 进阶用户

1. 阅读 [ROS2封装实现原理.md](ROS2封装实现原理.md)
2. 研究各模块的源代码
3. 运行 `demo_app.py` 查看综合应用
4. 开发自己的应用程序

### 高级用户

1. 理解完整的封装架构
2. 扩展新功能或修改现有功能
3. 为不同的机器人适配本库
4. 贡献代码和改进

---

## 🛠️ 常见问题

### Q: 初始化失败？

**A**: 检查以下几点：
1. ROS2 环境是否正确 source
2. 功能包是否编译成功
3. 硬件是否正确连接
4. 查看终端输出的错误信息

### Q: 传感器数据获取失败？

**A**: 确认：
1. 传感器驱动是否启动
2. 话题是否正常发布：`ros2 topic list`
3. 等待足够的初始化时间

### Q: 运动不准确？

**A**: 可能原因：
1. 地面太滑或不平
2. 速度设置过大
3. 里程计漂移
4. 使用闭环控制函数

详细问题解答请查看 [使用说明.md](使用说明.md) 的"常见问题"部分。

---

## 🤝 贡献

欢迎贡献代码、报告问题或提出建议！

### 贡献方式

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

### 开发指南

- 代码使用中文注释
- 遵循现有的代码风格
- 添加适当的使用示例
- 更新相关文档

---

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 📞 联系方式

- **作者**: yxzhang05
- **GitHub**: https://github.com/yxzhang05/Python-encapsulation-of-ROS-functions
- **问题反馈**: [GitHub Issues](https://github.com/yxzhang05/Python-encapsulation-of-ROS-functions/issues)

---

## 🌟 致谢

- ROS2 社区
- Wheeltec 机器人团队
- 所有贡献者

---

## 📌 更新日志

### v1.0.0 (2026-02-02)

- ✨ 初始版本发布
- ✅ 完成所有核心功能模块
- ✅ 提供完整的中文文档
- ✅ 包含丰富的示例应用

---

<a name="english"></a>

## English

### Project Overview

This project encapsulates ROS2 (Robot Operating System 2) complex functionalities into simple Python interfaces, allowing users to control robots **without learning ROS2**.

### Key Features

- ✅ **Zero ROS2 Knowledge Required**: No need to understand topics, services, actions
- ✅ **Complete Functionality**: System management, motion control, robotic arm, sensors, SLAM & navigation
- ✅ **Detailed Chinese Documentation**: Every function has complete documentation and examples
- ✅ **Modular Design**: Independent modules that can be used separately or together
- ✅ **Rich Examples**: Multiple complete application examples provided
- ✅ **Multi-Robot Support**: Ackermann, differential drive, and mecanum wheel robots

### Quick Start

```python
from robot_lib_full import Robot

with Robot() as robot:
    robot.initialize("mec")
    robot.forward(1.0)
    robot.turn(90)
```

For detailed English documentation, please refer to the source code comments and examples.

---

**祝使用愉快！Happy Coding! 🚀**
