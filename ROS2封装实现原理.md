# ROS2 功能 Python 封装实现原理

## 目录
1. [项目概述](#项目概述)
2. [封装架构](#封装架构)
3. [系统管理模块](#系统管理模块)
4. [底盘运动控制模块](#底盘运动控制模块)
5. [机械臂控制模块](#机械臂控制模块)
6. [感知与功能模块](#感知与功能模块)
7. [建图与导航模块](#建图与导航模块)
8. [核心技术说明](#核心技术说明)

---

## 项目概述

本项目将 ROS2 (Robot Operating System 2) 的复杂功能封装成简单易用的 Python 函数，让用户无需学习 ROS2 即可控制机器人。

### 设计目标

1. **降低使用门槛**：用户无需了解 ROS2 的话题、服务、action 等概念
2. **统一接口**：所有机器人功能通过统一的 Python 类和方法调用
3. **详细注释**：每个函数都有完整的中文注释，说明参数、返回值和使用示例
4. **模块化设计**：按功能划分为独立模块，便于维护和扩展

### 核心封装原理

所有封装都基于以下三个核心技术：

1. **subprocess 进程管理**：启动和管理 ROS2 节点进程
2. **话题通信**：通过 `ros2 topic pub/echo` 发送和接收数据
3. **命令行工具**：利用 ROS2 命令行工具完成各种操作

---

## 封装架构

### 模块划分

```
robot_lib_system.py       # 系统管理（初始化、关闭、电压、急停）
robot_lib_motion.py       # 底盘运动控制（速度、位姿、传感器）
robot_lib_arm.py          # 机械臂控制（关节、末端、夹爪）
robot_lib_sensors.py      # 感知功能（雷达、相机、视觉应用）
robot_lib_navigation.py   # 建图导航（SLAM、路径规划）
```

### 类的继承关系

每个模块都是独立的类，不依赖其他模块，可以单独使用：

```python
# 独立使用单个模块
from robot_lib_system import RobotSystem
robot = RobotSystem()

# 或组合使用多个模块
from robot_lib_system import RobotSystem
from robot_lib_motion import RobotMotion
system = RobotSystem()
motion = RobotMotion()
```

---

## 系统管理模块

### 模块文件
- `robot_lib_system.py` - 系统管理功能库
- `system_app.py` - 系统管理示例应用

### 核心函数

#### 1. initialize(robot_type)

**功能**：初始化机器人系统，启动底盘驱动。

**封装原理**：
```python
# 1. 构建 launch 命令
cmd = [
    "ros2", "launch",
    "turn_on_wheeltec_robot",
    "turn_on_wheeltec_robot.launch.py",
    f"robot_type:={robot_type}"
]

# 2. 后台启动进程
self.driver_process = subprocess.Popen(
    cmd,
    stdout=subprocess.DEVNULL,  # 屏蔽输出
    stderr=subprocess.PIPE       # 保留错误
)

# 3. 等待硬件初始化
time.sleep(5)
```

**原理说明**：
- 使用 `subprocess.Popen` 在后台启动 ROS2 launch 文件
- launch 文件会启动串口驱动、TF变换、底盘控制器等多个节点
- `DEVNULL` 屏蔽标准输出，保持终端清爽
- 等待 5 秒确保硬件（雷达/IMU/串口）完全初始化

#### 2. get_battery_voltage()

**功能**：获取底盘电池电压。

**封装原理**：
```python
# 1. 使用 topic echo 获取一次数据
cmd = [
    "ros2", "topic", "echo",
    "/PowerVoltage",      # 电压话题
    "--once",             # 只接收一次
    "--field", "data"     # 只提取 data 字段
]

# 2. 执行命令并解析输出
output = subprocess.check_output(cmd, timeout=2.0)
voltage = float(output.strip())
```

**原理说明**：
- ROS2 话题 `/PowerVoltage` 实时发布电压数据
- `--once` 参数表示只获取一条消息就退出
- `--field data` 直接提取数值字段，简化解析
- 设置 2 秒超时防止话题无数据时卡死

#### 3. emergency_stop()

**功能**：软件急停，立即停止所有运动。

**封装原理**：
```python
# 构建全 0 速度的 Twist 消息
cmd = [
    "ros2", "topic", "pub",
    "--once",
    "/cmd_vel",
    "geometry_msgs/msg/Twist",
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
]

# 发布命令
subprocess.run(cmd, timeout=1.0)
```

**原理说明**：
- `/cmd_vel` 是底盘速度控制话题
- 发送全 0 的 Twist 消息让机器人立即停止
- 使用 `--once` 只发布一次即可

---

## 底盘运动控制模块

### 模块文件
- `robot_lib_motion.py` - 底盘运动控制功能库
- `motion_app.py` - 运动控制示例应用

### 核心函数

#### 1. set_velocity(v_x, v_y, w_z)

**功能**：设置机器人运动速度（最底层控制接口）。

**封装原理**：
```python
# 1. 构建 Twist 消息
twist_msg = (
    f"{{linear: {{x: {v_x}, y: {v_y}, z: 0.0}}, "
    f"angular: {{x: 0.0, y: 0.0, z: {w_z}}}}}"
)

# 2. 发布到速度话题
cmd = ["ros2", "topic", "pub", "--once", "/cmd_vel", "geometry_msgs/msg/Twist", twist_msg]
subprocess.run(cmd)
```

**原理说明**：
- `Twist` 消息包含线速度（linear）和角速度（angular）
- `v_x`: X 方向线速度（前后）
- `v_y`: Y 方向线速度（左右，仅麦轮有效）
- `w_z`: Z 轴角速度（旋转）

**车型兼容性**：
- **阿克曼 (akm)**：只支持 v_x 和 w_z
- **差速 (diff)**：只支持 v_x 和 w_z
- **麦轮 (mec)**：支持 v_x, v_y, w_z 全向移动

#### 2. move_distance_mecanum(distance_x, distance_y, speed_x, speed_y)

**功能**：麦轮车型移动指定距离（闭环控制）。

**封装原理**：
```python
# 1. 获取起始位姿
start_pose = self._get_current_odom()
start_x, start_y = start_pose["x"], start_pose["y"]

# 2. 计算目标位姿
target_x = start_x + distance_x
target_y = start_y + distance_y

# 3. 持续发送速度命令，直到到达
while True:
    current = self._get_current_odom()
    error_x = target_x - current["x"]
    error_y = target_y - current["y"]
    distance_error = math.sqrt(error_x**2 + error_y**2)
    
    if distance_error < tolerance:
        break  # 到达目标
    
    self.set_velocity(vx, vy, 0.0)
    time.sleep(0.1)

# 4. 停止运动
self.stop()
```

**原理说明**：
- 从里程计（/odom）获取当前位置
- 计算与目标的距离误差
- 持续发送速度命令，并实时检查位置
- 到达目标（误差 < 容差）后停止

#### 3. rotate_angle(angle_degrees, angular_speed)

**功能**：原地旋转指定角度。

**封装原理**：
```python
# 1. 获取起始角度
start_theta = self._get_current_odom()["theta"]

# 2. 计算目标角度（考虑周期性）
target_theta = self._normalize_angle(start_theta + angle_radians)

# 3. 持续旋转直到到达
while True:
    current_theta = self._get_current_odom()["theta"]
    error = self._angle_difference(target_theta, current_theta)
    
    if abs(error) < tolerance:
        break
    
    self.set_velocity(0.0, 0.0, w_z)
    time.sleep(0.1)
```

**原理说明**：
- 角度在 [-π, π] 范围内，需要处理周期性
- `_normalize_angle()` 将角度归一化到标准范围
- `_angle_difference()` 计算两角度的最短差值

#### 4. get_imu_data()

**功能**：获取陀螺仪 6 轴数据。

**封装原理**：
```python
# 1. 订阅 IMU 话题
cmd = ["ros2", "topic", "echo", "/imu/data", "--once"]
output = subprocess.check_output(cmd, timeout=1.0)

# 2. 解析消息（使用正则表达式）
import re
accel_match = re.search(
    r"linear_acceleration:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
    output, re.DOTALL
)
gyro_match = re.search(
    r"angular_velocity:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)",
    output, re.DOTALL
)

# 3. 提取数值
imu_data = {
    "accel_x": float(accel_match.group(1)),
    "accel_y": float(accel_match.group(2)),
    ...
}
```

**原理说明**：
- IMU 消息类型为 `sensor_msgs/msg/Imu`
- 包含加速度（linear_acceleration）和角速度（angular_velocity）
- 使用正则表达式从文本输出中提取数值

---

## 机械臂控制模块

### 模块文件
- `robot_lib_arm.py` - 机械臂控制功能库
- `arm_app.py` - 机械臂控制示例应用

### 核心函数

#### 1. set_joint_angles(joint1, joint2)

**功能**：直接设置关节角度。

**封装原理**：
```python
# 1. 转换为弧度
joint1_rad = math.radians(joint1)
joint2_rad = math.radians(joint2)

# 2. 构建消息（Float64MultiArray）
msg = f"{{data: [{joint1_rad}, {joint2_rad}]}}"

# 3. 发布到关节控制话题
cmd = [
    "ros2", "topic", "pub", "--once",
    "/arm_joint_command",
    "std_msgs/msg/Float64MultiArray",
    msg
]
```

**原理说明**：
- 关节控制通常使用弧度制
- 底层驱动接收角度后转换为电机指令
- 需要检查关节限位，防止超出范围

#### 2. set_arm_position(x, y)

**功能**：笛卡尔空间控制（末端位置控制）。

**封装原理**：
```python
# 1. 逆运动学求解
def _inverse_kinematics(x, y):
    L1 = self.arm_length_1
    L2 = self.arm_length_2
    
    # 计算 joint2（余弦定理）
    distance = math.sqrt(x**2 + y**2)
    cos_joint2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
    joint2_rad = math.acos(cos_joint2)
    
    # 计算 joint1
    alpha = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(joint2_rad),
                     L1 + L2 * math.cos(joint2_rad))
    joint1_rad = alpha - beta
    
    return (joint1, joint2)

# 2. 调用关节控制
solution = self._inverse_kinematics(x, y)
self.set_joint_angles(solution[0], solution[1])
```

**原理说明**：
- 正运动学：已知关节角度 → 计算末端位置
- 逆运动学：已知末端位置 → 计算关节角度
- 两关节平面机械臂使用几何方法求解

#### 3. set_gripper(value)

**功能**：控制夹爪开合。

**封装原理**：
```python
# 1. 将 0-10 映射到实际控制范围
value = max(0, min(10, value))

# 2. 发布到夹爪控制话题
msg = f"{{data: {value}}}"
cmd = ["ros2", "topic", "pub", "--once", "/gripper_command", "std_msgs/msg/Float32", msg]
```

**原理说明**：
- 0 表示完全闭合，10 表示完全打开
- 底层驱动将数值转换为舵机 PWM 信号

---

## 感知与功能模块

### 模块文件
- `robot_lib_sensors.py` - 感知功能库
- `sensors_app.py` - 感知功能示例应用

### 核心函数

#### 1. launch_lidar(visualize)

**功能**：启动激光雷达驱动。

**封装原理**：
```python
# 1. 启动雷达驱动 launch 文件
cmd = [
    "ros2", "launch",
    "wheeltec_lidar_ros2",
    "wheeltec_lidar.launch.py"
]

# 2. 后台启动进程
self.lidar_process = subprocess.Popen(cmd, ...)

# 3. 等待初始化
time.sleep(3)

# 4. 可选启动 RViz 可视化
if visualize:
    subprocess.Popen(["rviz2"], ...)
```

**原理说明**：
- launch 文件启动雷达驱动节点
- 驱动节点读取雷达数据并发布到 `/scan` 话题
- RViz 订阅 `/scan` 话题显示雷达扫描图

#### 2. start_visual_follow(color, control_robot, callback)

**功能**：视觉跟随（识别并跟随指定颜色物体）。

**封装原理**：
```python
# 1. 检查相机是否启动
if not self.camera_process:
    self.launch_camera()

# 2. 启动视觉跟随节点
cmd = [
    "ros2", "launch",
    "wheeltec_robot_kcf",
    "visual_follow.launch.py",
    f"target_color:={color}",
    f"auto_control:={'true' if control_robot else 'false'}"
]

self.application_process = subprocess.Popen(cmd, ...)
```

**工作流程**：
1. 节点订阅相机图像 `/camera/image_raw`
2. 在图像中检测指定颜色（HSV 颜色空间）
3. 计算目标在画面中的位置和面积
4. 如果 `control_robot=True`，根据偏差计算速度并发布到 `/cmd_vel`
5. 如果提供了 `callback`，定期调用回调传递目标信息

**HSV 颜色阈值**（预设）：
```python
COLOR_RANGES = {
    'red':    ([0, 100, 100], [10, 255, 255]),
    'blue':   ([100, 100, 100], [130, 255, 255]),
    'green':  ([50, 100, 100], [70, 255, 255]),
    'yellow': ([20, 100, 100], [30, 255, 255])
}
```

#### 3. get_lidar_distance(angle_degrees)

**功能**：获取雷达指定角度的距离。

**封装原理**：
```python
# 1. 订阅一次 LaserScan 消息
cmd = ["ros2", "topic", "echo", "/scan", "--once"]
output = subprocess.check_output(cmd, timeout=2.0)

# 2. 解析 LaserScan 消息
angle_min = ...      # 起始角度
angle_increment = ... # 角度增量
ranges = [...]       # 距离数组

# 3. 计算索引
target_angle = math.radians(angle_degrees)
index = int((target_angle - angle_min) / angle_increment)

# 4. 返回距离
distance = ranges[index]
```

**LaserScan 消息结构**：
```
angle_min: -3.14159...    # 起始角度（弧度）
angle_max: 3.14159...     # 结束角度（弧度）
angle_increment: 0.0175   # 角度增量（弧度）
ranges: [1.23, 2.45, ...] # 距离数组（米）
```

---

## 建图与导航模块

### 模块文件
- `robot_lib_navigation.py` - 建图导航功能库
- `navigation_app.py` - 建图导航示例应用

### 核心函数

#### 1. start_mapping(method, visualize)

**功能**：启动 SLAM 建图。

**封装原理**：
```python
# 1. 根据算法选择 launch 文件
launch_configs = {
    "gmapping": ("wheeltec_robot_slam", "gmapping.launch.py"),
    "cartographer": ("wheeltec_robot_slam", "cartographer.launch.py"),
}

# 2. 启动建图节点
cmd = ["ros2", "launch", package, launch_file]
self.mapping_process = subprocess.Popen(cmd, ...)

# 3. 可选启动 RViz 显示建图过程
if visualize:
    subprocess.Popen(["rviz2"], ...)
```

**SLAM 工作原理**：
1. 节点订阅 `/scan`（雷达）和 `/odom`（里程计）
2. 使用粒子滤波（gmapping）或图优化（cartographer）构建地图
3. 实时地图发布到 `/map` 话题
4. RViz 订阅 `/map` 显示建图过程

**非阻塞设计**：
- 建图在后台运行，函数立即返回
- 可以同时执行其他操作（如读取传感器、控制运动）

#### 2. save_map(map_name)

**功能**：保存当前构建的地图。

**封装原理**：
```python
# 1. 确保地图目录存在
os.makedirs("maps", exist_ok=True)

# 2. 调用 map_saver 服务
cmd = [
    "ros2", "run",
    "nav2_map_server", "map_saver_cli",
    "-f", f"maps/{map_name}"
]

result = subprocess.run(cmd, timeout=10.0)
```

**保存文件**：
- `map_name.pgm`：地图图像（灰度图）
  - 黑色：障碍物
  - 白色：自由空间
  - 灰色：未探索区域
- `map_name.yaml`：地图元数据
  ```yaml
  image: map_name.pgm
  resolution: 0.05      # 每像素的实际距离（米）
  origin: [-10.0, -10.0, 0.0]  # 地图原点坐标
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```

#### 3. move_to_goal(x, y, theta, callback)

**功能**：导航到目标点。

**封装原理**：
```python
# 1. 将角度转换为四元数
theta_rad = math.radians(theta)
qz = math.sin(theta_rad / 2.0)
qw = math.cos(theta_rad / 2.0)

# 2. 构建 PoseStamped 消息
pose_msg = f"""
{{
  header: {{frame_id: 'map'}},
  pose: {{
    position: {{x: {x}, y: {y}, z: 0.0}},
    orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}
  }}
}}
"""

# 3. 发布到导航目标话题
cmd = ["ros2", "topic", "pub", "--once", "/goal_pose", "geometry_msgs/msg/PoseStamped", pose_msg]
subprocess.run(cmd)
```

**Nav2 导航流程**：
1. **定位**：AMCL 使用粒子滤波在地图中定位机器人
2. **全局规划**：A* 或 Dijkstra 算法规划从当前位置到目标的路径
3. **局部规划**：DWA 或 TEB 算法在局部窗口内规划轨迹
4. **避障**：实时使用雷达数据避开动态障碍物
5. **控制**：计算速度命令并发布到 `/cmd_vel`

---

## 核心技术说明

### 1. subprocess 进程管理

**为什么使用 subprocess**：
- ROS2 节点需要独立进程运行
- 需要管理多个并发的节点进程
- 需要能够启动、停止和监控进程状态

**关键技术点**：
```python
# 启动后台进程
proc = subprocess.Popen(
    cmd,
    stdout=subprocess.DEVNULL,  # 屏蔽标准输出
    stderr=subprocess.PIPE       # 保留错误信息
)

# 检查进程是否还在运行
if proc.poll() is None:
    print("进程正在运行")

# 优雅关闭进程
proc.send_signal(signal.SIGINT)  # 发送 Ctrl+C 信号
proc.wait(timeout=5)              # 等待进程退出

# 强制杀死进程
proc.kill()
```

### 2. ROS2 话题通信

**话题发布**：
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "..."
```

**话题订阅**：
```bash
ros2 topic echo /scan --once
```

**消息类型**：
- `geometry_msgs/msg/Twist`：速度控制
- `sensor_msgs/msg/LaserScan`：雷达扫描
- `sensor_msgs/msg/Image`：图像数据
- `nav_msgs/msg/Odometry`：里程计
- `geometry_msgs/msg/PoseStamped`：位姿

### 3. 坐标系与变换

**常用坐标系**：
- `map`：地图坐标系（固定）
- `odom`：里程计坐标系（随时间漂移）
- `base_link`：机器人本体坐标系
- `laser`：雷达坐标系

**TF 变换**：
- ROS2 使用 TF 库管理坐标系变换
- launch 文件会自动发布各坐标系之间的变换关系

### 4. 非阻塞与回调

**设计原则**：
- 所有耗时操作都在后台进行
- 函数立即返回，不阻塞主程序
- 支持回调函数获取异步结果

**实现方式**：
```python
# 启动后台进程
self.process = subprocess.Popen(...)

# 立即返回
return True

# 用户可以继续执行其他操作
while True:
    # 读取传感器
    data = get_sensor_data()
    # 控制机器人
    control_robot(data)
```

---

## 总结

本项目通过以下技术实现了 ROS2 功能的 Python 封装：

1. **进程管理**：使用 subprocess 管理 ROS2 节点生命周期
2. **话题通信**：通过 `ros2 topic` 命令进行数据交互
3. **数据解析**：使用正则表达式解析话题消息
4. **闭环控制**：结合传感器反馈实现精确运动控制
5. **非阻塞设计**：所有功能支持后台运行和异步回调

**优点**：
- ✅ 无需安装 rclpy Python 库
- ✅ 无需编写 ROS2 代码
- ✅ 简单易用，适合快速开发
- ✅ 详细注释，便于学习和维护

**局限性**：
- ⚠️ 性能略低于直接使用 rclpy
- ⚠️ 某些高级功能可能需要修改底层驱动
- ⚠️ 依赖 ROS2 命令行工具的稳定性

---

**文档版本**：v1.0  
**最后更新**：2026-02-02  
**作者**：自动生成
