# ROS2 功能 Python 封装实现原理

## 目录
1. [项目概述](#项目概述)
2. [封装架构](#封装架构)
3. [系统管理函数](#系统管理函数)
4. [底盘运动控制函数](#底盘运动控制函数)
5. [机械臂控制函数](#机械臂控制函数)
6. [感知与功能函数](#感知与功能函数)
7. [建图与导航函数](#建图与导航函数)
8. [核心技术说明](#核心技术说明)

---

## 项目概述

本项目将 ROS2 的复杂功能封装成简单易用的 Python 函数，让用户无需学习 ROS2 即可控制机器人。

### 设计目标

1. **降低使用门槛**：用户无需了解 ROS2 的话题、服务、action 等概念
2. **统一接口**：所有机器人功能通过统一的 Python 类和方法调用
3. **累积式设计**：每个文件包含前面所有功能，逐步增加新功能
4. **简洁注释**：采用原示例文件的注释风格，简单实用

### 核心封装原理

所有封装都基于以下技术：

1. **subprocess 进程管理**：启动和管理 ROS2 节点进程
2. **话题通信**：通过 `ros2 topic pub/echo` 发送和接收数据
3. **命令行工具**：利用 ROS2 命令行工具完成各种操作

---

## 封装架构

### 累积式文件结构

```
robot_lib_system.py                      # 系统管理
robot_lib_system_chassis.py              # 系统管理 + 底盘运动
robot_lib_system_chassis_arm.py          # 系统 + 底盘 + 机械臂
robot_lib_system_chassis_arm_sensors.py  # 系统 + 底盘 + 机械臂 + 感知
robot_lib_full.py                        # 完整版（包含所有功能）
```

每个文件都包含前面文件的所有功能，并新增一类功能。

---

## 系统管理函数

本节说明系统管理相关函数的封装原理，每个函数都对比ROS原生实现方式。

### 1. initialize(robot_type)

**功能**：初始化机器人底盘

**ROS原生实现方式**：
```bash
# 在终端直接运行
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py robot_type:=mecanum
```

**Python封装实现**：
```python
def initialize(self, robot_type):
    real_type_name = self.ROBOT_TYPE_MAP[robot_type]
    cmd = ["ros2", "launch", "turn_on_wheeltec_robot", 
           "turn_on_wheeltec_robot.launch.py", f"robot_type:={real_type_name}"]
    self.driver_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    time.sleep(5)  # 等待硬件初始化
    return self.driver_process.poll() is None
```

**对比说明**：
- **ROS方式**：需要手动在终端输入长命令，记住launch文件路径和参数
- **Python方式**：一个函数调用，自动处理启动和初始化，隐藏底层细节

---

### 2. shutdown()

**功能**：关闭机器人系统

**ROS原生实现方式**：
```bash
# 在终端按 Ctrl+C，或者查找进程ID
ps aux | grep ros2
# 然后手动终止
terminate <进程ID>
```

**Python封装实现**：
```python
def shutdown(self):
    if self.driver_process:
        self.driver_process.send_signal(signal.SIGINT)  # 发送Ctrl+C信号
        try:
            self.driver_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.driver_process.terminate()  # 强制结束
```

**对比说明**：
- **ROS方式**：需要手动查找进程ID或在多个终端中按Ctrl+C
- **Python方式**：一个函数自动优雅关闭，超时则强制结束

---

### 3. get_battery_voltage()

**功能**：获取电池电压

**ROS原生实现方式**：
```bash
# 在终端运行，手动查看输出
ros2 topic echo /PowerVoltage --once --field data
# 输出：12.5
# 需要人工读取数值
```

**Python封装实现**：
```python
def get_battery_voltage(self):
    cmd = ["ros2", "topic", "echo", "/PowerVoltage", "--once", "--field", "data"]
    output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
    voltage = float(output.strip())
    return voltage
```

**对比说明**：
- **ROS方式**：需要知道话题名称，手动解析输出文本
- **Python方式**：直接返回浮点数，自动处理异常和超时

---

## 底盘运动控制函数

### 1. set_velocity(v_x, v_y, w_z)

**功能**：设置运动速度

**ROS原生实现方式**：
```bash
# 手动发布速度消息，每次都要输入完整格式
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

**Python封装实现**：
```python
def set_velocity(self, v_x, v_y=0.0, w_z=0.0):
    twist_msg = f"{{linear: {{x: {v_x}, y: {v_y}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {w_z}}}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/cmd_vel", "geometry_msgs/msg/Twist", twist_msg]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
```

**对比说明**：
- **ROS方式**：每次都要手动输入完整的消息格式，容易出错
- **Python方式**：简单的函数参数，自动构建消息

---

### 2. move_distance(distance, speed)

**功能**：移动指定距离

**ROS原生实现方式**：
```python
# 需要编写完整的ROS节点代码（使用rclpy）
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveDistanceNode(Node):
    def __init__(self):
        super().__init__('move_distance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', 
                                                     self.odom_callback, 10)
        # ... 更多初始化代码
    
    def odom_callback(self, msg):
        # 处理里程计数据
        pass
    
    def move_distance(self, distance):
        # 闭环控制逻辑
        pass

# 需要60-100行代码
```

**Python封装实现**：
```python
def move_distance(self, distance, speed):
    start_pose = self._get_odom()
    target_distance = abs(distance)
    v_x = speed if distance > 0 else -speed
    
    while True:
        current_pose = self._get_odom()
        traveled = math.sqrt((current_pose["x"]-start_pose["x"])**2 + 
                           (current_pose["y"]-start_pose["y"])**2)
        if traveled >= target_distance - 0.05:
            break
        self.set_velocity(v_x, 0.0, 0.0)
        time.sleep(0.1)
    
    self.set_velocity(0, 0, 0)
```

**对比说明**：
- **ROS方式**：需要编写完整的ROS节点（60+行代码），使用rclpy库，理解回调函数机制
- **Python方式**：简单的循环逻辑（10行代码），无需ROS编程知识

---

## 机械臂控制函数

### 1. set_arm_position(x, y)

**功能**：设置末端位置（逆运动学）

**ROS原生实现方式**：
```bash
# 需要先调用逆运动学服务
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: 'arm', pose_stamped: {...}}}"
# 然后发布计算得到的关节角度
ros2 topic pub /arm_joint_command ...
```

**Python封装实现**：
```python
def set_arm_position(self, x, y):
    solution = self._inverse_kinematics(x, y)  # 内置逆运动学
    if solution is None:
        return False
    joint1, joint2 = solution
    return self.set_joint_angles(joint1, joint2)

def _inverse_kinematics(self, x, y):
    # 几何法求解 - 10行代码
    L1, L2 = self.arm_length_1, self.arm_length_2
    distance = math.sqrt(x**2 + y**2)
    cos_joint2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
    joint2_rad = math.acos(cos_joint2)
    alpha = math.atan2(y, x)
    beta = math.atan2(L2 * math.sin(joint2_rad), L1 + L2 * math.cos(joint2_rad))
    return (math.degrees(alpha - beta), math.degrees(joint2_rad))
```

**对比说明**：
- **ROS方式**：需要配置MoveIt，调用逆运动学服务，理解复杂的消息格式
- **Python方式**：内置逆运动学算法，直接使用坐标

---

## 感知与功能函数

### 1. start_visual_follow(color)

**功能**：视觉跟随

**ROS原生实现方式**：
```bash
# 步骤1：启动相机（终端1）
ros2 launch usb_cam camera.launch.py

# 步骤2：启动视觉跟随（终端2）
ros2 launch wheeltec_robot_kcf visual_follow.launch.py target_color:=red

# 需要管理多个终端窗口
```

**Python封装实现**：
```python
def start_visual_follow(self, color):
    if not self.camera_process:
        self.launch_camera()  # 自动启动相机
    
    cmd = ["ros2", "launch", "wheeltec_robot_kcf", "visual_follow.launch.py",
           f"target_color:={color}"]
    self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    time.sleep(2)
    return self.application_process.poll() is None
```

**对比说明**：
- **ROS方式**：需要手动启动多个launch文件，记住依赖关系，管理多个终端
- **Python方式**：一个函数自动处理所有依赖

---

### 2. get_lidar_distance(angle_degrees)

**功能**：获取雷达指定角度的距离

**ROS原生实现方式**：
```bash
# 查看雷达数据
ros2 topic echo /scan --once
# 输出几百行数据：
# angle_min: -3.14159
# angle_max: 3.14159
# angle_increment: 0.0174533
# ranges: [1.2, 1.3, 1.4, 1.5, ... 720个数值 ...]
# 
# 需要手动：
# 1. 找到 angle_min 和 angle_increment
# 2. 计算索引：index = (目标角度 - angle_min) / angle_increment
# 3. 从ranges数组中找到对应索引的值
```

**Python封装实现**：
```python
def get_lidar_distance(self, angle_degrees=0):
    cmd = ["ros2", "topic", "echo", "/scan", "--once"]
    output = subprocess.check_output(cmd, timeout=2.0, stderr=subprocess.DEVNULL).decode("utf-8")
    
    # 自动解析
    angle_min = float(re.search(r"angle_min:\s*([-\d.]+)", output).group(1))
    angle_increment = float(re.search(r"angle_increment:\s*([-\d.]+)", output).group(1))
    ranges = [float(x.strip()) for x in re.search(r"ranges:.*?\[(.*?)\]", output, re.DOTALL).group(1).split(",")]
    
    # 自动计算索引
    target_angle = math.radians(angle_degrees)
    index = int((target_angle - angle_min) / angle_increment)
    return ranges[index] if 0 <= index < len(ranges) else -1
```

**对比说明**：
- **ROS方式**：需要理解LaserScan消息格式，手动计算索引，从大量数据中提取
- **Python方式**：输入角度直接得到距离，自动处理所有解析

---

## 建图与导航函数

### 1. move_to_goal(x, y, theta)

**功能**：导航到目标点

**ROS原生实现方式**：
```bash
# 需要手动计算四元数
# theta=90度 → qz=sin(45°)=0.707, qw=cos(45°)=0.707
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
           orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}"
```

**Python封装实现**：
```python
def move_to_goal(self, x, y, theta=0.0):
    # 自动转换角度到四元数
    theta_rad = math.radians(theta)
    qz = math.sin(theta_rad / 2.0)
    qw = math.cos(theta_rad / 2.0)
    
    pose_msg = f"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/goal_pose", "geometry_msgs/msg/PoseStamped", pose_msg]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
```

**对比说明**：
- **ROS方式**：需要理解四元数，手动计算（或查表）
- **Python方式**：使用常见的角度值（度），自动转换

---

## 核心技术说明

### 1. subprocess 进程管理

**为什么使用 subprocess**：
- ROS2 节点需要独立进程运行
- Python可以启动、监控和终止ROS2进程
- 不依赖rclpy库，不需要配置ROS2 Python环境

**关键代码**：
```python
# 启动进程
proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

# 检查是否在运行
if proc.poll() is None:  # None表示还在运行
    print("进程运行中")

# 优雅关闭
proc.send_signal(signal.SIGINT)
proc.wait(timeout=5)

# 强制终止
proc.terminate()
```

---

### 2. 数据解析技术

**正则表达式解析ROS消息**：
```python
import re

# 简单数值
voltage = float(output.strip())

# 复杂结构
pos_match = re.search(r"position:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+)", output, re.DOTALL)
x = float(pos_match.group(1))
y = float(pos_match.group(2))

# 数组
ranges = [float(x.strip()) for x in ranges_str.split(",")]
```

---

## 总结对比表

| 方面 | ROS原生 | Python封装 |
|------|---------|-----------|
| 学习曲线 | 陡峭（需学习ROS概念） | 平缓（只需Python基础） |
| 命令长度 | 长，难记忆 | 短，一个函数调用 |
| 数据格式 | 复杂（话题、消息类型） | 简单（参数和返回值） |
| 进程管理 | 手动，多个终端 | 自动，后台管理 |
| 数学计算 | 手动（四元数等） | 自动（角度↔四元数） |
| 代码量 | 多（完整节点60+行） | 少（1行函数调用） |

---

**文档版本**：v2.0  
**最后更新**：2026-02-02
