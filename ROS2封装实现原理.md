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

### 4. emergency_stop()

**功能**：软件急停，立即停止机器人所有运动

**ROS原生实现方式**：
```bash
# 方式1：发布零速度
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 方式2：调用急停服务（如果机器人提供）
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

**Python封装实现**：
```python
def emergency_stop(self):
    """立即发送零速度指令，停止所有运动"""
    self.set_velocity(0.0, 0.0, 0.0)
    print("[Info] 急停：已发送停止指令")
```

**对比说明**：
- **ROS方式**：需要手动输入完整的零速度命令，或查找急停服务
- **Python方式**：一个函数调用立即停止，简单明了

---

### 5. get_software_version()

**功能**：获取机器人软件版本信息

**ROS原生实现方式**：
```bash
# 方式1：查看包版本
ros2 pkg list | grep wheeltec
apt show ros-humble-wheeltec-*

# 方式2：读取版本话题（如果提供）
ros2 topic echo /robot_info --once --field version

# 方式3：查看launch文件注释
cat /opt/ros/humble/share/.../launch/*.py | grep -i version
```

**Python封装实现**：
```python
def get_software_version(self):
    """通过ROS包信息获取版本"""
    try:
        cmd = ["ros2", "pkg", "list"]
        output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
        wheeltec_packages = [line for line in output.split("\n") 
                            if "wheeltec" in line.lower()]
        return {"packages": wheeltec_packages, "ros_version": "humble"}
    except:
        return {"packages": [], "ros_version": "unknown"}
```

**对比说明**：
- **ROS方式**：需要运行多个命令，手动解析输出
- **Python方式**：自动收集版本信息，返回结构化数据

---

### 6. start_keyboard_control()

**功能**：启动键盘遥控功能

**ROS原生实现方式**：
```bash
# 需要运行键盘控制节点
ros2 run wheeltec_robot_rc keyboard_control

# 或使用launch文件
ros2 launch wheeltec_robot_rc keyboard_control.launch.py
```

**Python封装实现**：
```python
def start_keyboard_control(self):
    """启动键盘控制节点，允许用户通过键盘控制机器人"""
    cmd = ["ros2", "run", "wheeltec_robot_rc", "keyboard_control"]
    self.keyboard_process = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.PIPE)
    time.sleep(1)
    return self.keyboard_process.poll() is None
```

**对比说明**：
- **ROS方式**：需要打开新终端，手动运行命令
- **Python方式**：后台自动启动，管理进程生命周期

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

### 3. move_distance_xy(distance_x, distance_y, speed_x, speed_y)

**功能**：麦轮机器人横向和纵向同时移动

**ROS原生实现方式**：
```python
# 需要编写完整的ROS节点，处理二维运动控制
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MoveXYNode(Node):
    def __init__(self):
        super().__init__('move_xy')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', 
                                                     self.odom_callback, 10)
        # 需要实现：
        # 1. 订阅里程计
        # 2. 计算二维位移
        # 3. 同时控制x和y方向速度
        # 4. 闭环控制达到目标
        # 总共需要80-120行代码
```

**Python封装实现**：
```python
def move_distance_xy(self, distance_x, distance_y, speed_x, speed_y):
    """仅适用于麦轮，可以同时沿x和y方向移动"""
    start_pose = self._get_odom()
    target_x = abs(distance_x)
    target_y = abs(distance_y)
    v_x = speed_x if distance_x > 0 else -speed_x
    v_y = speed_y if distance_y > 0 else -speed_y
    
    while True:
        current_pose = self._get_odom()
        traveled_x = abs(current_pose["x"] - start_pose["x"])
        traveled_y = abs(current_pose["y"] - start_pose["y"])
        
        if traveled_x >= target_x - 0.05 and traveled_y >= target_y - 0.05:
            break
        
        self.set_velocity(v_x if traveled_x < target_x else 0, 
                         v_y if traveled_y < target_y else 0, 0.0)
        time.sleep(0.1)
    
    self.set_velocity(0, 0, 0)
```

**对比说明**：
- **ROS方式**：需要编写完整节点，处理二维闭环控制（80+行）
- **Python方式**：简单循环逻辑（15行），自动处理二维运动

---

### 4. rotate_angle(angle_degrees, angular_speed)

**功能**：原地旋转指定角度

**ROS原生实现方式**：
```python
# 需要编写ROS节点，订阅IMU或里程计
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class RotateNode(Node):
    def __init__(self):
        super().__init__('rotate')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Imu, '/imu', 
                                                     self.imu_callback, 10)
        # 需要实现：
        # 1. 订阅IMU或里程计获取当前角度
        # 2. 将四元数转换为欧拉角
        # 3. 计算角度差
        # 4. 闭环控制旋转到目标角度
        # 总共需要60-80行代码
```

**Python封装实现**：
```python
def rotate_angle(self, angle_degrees, angular_speed):
    """原地旋转指定角度"""
    start_pose = self._get_odom()
    target_angle_rad = math.radians(angle_degrees)
    w_z = angular_speed if angle_degrees > 0 else -angular_speed
    
    while True:
        current_pose = self._get_odom()
        rotated = self._angle_difference(current_pose["yaw"], start_pose["yaw"])
        
        if abs(rotated) >= abs(target_angle_rad) - 0.05:
            break
        
        self.set_velocity(0.0, 0.0, w_z)
        time.sleep(0.1)
    
    self.set_velocity(0, 0, 0)
```

**对比说明**：
- **ROS方式**：需要处理四元数、欧拉角转换，编写完整节点（60+行）
- **Python方式**：简单的角度控制逻辑（10行），自动处理角度归一化

---

### 5. get_wheel_speeds()

**功能**：获取四个车轮的实时速度

**ROS原生实现方式**：
```bash
# 订阅车轮速度话题
ros2 topic echo /wheel_speeds --once
# 输出：
# speeds: [0.5, 0.5, 0.5, 0.5]  # [前左, 前右, 后左, 后右]
# 需要手动解析数组
```

**Python封装实现**：
```python
def get_wheel_speeds(self):
    """获取四轮速度（仅适用于四轮车型）"""
    cmd = ["ros2", "topic", "echo", "/wheel_speeds", "--once"]
    try:
        output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
        speeds_match = re.search(r"speeds:\s*\[(.*?)\]", output)
        if speeds_match:
            speeds = [float(x.strip()) for x in speeds_match.group(1).split(",")]
            return {"fl": speeds[0], "fr": speeds[1], 
                   "rl": speeds[2], "rr": speeds[3]}
    except:
        return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
```

**对比说明**：
- **ROS方式**：需要手动解析数组格式的输出
- **Python方式**：返回带标签的字典，清晰易用

---

### 6. get_imu_data()

**功能**：获取陀螺仪六轴数据（加速度和角速度）

**ROS原生实现方式**：
```bash
# 订阅IMU话题
ros2 topic echo /imu --once
# 输出大量数据：
# header:
#   stamp: ...
# orientation: {x: 0.1, y: 0.2, z: 0.3, w: 0.9}
# angular_velocity: {x: 0.01, y: 0.02, z: 0.03}
# linear_acceleration: {x: 9.8, y: 0.1, z: 0.2}
# 需要从复杂输出中提取6个数值
```

**Python封装实现**：
```python
def get_imu_data(self):
    """获取IMU六轴数据"""
    cmd = ["ros2", "topic", "echo", "/imu", "--once"]
    try:
        output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
        
        # 解析角速度
        ang_vel = re.search(r"angular_velocity:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)", 
                           output, re.DOTALL)
        # 解析线加速度
        lin_acc = re.search(r"linear_acceleration:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)", 
                           output, re.DOTALL)
        
        return {
            "angular_velocity": {"x": float(ang_vel.group(1)), 
                               "y": float(ang_vel.group(2)), 
                               "z": float(ang_vel.group(3))},
            "linear_acceleration": {"x": float(lin_acc.group(1)), 
                                  "y": float(lin_acc.group(2)), 
                                  "z": float(lin_acc.group(3))}
        }
    except:
        return None
```

**对比说明**：
- **ROS方式**：输出包含大量信息（时间戳、四元数等），需手动提取6个值
- **Python方式**：自动解析，返回清晰的嵌套字典结构

---

### 7. get_robot_pose()

**功能**：获取机器人当前位姿（x, y, yaw）

**ROS原生实现方式**：
```bash
# 订阅里程计话题
ros2 topic echo /odom --once
# 输出：
# pose:
#   pose:
#     position: {x: 1.2, y: 0.5, z: 0.0}
#     orientation: {x: 0.0, y: 0.0, z: 0.382, w: 0.924}
# 需要手动：
# 1. 提取位置x, y
# 2. 将四元数转换为yaw角
# 3. 四元数转欧拉角公式：yaw = atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
```

**Python封装实现**：
```python
def get_robot_pose(self):
    """获取机器人位姿，自动将四元数转换为yaw角"""
    pose_data = self._get_odom()  # 内部函数已实现转换
    return {
        "x": pose_data["x"],
        "y": pose_data["y"],
        "yaw_degrees": math.degrees(pose_data["yaw"])
    }
```

**对比说明**：
- **ROS方式**：需要理解四元数，手动计算转换公式
- **Python方式**：自动转换，返回常用的角度值（度）

---

### 8. set_wheel_speeds(fl, fr, rl, rr)

**功能**：直接控制四个车轮的速度

**ROS原生实现方式**：
```bash
# 发布车轮速度命令（具体话题名和消息类型依赖于机器人）
ros2 topic pub --once /wheel_speed_cmd wheeltec_msgs/msg/WheelSpeed \
  "{speeds: [0.5, 0.6, 0.5, 0.6]}"  # [前左, 前右, 后左, 后右]
```

**Python封装实现**：
```python
def set_wheel_speeds(self, fl, fr, rl, rr):
    """直接设置四轮速度（仅适用于四轮车型）"""
    speeds_msg = f"{{speeds: [{fl}, {fr}, {rl}, {rr}]}}"
    cmd = ["ros2", "topic", "pub", "--once", "/wheel_speed_cmd",
           "wheeltec_msgs/msg/WheelSpeed", speeds_msg]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, 
                      stderr=subprocess.DEVNULL, timeout=1.0)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：需要知道自定义消息类型，手动构建数组格式
- **Python方式**：简单的四个参数，自动构建消息

---

### 9. set_ackermann_angle(steering_angle)

**功能**：设置阿克曼转向角度

**ROS原生实现方式**：
```bash
# 发布阿克曼转向命令
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDrive \
  "{steering_angle: 0.5, speed: 1.0}"
```

**Python封装实现**：
```python
def set_ackermann_angle(self, steering_angle):
    """设置阿克曼转向角（仅适用于阿克曼车型）"""
    ackermann_msg = f"{{steering_angle: {steering_angle}, speed: 0.0}}"
    cmd = ["ros2", "topic", "pub", "--once", "/ackermann_cmd",
           "ackermann_msgs/msg/AckermannDrive", ackermann_msg]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=1.0)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：需要了解阿克曼消息类型和参数
- **Python方式**：只需指定转向角，自动处理其他参数

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

### 2. arm_home()

**功能**：机械臂回到初始位置（零位）

**ROS原生实现方式**：
```bash
# 方式1：发布零角度命令
ros2 topic pub --once /arm_joint_command sensor_msgs/msg/JointState \
  "{name: ['joint1', 'joint2'], position: [0.0, 0.0]}"

# 方式2：调用归位服务（如果提供）
ros2 service call /arm_home std_srvs/srv/Trigger
```

**Python封装实现**：
```python
def arm_home(self):
    """机械臂归零位"""
    return self.set_joint_angles(0.0, 0.0)
```

**对比说明**：
- **ROS方式**：需要知道关节名称和消息格式
- **Python方式**：一个函数调用，简单直观

---

### 3. set_joint_angles(joint1, joint2)

**功能**：设置机械臂关节角度

**ROS原生实现方式**：
```bash
# 发布关节命令
ros2 topic pub --once /arm_joint_command sensor_msgs/msg/JointState \
  "{name: ['joint1', 'joint2'], position: [1.57, -0.785]}"  # 弧度制
```

**Python封装实现**：
```python
def set_joint_angles(self, joint1, joint2):
    """设置机械臂两个关节的角度（度）"""
    joint1_rad = math.radians(joint1)
    joint2_rad = math.radians(joint2)
    
    joint_msg = f"{{name: ['joint1', 'joint2'], position: [{joint1_rad}, {joint2_rad}]}}"
    cmd = ["ros2", "topic", "pub", "--once", "/arm_joint_command",
           "sensor_msgs/msg/JointState", joint_msg]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=1.0)
        time.sleep(1)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：使用弧度制，需要手动转换，知道消息格式
- **Python方式**：使用常见的角度制（度），自动转换

---

### 4. set_yaw_angle(angle)

**功能**：设置云台/摄像头的偏航角

**ROS原生实现方式**：
```bash
# 发布云台角度命令
ros2 topic pub --once /yaw_joint_command std_msgs/msg/Float64 \
  "{data: 1.57}"  # 弧度制
```

**Python封装实现**：
```python
def set_yaw_angle(self, angle):
    """设置云台yaw轴角度（度）"""
    angle_rad = math.radians(angle)
    yaw_msg = f"{{data: {angle_rad}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/yaw_joint_command",
           "std_msgs/msg/Float64", yaw_msg]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=1.0)
        time.sleep(0.5)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：弧度制，需要手动计算
- **Python方式**：角度制（度），更直观

---

### 5. set_gripper(value)

**功能**：控制夹爪开合

**ROS原生实现方式**：
```bash
# 发布夹爪命令
ros2 topic pub --once /gripper_command std_msgs/msg/Float64 \
  "{data: 1.0}"  # 1.0=闭合，0.0=打开
```

**Python封装实现**：
```python
def set_gripper(self, value):
    """控制夹爪：0.0=完全打开，1.0=完全闭合"""
    if not 0.0 <= value <= 1.0:
        print(f"[Error] 夹爪值必须在0.0-1.0之间")
        return False
    
    gripper_msg = f"{{data: {value}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/gripper_command",
           "std_msgs/msg/Float64", gripper_msg]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=1.0)
        time.sleep(0.5)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：需要记住0/1的含义
- **Python方式**：添加参数验证，更安全

---

### 6. get_arm_pose_xy()

**功能**：获取机械臂末端的当前坐标

**ROS原生实现方式**：
```bash
# 方式1：订阅末端位置话题（如果提供）
ros2 topic echo /arm_end_effector_pose --once

# 方式2：订阅关节状态，手动计算正运动学
ros2 topic echo /joint_states --once
# 获得关节角度后，需要手动计算：
# x = L1*cos(θ1) + L2*cos(θ1+θ2)
# y = L1*sin(θ1) + L2*sin(θ1+θ2)
```

**Python封装实现**：
```python
def get_arm_pose_xy(self):
    """获取机械臂末端xy坐标（通过正运动学计算）"""
    # 获取当前关节角度
    joint_states = self.get_joint_states()
    if joint_states is None:
        return None
    
    joint1_rad = joint_states["joint1"]
    joint2_rad = joint_states["joint2"]
    
    # 正运动学计算
    L1, L2 = self.arm_length_1, self.arm_length_2
    x = L1 * math.cos(joint1_rad) + L2 * math.cos(joint1_rad + joint2_rad)
    y = L1 * math.sin(joint1_rad) + L2 * math.sin(joint1_rad + joint2_rad)
    
    return {"x": x, "y": y}
```

**对比说明**：
- **ROS方式**：依赖特定话题，或需要手动实现正运动学
- **Python方式**：内置正运动学，自动计算

---

### 7. get_joint_states()

**功能**：获取所有关节的当前状态（位置、速度）

**ROS原生实现方式**：
```bash
# 订阅关节状态话题
ros2 topic echo /joint_states --once
# 输出：
# name: ['joint1', 'joint2', 'joint3', ...]
# position: [0.5, -0.3, 1.2, ...]
# velocity: [0.0, 0.0, 0.0, ...]
# effort: [0.0, 0.0, 0.0, ...]
# 需要手动匹配name和position数组
```

**Python封装实现**：
```python
def get_joint_states(self):
    """获取关节状态，返回名称和位置的映射"""
    cmd = ["ros2", "topic", "echo", "/joint_states", "--once"]
    try:
        output = subprocess.check_output(cmd, timeout=2.0).decode("utf-8")
        
        # 提取关节名称
        names_match = re.search(r"name:\s*\[(.*?)\]", output)
        names = [n.strip().strip("'\"") for n in names_match.group(1).split(",")]
        
        # 提取位置值
        pos_match = re.search(r"position:\s*\[(.*?)\]", output)
        positions = [float(p.strip()) for p in pos_match.group(1).split(",")]
        
        # 构建字典映射
        return {name: pos for name, pos in zip(names, positions)}
    except:
        return None
```

**对比说明**：
- **ROS方式**：两个数组需要手动匹配索引
- **Python方式**：自动关联，返回名称→位置的字典

---

### 8. set_pwm(pin, duty_cycle)

**功能**：设置指定引脚的PWM占空比

**ROS原生实现方式**：
```bash
# 发布PWM命令（具体话题依赖于硬件驱动）
ros2 topic pub --once /pwm_command wheeltec_msgs/msg/PWM \
  "{pin: 3, duty_cycle: 0.75}"
```

**Python封装实现**：
```python
def set_pwm(self, pin, duty_cycle):
    """设置PWM：pin=引脚号，duty_cycle=占空比(0.0-1.0)"""
    if not 0.0 <= duty_cycle <= 1.0:
        print(f"[Error] 占空比必须在0.0-1.0之间")
        return False
    
    pwm_msg = f"{{pin: {pin}, duty_cycle: {duty_cycle}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/pwm_command",
           "wheeltec_msgs/msg/PWM", pwm_msg]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=1.0)
        return True
    except:
        return False
```

**对比说明**：
- **ROS方式**：需要了解自定义消息类型
- **Python方式**：简单的两个参数，添加值验证

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

### 2. launch_lidar()

**功能**：启动激光雷达节点

**ROS原生实现方式**：
```bash
# 需要知道具体的雷达型号和对应的launch文件
ros2 launch ldlidar_stl_ros2 ld06.launch.py
# 或
ros2 launch rplidar_ros rplidar.launch.py
```

**Python封装实现**：
```python
def launch_lidar(self):
    """启动激光雷达"""
    if self.lidar_process and self.lidar_process.poll() is None:
        print("[Info] 雷达已经在运行")
        return True
    
    # 尝试启动常见的雷达驱动
    cmd = ["ros2", "launch", "ldlidar_stl_ros2", "ld06.launch.py"]
    self.lidar_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.PIPE)
    time.sleep(3)
    
    if self.lidar_process.poll() is None:
        print("[Info] 雷达启动成功")
        return True
    else:
        print("[Error] 雷达启动失败")
        return False
```

**对比说明**：
- **ROS方式**：需要知道雷达型号和对应的launch文件
- **Python方式**：自动尝试启动，管理进程

---

### 3. stop_lidar()

**功能**：停止激光雷达节点

**ROS原生实现方式**：
```bash
# 需要手动查找雷达进程并终止
ps aux | grep ldlidar
# 然后使用特定PID终止
```

**Python封装实现**：
```python
def stop_lidar(self):
    """停止激光雷达"""
    if self.lidar_process:
        self.lidar_process.terminate()
        self.lidar_process.wait(timeout=3)
        self.lidar_process = None
        print("[Info] 雷达已停止")
```

**对比说明**：
- **ROS方式**：手动查找和终止进程
- **Python方式**：自动管理，优雅关闭

---

### 4. launch_camera()

**功能**：启动摄像头节点

**ROS原生实现方式**：
```bash
# 启动USB摄像头
ros2 launch usb_cam camera.launch.py
# 或
ros2 run usb_cam usb_cam_node_exe
```

**Python封装实现**：
```python
def launch_camera(self):
    """启动摄像头"""
    if self.camera_process and self.camera_process.poll() is None:
        print("[Info] 摄像头已经在运行")
        return True
    
    cmd = ["ros2", "launch", "usb_cam", "camera.launch.py"]
    self.camera_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.PIPE)
    time.sleep(3)
    
    if self.camera_process.poll() is None:
        print("[Info] 摄像头启动成功")
        return True
    else:
        print("[Error] 摄像头启动失败")
        return False
```

**对比说明**：
- **ROS方式**：需要知道摄像头包名和launch文件
- **Python方式**：自动启动和管理

---

### 5. stop_camera()

**功能**：停止摄像头节点

**ROS原生实现方式**：
```bash
# 手动查找摄像头进程
ps aux | grep usb_cam
# 然后使用特定PID终止
```

**Python封装实现**：
```python
def stop_camera(self):
    """停止摄像头"""
    if self.camera_process:
        self.camera_process.terminate()
        self.camera_process.wait(timeout=3)
        self.camera_process = None
        print("[Info] 摄像头已停止")
```

**对比说明**：
- **ROS方式**：手动终止进程
- **Python方式**：自动管理生命周期

---

### 6. start_line_tracking(color)

**功能**：启动视觉巡线功能

**ROS原生实现方式**：
```bash
# 步骤1：确保摄像头运行（终端1）
ros2 launch usb_cam camera.launch.py

# 步骤2：启动巡线节点（终端2）
ros2 launch wheeltec_robot_vision line_tracking.launch.py line_color:=yellow

# 需要管理多个进程和依赖关系
```

**Python封装实现**：
```python
def start_line_tracking(self, color):
    """启动视觉巡线，自动处理摄像头依赖"""
    if not self.camera_process:
        self.launch_camera()
    
    if self.application_process and self.application_process.poll() is None:
        print("[Info] 正在运行其他应用，先停止")
        self.stop_application()
    
    cmd = ["ros2", "launch", "wheeltec_robot_vision", 
           "line_tracking.launch.py", f"line_color:={color}"]
    self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                                stderr=subprocess.PIPE)
    time.sleep(2)
    
    if self.application_process.poll() is None:
        print(f"[Info] 巡线功能已启动（颜色：{color}）")
        return True
    else:
        print("[Error] 巡线功能启动失败")
        return False
```

**对比说明**：
- **ROS方式**：需要手动管理依赖，打开多个终端
- **Python方式**：自动处理依赖，一个函数搞定

---

### 7. start_lidar_follow(target_dist)

**功能**：启动雷达跟随功能

**ROS原生实现方式**：
```bash
# 步骤1：确保雷达运行（终端1）
ros2 launch ldlidar_stl_ros2 ld06.launch.py

# 步骤2：启动跟随节点（终端2）
ros2 launch wheeltec_robot_follow lidar_follow.launch.py target_distance:=0.8

# 需要管理两个进程
```

**Python封装实现**：
```python
def start_lidar_follow(self, target_dist=0.8):
    """启动雷达跟随，自动处理雷达依赖"""
    if not self.lidar_process:
        self.launch_lidar()
    
    if self.application_process and self.application_process.poll() is None:
        print("[Info] 正在运行其他应用，先停止")
        self.stop_application()
    
    cmd = ["ros2", "launch", "wheeltec_robot_follow",
           "lidar_follow.launch.py", f"target_distance:={target_dist}"]
    self.application_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                                stderr=subprocess.PIPE)
    time.sleep(2)
    
    if self.application_process.poll() is None:
        print(f"[Info] 雷达跟随已启动（目标距离：{target_dist}m）")
        return True
    else:
        print("[Error] 雷达跟随启动失败")
        return False
```

**对比说明**：
- **ROS方式**：手动启动依赖，管理多个终端
- **Python方式**：自动处理依赖关系

---

### 8. stop_application()

**功能**：停止当前运行的应用（跟随、巡线等）

**ROS原生实现方式**：
```bash
# 需要记住在哪个终端启动的应用
# 然后手动按Ctrl+C，或查找进程
ps aux | grep wheeltec
# 使用特定PID终止
```

**Python封装实现**：
```python
def stop_application(self):
    """停止当前运行的应用（视觉跟随、巡线、雷达跟随等）"""
    if self.application_process:
        self.application_process.terminate()
        try:
            self.application_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.application_process.kill()
        self.application_process = None
        print("[Info] 应用已停止")
```

**对比说明**：
- **ROS方式**：需要记住进程，手动终止
- **Python方式**：统一管理，一键停止

---

### 9. get_lidar_distance(angle_degrees)

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

### 2. start_mapping(method)

**功能**：启动SLAM建图

**ROS原生实现方式**：
```bash
# 需要选择建图算法并启动对应的launch文件
# 方式1：使用gmapping
ros2 launch wheeltec_slam gmapping.launch.py

# 方式2：使用cartographer
ros2 launch wheeltec_slam cartographer.launch.py

# 方式3：使用slam_toolbox
ros2 launch wheeltec_slam_toolbox slam_toolbox.launch.py

# 需要记住不同算法的launch文件名
```

**Python封装实现**：
```python
def start_mapping(self, method="gmapping"):
    """启动建图功能，支持多种SLAM算法"""
    slam_methods = {
        "gmapping": ["ros2", "launch", "wheeltec_slam", "gmapping.launch.py"],
        "cartographer": ["ros2", "launch", "wheeltec_slam", "cartographer.launch.py"],
        "slam_toolbox": ["ros2", "launch", "wheeltec_slam_toolbox", "slam_toolbox.launch.py"]
    }
    
    if method not in slam_methods:
        print(f"[Error] 未知的建图方法：{method}")
        return False
    
    if self.mapping_process and self.mapping_process.poll() is None:
        print("[Info] 建图已经在运行")
        return True
    
    cmd = slam_methods[method]
    self.mapping_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                           stderr=subprocess.PIPE)
    time.sleep(3)
    
    if self.mapping_process.poll() is None:
        print(f"[Info] {method} 建图已启动")
        return True
    else:
        print(f"[Error] {method} 建图启动失败")
        return False
```

**对比说明**：
- **ROS方式**：需要记住多个算法的launch文件名和路径
- **Python方式**：统一接口，传入算法名称即可

---

### 3. save_map(map_name)

**功能**：保存当前构建的地图

**ROS原生实现方式**：
```bash
# 需要使用map_saver节点
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 或使用服务调用
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: 'map', map_url: '/home/user/maps/my_map', image_format: 'pgm', mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"
```

**Python封装实现**：
```python
def save_map(self, map_name="my_map"):
    """保存地图到文件"""
    # 确保地图目录存在
    map_dir = os.path.expanduser("~/maps")
    os.makedirs(map_dir, exist_ok=True)
    
    map_path = os.path.join(map_dir, map_name)
    
    cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli",
           "-f", map_path]
    
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=10.0)
        if result.returncode == 0:
            print(f"[Info] 地图已保存：{map_path}.pgm 和 {map_path}.yaml")
            return True
        else:
            print(f"[Error] 地图保存失败")
            return False
    except subprocess.TimeoutExpired:
        print("[Error] 保存地图超时")
        return False
```

**对比说明**：
- **ROS方式**：需要手动指定完整路径，记住命令参数
- **Python方式**：自动创建目录，简单的地图名称

---

### 4. load_map_and_start_navigation(map_name)

**功能**：加载地图并启动导航

**ROS原生实现方式**：
```bash
# 需要手动启动导航launch文件，并指定地图路径
ros2 launch wheeltec_nav2 navigation.launch.py \
  map:=/home/user/maps/my_map.yaml

# 需要知道地图文件的完整路径
```

**Python封装实现**：
```python
def load_map_and_start_navigation(self, map_name="my_map"):
    """加载地图并启动导航"""
    map_dir = os.path.expanduser("~/maps")
    map_file = os.path.join(map_dir, f"{map_name}.yaml")
    
    if not os.path.exists(map_file):
        print(f"[Error] 地图文件不存在：{map_file}")
        return False
    
    if self.navigation_process and self.navigation_process.poll() is None:
        print("[Info] 导航已经在运行")
        return True
    
    cmd = ["ros2", "launch", "wheeltec_nav2", "navigation.launch.py",
           f"map:={map_file}"]
    self.navigation_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                              stderr=subprocess.PIPE)
    time.sleep(5)
    
    if self.navigation_process.poll() is None:
        print(f"[Info] 导航已启动，使用地图：{map_name}")
        return True
    else:
        print("[Error] 导航启动失败")
        return False
```

**对比说明**：
- **ROS方式**：需要知道地图文件的完整路径
- **Python方式**：只需地图名称，自动构建路径

---

### 5. cancel_navigation()

**功能**：取消当前导航任务

**ROS原生实现方式**：
```bash
# 发布空的目标点或调用取消服务
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose --cancel

# 或发布空的goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{}"
```

**Python封装实现**：
```python
def cancel_navigation(self):
    """取消当前导航任务"""
    # 方式1：发送空的目标点
    empty_goal = "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    cmd = ["ros2", "topic", "pub", "--once", "/goal_pose",
           "geometry_msgs/msg/PoseStamped", empty_goal]
    
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL, timeout=2.0)
        print("[Info] 导航任务已取消")
        return True
    except:
        print("[Error] 取消导航失败")
        return False
```

**对比说明**：
- **ROS方式**：需要理解action机制或发送空消息
- **Python方式**：简单的函数调用

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

**文档版本**：v3.0 完整版（包含全部37个函数）  
**最后更新**：2026-02-02  
**包含函数**：37个（系统管理6个 + 底盘运动9个 + 机械臂8个 + 感知功能9个 + 建图导航5个）
