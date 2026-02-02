# ROS2功能Python封装详细说明

## 目录
1. [项目概述](#项目概述)
2. [封装原理](#封装原理)
3. [文件结构](#文件结构)
4. [函数详解](#函数详解)
5. [使用示例](#使用示例)
6. [常见问题](#常见问题)

---

## 项目概述

本项目将ROS2机器人的功能封装成简单的Python接口，让用户无需学习ROS2即可控制机器人。

### 设计目标
- **简化操作**：将复杂的 `ros2 launch` 和 `ros2 run` 命令封装为简单的Python函数调用
- **统一接口**：提供统一的API接口，支持不同型号的机器人（阿克曼、差速、麦轮）
- **易于使用**：用户只需调用Python函数，无需了解ROS2的话题、服务、launch文件等概念

### 封装前后对比

**封装前（ROS2命令）：**
```bash
# 启动机器人
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 启动雷达
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py

# 启动建图
ros2 launch wheeltec_slam_toolbox online_async_launch.py

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map
```

**封装后（Python代码）：**
```python
from robot_lib import Robot

robot = Robot()
robot.initialize("mec")        # 启动机器人
robot.launch_lidar()            # 启动雷达
robot.start_mapping("slam_toolbox")  # 启动建图
robot.save_map("my_map")        # 保存地图
robot.shutdown()                # 关闭系统
```

---

## 封装原理

### 1. 核心思想

ROS2功能的Python封装主要基于以下几个原理：

#### 1.1 进程管理
使用Python的 `subprocess` 模块启动和管理ROS2进程：

```python
import subprocess

# 启动ROS2 launch文件
cmd = ["ros2", "launch", "package_name", "launch_file.py"]
process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
```

**关键点：**
- `Popen`：以非阻塞方式启动进程，允许后台运行
- `stdout=subprocess.DEVNULL`：屏蔽标准输出，保持界面清爽
- `stderr=subprocess.PIPE`：保留错误输出，便于调试
- 进程句柄保存在对象属性中，便于后续管理

#### 1.2 话题通信
通过 `ros2 topic` 命令与ROS2话题交互：

```python
# 发布速度指令
cmd = ["ros2", "topic", "pub", "--once", "/cmd_vel", 
       "geometry_msgs/msg/Twist", 
       '{"linear": {"x": 0.5}, "angular": {"z": 0.0}}']
subprocess.run(cmd)

# 订阅话题获取数据
cmd = ["ros2", "topic", "echo", "/odom", "--once"]
output = subprocess.check_output(cmd, timeout=2.0)
```

**关键点：**
- `pub --once`：发布一次消息后退出
- `echo --once`：接收一次消息后退出
- `--field`：仅获取消息的特定字段，简化解析
- `timeout`：设置超时防止程序卡死

#### 1.3 服务调用
通过 `ros2 service` 命令调用ROS2服务：

```python
cmd = ["ros2", "service", "call", "/service_name", 
       "service_type", "{}"]
subprocess.run(cmd, timeout=5.0)
```

### 2. 架构设计

#### 2.1 单一类设计
所有功能封装在一个 `Robot` 类中：

```python
class Robot:
    def __init__(self):
        # 进程句柄
        self.driver_process = None
        self.lidar_process = None
        self.camera_process = None
        # ... 其他进程
        
        # 配置参数
        self.ROBOT_TYPE_MAP = {...}
        self.current_robot_type = None
```

**优点：**
- 统一管理所有功能
- 便于资源清理（shutdown时关闭所有进程）
- 状态信息集中存储

#### 2.2 功能模块划分

```
Robot类
├── 系统管理
│   ├── initialize()      # 初始化
│   ├── shutdown()        # 关闭
│   ├── emergency_stop()  # 急停
│   └── get_battery_voltage()  # 电压
│
├── 底盘控制
│   ├── set_velocity()    # 设置速度
│   ├── move_distance()   # 移动距离
│   ├── rotate_angle()    # 旋转角度
│   └── get_robot_pose()  # 获取位姿
│
├── 机械臂控制
│   ├── arm_home()        # 复位
│   ├── set_joint_angles()  # 设置关节角度
│   ├── set_gripper()     # 控制夹爪
│   └── get_arm_pose_xy()  # 获取位置
│
├── 传感器控制
│   ├── launch_lidar()    # 启动雷达
│   ├── launch_camera()   # 启动相机
│   └── get_lidar_distance()  # 获取距离
│
└── 建图与导航
    ├── start_mapping()   # 启动建图
    ├── save_map()        # 保存地图
    ├── start_navigation()  # 启动导航
    └── move_to_goal()    # 移动到目标点
```

### 3. 关键技术实现

#### 3.1 车型兼容性处理

```python
self.ROBOT_TYPE_MAP = {
    "akm": "ackermann",    # 阿克曼
    "diff": "diff",        # 差速
    "mec": "mecanum"       # 麦轮
}

def set_velocity(self, v_x, v_y, w_z):
    # 自动处理不同车型
    # 阿克曼/差速: v_y 自动设为0
    # 麦轮: 支持全向移动
```

#### 3.2 闭环控制实现

以 `move_distance()` 为例：

```python
def move_distance(self, distance, speed):
    # 1. 获取初始位置
    start_pos = self._get_odom_position()
    
    # 2. 开始移动
    while moved_distance < target_distance:
        self.set_velocity(v_x, 0, 0)  # 发送速度
        time.sleep(0.1)                # 控制周期
        
        # 3. 获取当前位置，计算已移动距离
        current_pos = self._get_odom_position()
        moved_distance = math.sqrt(...)
    
    # 4. 停止
    self.set_velocity(0, 0, 0)
```

**关键点：**
- 使用里程计 `/odom` 话题反馈位置
- 循环控制，实时计算移动距离
- 到达目标后自动停止

#### 3.3 数据解析

从ROS2话题获取数据后需要解析：

```python
def _get_odom_position(self):
    cmd = ["ros2", "topic", "echo", "/odom", "--once", 
           "--field", "pose.pose.position"]
    output = subprocess.check_output(cmd, timeout=1.0)
    
    # 解析输出
    lines = output.decode("utf-8").strip().split('\n')
    for line in lines:
        if 'x:' in line:
            x = float(line.split(':')[1].strip())
        elif 'y:' in line:
            y = float(line.split(':')[1].strip())
    
    return (x, y)
```

#### 3.4 四元数转欧拉角

在获取机器人朝向时需要转换：

```python
def _get_odom_yaw(self):
    # 获取四元数 (qx, qy, qz, qw)
    # ...
    
    # 转换为欧拉角yaw
    yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    return yaw
```

### 4. 错误处理

#### 4.1 超时保护
```python
try:
    output = subprocess.check_output(cmd, timeout=2.0)
except subprocess.TimeoutExpired:
    print("[Warning] 操作超时")
    return None
```

#### 4.2 进程状态检查
```python
# 启动后检查进程是否正常运行
self.driver_process = subprocess.Popen(cmd, ...)
time.sleep(5)  # 等待初始化

if self.driver_process.poll() is not None:
    print("[Error] 进程启动失败")
    return False
```

#### 4.3 安全关闭
```python
def shutdown(self):
    if self.driver_process:
        # 先发送SIGINT（优雅关闭）
        self.driver_process.send_signal(signal.SIGINT)
        try:
            self.driver_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            # 超时则强制杀进程
            self.driver_process.kill()
```

---

## 文件结构

```
Python-encapsulation-of-ROS-functions/
├── robot_lib.py              # 核心库文件（所有功能封装）
├── robot_app.py              # 原始示例（保持不变）
├── chassis_control.py        # 底盘控制应用示例
├── arm_control.py            # 机械臂控制应用示例
├── sensor_app.py             # 传感器应用示例
├── mapping_app.py            # 建图与导航应用示例
├── ENCAPSULATION_GUIDE.md    # 本文档
├── README_CN.md              # 中文使用说明
└── src/                      # ROS2源代码
    ├── turn_on_wheeltec_robot/
    ├── simple_follower_ros2/
    ├── wheeltec_robot_slam/
    └── ...
```

### 文件说明

#### robot_lib.py
- **作用**：核心库，包含所有封装的ROS2功能
- **内容**：Robot类及所有方法
- **修改**：在原有基础上扩展，原有代码和注释保持不变

#### 应用示例文件
每个应用文件演示一类功能的使用：

1. **chassis_control.py**：底盘运动控制
   - 速度控制
   - 距离移动
   - 旋转控制
   - 位姿获取

2. **sensor_app.py**：传感器和应用功能
   - 雷达控制
   - 相机控制
   - 视觉跟随
   - 视觉巡线
   - 雷达跟随

3. **mapping_app.py**：建图与导航
   - SLAM建图
   - 地图保存
   - 自主导航
   - 目标点导航

4. **arm_control.py**：机械臂控制
   - 关节控制
   - 位置控制
   - 夹爪控制
   - 抓取放置

---

## 函数详解

### 系统管理函数

#### initialize(robot_type)
初始化机器人系统。

**参数：**
- `robot_type`: str - 车型，可选 "akm"(阿克曼)、"diff"(差速)、"mec"(麦轮)

**返回：**
- bool - 成功返回True，失败返回False

**实现原理：**
```python
# 1. 构建launch命令
cmd = ["ros2", "launch", "turn_on_wheeltec_robot", 
       "turn_on_wheeltec_robot.launch.py", 
       f"robot_type:={robot_type}"]

# 2. 启动后台进程
self.driver_process = subprocess.Popen(cmd, ...)

# 3. 等待硬件初始化
time.sleep(5)
```

**调用的ROS2命令：**
```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py robot_type:=mecanum
```

---

#### shutdown()
安全关闭所有子进程。

**实现原理：**
```python
# 依次关闭所有启动的进程
for process in [driver, lidar, camera, ...]:
    if process:
        process.send_signal(signal.SIGINT)  # 优雅关闭
        process.wait(timeout=3)
        # 超时则强制kill
```

---

#### emergency_stop()
软件急停，立即发送零速度指令。

**实现原理：**
```python
# 发布零速度到/cmd_vel话题
cmd = ["ros2", "topic", "pub", "--once", "/cmd_vel",
       "geometry_msgs/msg/Twist",
       '{"linear": {"x": 0}, "angular": {"z": 0}}']
```

---

#### get_battery_voltage()
获取电池电压。

**返回：**
- float - 电压值（伏特）

**实现原理：**
```python
# 从/PowerVoltage话题获取电压
cmd = ["ros2", "topic", "echo", "/PowerVoltage", 
       "--once", "--field", "data"]
output = subprocess.check_output(cmd, timeout=2.0)
voltage = float(output.strip())
```

---

### 底盘控制函数

#### set_velocity(v_x, v_y, w_z)
设置机器人速度（最底层控制）。

**参数：**
- `v_x`: float - 前进线速度 (m/s)，正值前进
- `v_y`: float - 横向速度 (m/s)，仅麦轮有效
- `w_z`: float - 角速度 (rad/s)，正值逆时针

**实现原理：**
```python
# 构建Twist消息
twist_msg = {
    "linear": {"x": v_x, "y": v_y, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": w_z}
}

# 发布到/cmd_vel话题
cmd = ["ros2", "topic", "pub", "--once", "/cmd_vel",
       "geometry_msgs/msg/Twist", json.dumps(twist_msg)]
```

**车型兼容：**
- 阿克曼/差速：只使用v_x和w_z
- 麦轮：可使用v_x、v_y、w_z实现全向移动

---

#### move_distance(distance, speed, lateral_distance, lateral_speed)
闭环控制移动指定距离。

**参数：**
- `distance`: float - 前向距离(米)，负值后退
- `speed`: float - 前向速度(m/s)
- `lateral_distance`: float - 横向距离(米)，仅麦轮
- `lateral_speed`: float - 横向速度(m/s)，仅麦轮

**实现原理：**
1. 获取初始位置（从/odom话题）
2. 循环发送速度指令
3. 实时获取当前位置
4. 计算已移动距离
5. 到达目标停止

**核心代码：**
```python
start_pos = self._get_odom_position()
moved_distance = 0.0

while moved_distance < target_distance:
    self.set_velocity(v_x, v_y, 0)
    time.sleep(0.1)
    
    current_pos = self._get_odom_position()
    moved_distance = math.sqrt(
        (current_x - start_x)**2 + 
        (current_y - start_y)**2
    )

self.set_velocity(0, 0, 0)  # 停止
```

---

#### rotate_angle(angle, speed)
原地旋转指定角度。

**参数：**
- `angle`: float - 角度(度)，正值逆时针
- `speed`: float - 角速度(rad/s)

**实现原理：**
1. 获取初始朝向（从/odom话题）
2. 循环发送旋转指令
3. 实时获取当前朝向
4. 计算累计旋转量（处理角度跳变）
5. 到达目标停止

**处理角度跳变：**
```python
# 处理±180度跳变
delta_yaw = current_yaw - last_yaw
if delta_yaw > math.pi:
    delta_yaw -= 2 * math.pi
elif delta_yaw < -math.pi:
    delta_yaw += 2 * math.pi

rotated_angle += abs(delta_yaw)
```

---

### 机械臂控制函数

#### arm_home()
机械臂复位到初始姿态。

**实现原理：**
```python
# 调用机械臂复位服务
cmd = ["ros2", "service", "call", "/arm_home",
       "std_srvs/srv/Trigger", "{}"]
```

**注意：**
- 需要机械臂ROS驱动节点支持
- 不同机械臂的服务名可能不同

---

#### set_joint_angles(joint1, joint2)
设置机械臂关节角度。

**参数：**
- `joint1`: float - 大臂角度(度)
- `joint2`: float - 小臂角度(度)

**实现原理：**
- 发布关节角度到机械臂控制话题
- 需要根据实际机械臂调整话题名称

---

#### set_gripper(value)
控制夹爪开合。

**参数：**
- `value`: int - 开合程度 0-10（0=闭合，10=张开）

---

### 传感器控制函数

#### launch_lidar()
启动雷达驱动。

**实现原理：**
```python
cmd = ["ros2", "launch", "turn_on_wheeltec_robot", 
       "wheeltec_lidar.launch.py"]
self.lidar_process = subprocess.Popen(cmd, ...)
time.sleep(3)  # 等待初始化
```

**效果：**
- 雷达数据发布在 `/scan` 话题
- 可在RViz中可视化

---

#### launch_camera()
启动相机驱动。

**实现原理：**
```python
cmd = ["ros2", "launch", "turn_on_wheeltec_robot",
       "wheeltec_camera.launch.py"]
self.camera_process = subprocess.Popen(cmd, ...)
```

**效果：**
- 图像数据发布在 `/camera/image_raw` 话题

---

#### start_visual_follow(color)
启动视觉跟随。

**参数：**
- `color`: str - 目标颜色 "red", "blue", "green", "yellow"

**实现原理：**
```python
cmd = ["ros2", "launch", "simple_follower_ros2",
       "visual_follower.launch.py",
       f"target_color:={color}"]
self.app_process = subprocess.Popen(cmd, ...)
```

**功能：**
- 识别指定颜色的物体
- 自动调整速度保持物体在视野中心

---

#### start_line_tracking(color)
启动视觉巡线。

**参数：**
- `color`: str - 线条颜色 "black", "red", "yellow"

**功能：**
- 识别地面线条
- 自动沿线行驶

---

#### start_lidar_follow(target_dist)
启动雷达跟随。

**参数：**
- `target_dist`: float - 跟随距离(米)

**功能：**
- 跟随前方最近的移动物体（人）
- 保持指定距离

---

### 建图与导航函数

#### start_mapping(method)
启动SLAM建图。

**参数：**
- `method`: str - 建图算法
  - "gmapping": 2D网格建图
  - "cartographer": Google Cartographer
  - "slam_toolbox": 推荐算法

**实现原理：**
```python
launch_map = {
    'gmapping': ['ros2', 'launch', 'slam_gmapping', 
                 'slam_gmapping.launch.py'],
    'cartographer': ['ros2', 'launch', 'wheeltec_cartographer',
                     'cartographer.launch.py'],
    'slam_toolbox': ['ros2', 'launch', 'wheeltec_slam_toolbox',
                     'online_async_launch.py']
}

cmd = launch_map[method]
self.mapping_process = subprocess.Popen(cmd, ...)
```

**使用方法：**
1. 启动建图
2. 移动机器人（键盘或自动控制）
3. 保存地图

---

#### save_map(map_name)
保存当前地图。

**参数：**
- `map_name`: str - 地图文件名（不含扩展名）

**实现原理：**
```python
cmd = ["ros2", "run", "nav2_map_server", 
       "map_saver_cli", "-f", map_name]
subprocess.run(cmd, timeout=10.0)
```

**生成文件：**
- `map_name.pgm`: 地图图片
- `map_name.yaml`: 地图元数据

---

#### start_navigation(map_file)
启动导航功能。

**参数：**
- `map_file`: str - 地图文件路径(.yaml)

**实现原理：**
```python
cmd = ["ros2", "launch", "wheeltec_nav2", 
       "wheeltec_nav2.launch.py"]
if map_file:
    cmd.extend([f"map:={map_file}"])
self.nav_process = subprocess.Popen(cmd, ...)
```

---

#### move_to_goal(x, y, theta)
发送导航目标点。

**参数：**
- `x`: float - 目标X坐标(米)
- `y`: float - 目标Y坐标(米)
- `theta`: float - 目标朝向(度)

**实现原理：**
```python
# 角度转四元数
theta_rad = math.radians(theta)
qz = math.sin(theta_rad / 2.0)
qw = math.cos(theta_rad / 2.0)

# 构建目标点消息
goal_msg = {
    "pose": {
        "position": {"x": x, "y": y, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
    }
}

# 发布到/goal_pose话题
cmd = ["ros2", "topic", "pub", "--once", "/goal_pose",
       "geometry_msgs/msg/PoseStamped", json.dumps(goal_msg)]
```

---

## 使用示例

### 示例1：基础底盘控制

```python
#!/usr/bin/env python3
from robot_lib import Robot

robot = Robot()

# 初始化
robot.initialize("mec")  # 麦轮车型

# 前进1米
robot.move_distance(1.0, 0.3)

# 旋转90度
robot.rotate_angle(90)

# 关闭
robot.shutdown()
```

### 示例2：雷达跟随

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

# 初始化
robot.initialize("diff")  # 差速车型

# 启动雷达
robot.launch_lidar()
time.sleep(3)

# 启动雷达跟随（保持0.8米距离）
robot.start_lidar_follow(0.8)

# 运行30秒
time.sleep(30)

# 停止跟随
robot.stop_application()

# 关闭
robot.shutdown()
```

### 示例3：自动建图

```python
#!/usr/bin/env python3
from robot_lib import Robot

robot = Robot()

# 初始化
robot.initialize("mec")

# 启动雷达
robot.launch_lidar()

# 启动建图
robot.start_mapping("slam_toolbox")

# 自动移动建图（简单方形路径）
for i in range(4):
    robot.move_distance(2.0, 0.3)  # 前进2米
    robot.rotate_angle(90)          # 转90度

# 保存地图
robot.save_map("my_room")

# 关闭
robot.shutdown()
```

### 示例4：导航到目标点

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

# 初始化
robot.initialize("mec")

# 启动雷达
robot.launch_lidar()

# 启动导航（使用之前保存的地图）
robot.start_navigation("my_room.yaml")

# 导航到多个目标点
goals = [
    (2.0, 0.0, 0),      # 点1
    (2.0, 2.0, 90),     # 点2
    (0.0, 2.0, 180),    # 点3
    (0.0, 0.0, -90)     # 回起点
]

for x, y, theta in goals:
    robot.move_to_goal(x, y, theta)
    time.sleep(15)  # 等待到达

# 关闭
robot.shutdown()
```

### 示例5：视觉巡线

```python
#!/usr/bin/env python3
from robot_lib import Robot
import time

robot = Robot()

# 初始化
robot.initialize("diff")

# 启动相机
robot.launch_camera()
time.sleep(3)

# 启动视觉巡线（黑线）
robot.start_line_tracking("black")

# 巡线30秒
time.sleep(30)

# 停止巡线
robot.stop_application()

# 关闭
robot.shutdown()
```

---

## 常见问题

### Q1: 为什么初始化后要等待5秒？

**A:** 机器人硬件（串口、雷达、IMU等）需要初始化时间。如果不等待直接发送指令，硬件可能还未就绪，导致指令失效。

```python
self.driver_process = subprocess.Popen(cmd, ...)
time.sleep(5)  # 等待硬件初始化
```

### Q2: 如何判断进程是否成功启动？

**A:** 使用 `poll()` 方法检查进程状态：

```python
if self.driver_process.poll() is not None:
    # 进程已退出，说明启动失败
    print("启动失败")
    return False
```

### Q3: move_distance() 为什么可能失败？

**A:** 此功能依赖里程计反馈，如果：
- 里程计未正常发布数据
- `/odom` 话题不存在
- 网络延迟太大

会导致功能失败。可以先测试：
```bash
ros2 topic echo /odom --once
```

### Q4: 不同车型有什么区别？

**A:** 

| 车型 | 代码 | 运动能力 | 控制方式 |
|------|------|----------|----------|
| 阿克曼 | akm | 前后移动 + 转向 | v_x, w_z |
| 差速 | diff | 前后移动 + 原地旋转 | v_x, w_z |
| 麦轮 | mec | 全向移动 | v_x, v_y, w_z |

### Q5: 如何修改适配自己的机器人？

**A:** 需要修改以下部分：

1. **话题名称**：根据实际话题修改
```python
topic_name = "/PowerVoltage"  # 改为实际电压话题
```

2. **launch文件**：根据实际包名修改
```python
cmd = ["ros2", "launch", "your_package", "your_launch.py"]
```

3. **消息类型**：根据实际消息类型修改
```python
"geometry_msgs/msg/Twist"  # 改为实际消息类型
```

### Q6: 如何添加新功能？

**A:** 按照以下步骤：

1. 在 `Robot` 类中添加方法
2. 使用 `subprocess` 调用ROS2命令
3. 添加错误处理和超时保护
4. 在应用示例中演示使用

示例：
```python
def my_new_function(self, param):
    """
    新功能说明
    :param param: 参数说明
    """
    try:
        cmd = ["ros2", "..."]
        subprocess.run(cmd, timeout=5.0)
        return True
    except Exception as e:
        print(f"[Error] {e}")
        return False
```

### Q7: 为什么有些功能显示"需要支持"？

**A:** 某些功能需要：
- 特定硬件（如机械臂）
- 特定ROS节点
- 修改串口协议

这些功能预留了接口，但需要根据实际硬件实现。

### Q8: 如何调试ROS2命令？

**A:** 可以先在终端测试命令：

```bash
# 测试话题发布
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# 测试话题订阅
ros2 topic echo /odom --once

# 查看可用话题
ros2 topic list

# 查看话题类型
ros2 topic info /cmd_vel
```

### Q9: 程序异常退出怎么办？

**A:** 确保使用 `try-finally` 结构：

```python
try:
    # 你的代码
    robot.initialize("mec")
    # ...
finally:
    # 总是执行清理
    robot.shutdown()
```

### Q10: 如何查看ROS2进程是否在运行？

**A:** 使用以下命令：

```bash
# 查看所有ROS2节点
ros2 node list

# 查看进程
ps aux | grep ros2

# 查看话题
ros2 topic list
```

---

## 进阶技巧

### 1. 多线程控制

如果需要同时执行多个任务（如边移动边建图），可以使用线程：

```python
import threading

def mapping_thread(robot):
    robot.start_mapping("slam_toolbox")

# 启动建图线程
thread = threading.Thread(target=mapping_thread, args=(robot,))
thread.start()

# 主线程控制移动
robot.move_distance(1.0, 0.3)
robot.rotate_angle(90)

# 等待建图完成
thread.join()
```

### 2. 回调函数

可以添加回调机制获取实时状态：

```python
def odom_callback(x, y, yaw):
    print(f"位置: {x:.2f}, {y:.2f}, 朝向: {yaw:.2f}")

# 在move_distance中调用回调
def move_distance(self, distance, speed, callback=None):
    while moving:
        pos = self._get_odom_position()
        if callback:
            callback(*pos, yaw)
        # ...
```

### 3. 配置文件

可以使用配置文件管理参数：

```python
import json

# 加载配置
with open('robot_config.json', 'r') as f:
    config = json.load(f)

robot.initialize(config['robot_type'])
```

---

## 总结

本封装项目通过Python的 `subprocess` 模块实现了对ROS2功能的简化封装，主要特点：

1. **简单易用**：无需学习ROS2，只需调用Python函数
2. **统一接口**：支持多种车型和功能
3. **易于扩展**：可根据需求添加新功能
4. **完整示例**：提供多个实用示例程序

希望这份文档能帮助你理解封装原理和使用方法。如有问题，请参考示例代码或查阅ROS2文档。
