# ROS启动问题快速解决指南

## 问题：start.sh 不存在

### 错误信息
```bash
cd ~/wheeltec_ros2
./start.sh
# 错误：-bash: ./start.sh: No such file or directory
```

---

## 快速解决方案

### 方案1：使用标准ROS2启动命令（推荐）

```bash
# 第1步：source ROS2环境
source /opt/ros/humble/setup.bash

# 第2步：source工作空间
source ~/wheeltec_ros2/install/setup.bash

# 第3步：启动底盘驱动
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 方案2：一行命令

```bash
bash -c "source /opt/ros/humble/setup.bash && source ~/wheeltec_ros2/install/setup.bash && ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"
```

### 方案3：创建自己的启动脚本

```bash
# 创建脚本
cat > ~/start_robot.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
EOF

# 添加执行权限
chmod +x ~/start_robot.sh

# 使用
~/start_robot.sh
```

---

## 找不到工作空间？

### 查找ROS工作空间

```bash
# 方法1：搜索目录
find ~ -maxdepth 2 -type d -name "*ros2*" 2>/dev/null

# 方法2：搜索launch文件
find ~ -name "turn_on_wheeltec_robot.launch.py" 2>/dev/null

# 方法3：检查常见路径
ls -la ~/wheeltec_ros2/
ls -la ~/ros2_ws/
ls -la ~/wheeltec_ws/
```

### 常见工作空间路径
- `~/wheeltec_ros2/`
- `~/ros2_ws/`
- `~/wheeltec_ws/`

---

## 验证ROS系统启动成功

```bash
# 在新终端执行（不要关闭启动ROS的终端）

# 1. 检查节点
ros2 node list

# 2. 检查话题
ros2 topic list

# 3. 测试数据
ros2 topic echo /odom --once
```

**应该看到**：
- 节点列表包含 `/wheeltec_robot` 或类似名称
- 话题列表包含 `/cmd_vel`, `/odom`, `/imu/data_raw` 等
- 能够接收到里程计数据

---

## 仍然有问题？

### A. ROS2环境问题

```bash
# 检查ROS2是否安装
ros2 --version

# 如果命令不存在
source /opt/ros/humble/setup.bash
ros2 --version
```

### B. 工作空间未构建

```bash
cd ~/wheeltec_ros2  # 或实际路径
colcon build --symlink-install
source install/setup.bash
```

### C. 串口权限问题

```bash
# 添加串口权限
sudo usermod -a -G dialout $USER

# 检查串口设备
ls -l /dev/ttyUSB* /dev/ttyACM*

# 重新登录以生效
```

### D. 手动启动最小系统

```bash
# 只启动底盘驱动节点
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
ros2 run turn_on_wheeltec_robot wheeltec_robot
```

---

## 完整的启动脚本模板

将以下内容保存为 `~/start_robot.sh`：

```bash
#!/bin/bash
# Wheeltec机器人完整启动脚本

echo "========================================="
echo "  Wheeltec 机器人系统启动"
echo "========================================="

# 1. Source ROS2环境
echo "[1/4] 加载ROS2环境..."
source /opt/ros/humble/setup.bash
if [ $? -ne 0 ]; then
    echo "❌ 错误：无法source ROS2环境"
    echo "   请确认ROS2已正确安装"
    exit 1
fi
echo "✅ ROS2环境加载成功"

# 2. 查找并source工作空间
echo "[2/4] 查找ROS工作空间..."
if [ -f ~/wheeltec_ros2/install/setup.bash ]; then
    WORKSPACE=~/wheeltec_ros2
    source ~/wheeltec_ros2/install/setup.bash
    echo "✅ 找到工作空间: ~/wheeltec_ros2"
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    WORKSPACE=~/ros2_ws
    source ~/ros2_ws/install/setup.bash
    echo "✅ 找到工作空间: ~/ros2_ws"
elif [ -f ~/wheeltec_ws/install/setup.bash ]; then
    WORKSPACE=~/wheeltec_ws
    source ~/wheeltec_ws/install/setup.bash
    echo "✅ 找到工作空间: ~/wheeltec_ws"
else
    echo "❌ 错误：找不到ROS工作空间"
    echo "   请检查以下路径："
    echo "   - ~/wheeltec_ros2/install/setup.bash"
    echo "   - ~/ros2_ws/install/setup.bash"
    echo "   - ~/wheeltec_ws/install/setup.bash"
    exit 1
fi

# 3. 检查串口设备
echo "[3/4] 检查串口设备..."
if ls /dev/ttyUSB* 1> /dev/null 2>&1 || ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "✅ 找到串口设备"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -3
else
    echo "⚠️  警告：未找到串口设备"
    echo "   请检查："
    echo "   1. 底盘连接是否正常"
    echo "   2. 是否需要添加串口权限："
    echo "      sudo usermod -a -G dialout $USER"
fi

# 4. 启动底盘驱动
echo "[4/4] 启动底盘驱动..."
echo "========================================="
echo ""
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

使用方法：
```bash
chmod +x ~/start_robot.sh
~/start_robot.sh
```

---

## 后续步骤

ROS系统启动成功后，可以：

1. **运行Python测试脚本**：
   ```bash
   cd ~/wheeltec_robot_python
   python3 test_basic.py
   ```

2. **测试速度控制**：
   ```bash
   python3 test_motion.py
   ```

3. **综合功能测试**：
   ```bash
   python3 test_full.py
   ```

---

## 相关文档

- **上车测试部署指南.md** - 完整的部署和测试文档
- **部署指南总结.md** - 快速参考指南
- **使用说明.md** - Python API详细说明

---

**更新日期**：2026-02-03  
**适用系统**：ROS2 Humble + Ubuntu 22.04  
**机器人**：Wheeltec全系列
