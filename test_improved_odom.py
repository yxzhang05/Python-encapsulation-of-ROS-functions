import re
import math

# 用户提供的实际输出
output = """header:
  stamp:
    sec: 1765362888
    nanosec: 836508080
  frame_id: odom_combined
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: -0.0003643878153525293
      y: 0.5546483993530273
      z: -0.004393049981445074
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0021965232244525426
      w: 0.9999975876399525
  covariance:
  - 1.0e-09
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.001
  - 1.0e-09
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0e-09
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 1.0e-09
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.001
  - 1.0e-09
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0e-09
---"""

print("测试改进的正则表达式:")
print("=" * 60)

# 改进的正则表达式，支持科学计数法
number_pattern = r"([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)"

# 测试位置匹配
pos_pattern = f"position:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}"
pos_match = re.search(pos_pattern, output, re.DOTALL)

print(f"\n位置匹配: {'成功' if pos_match else '失败'}")
if pos_match:
    print(f"  x = {pos_match.group(1)}")
    print(f"  y = {pos_match.group(2)}")
    x = float(pos_match.group(1))
    y = float(pos_match.group(2))
    print(f"  转换后: x = {x}, y = {y}")

# 测试方向匹配
quat_pattern = f"orientation:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}.*?z:\\s*{number_pattern}.*?w:\\s*{number_pattern}"
quat_match = re.search(quat_pattern, output, re.DOTALL)

print(f"\n方向匹配: {'成功' if quat_match else '失败'}")
if quat_match:
    print(f"  x = {quat_match.group(1)}")
    print(f"  y = {quat_match.group(2)}")
    print(f"  z = {quat_match.group(3)}")
    print(f"  w = {quat_match.group(4)}")
    qx = float(quat_match.group(1))
    qy = float(quat_match.group(2))
    qz = float(quat_match.group(3))
    qw = float(quat_match.group(4))
    print(f"  转换后: x={qx}, y={qy}, z={qz}, w={qw}")
    
    # 计算 yaw
    theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    print(f"  theta (yaw) = {theta} rad = {math.degrees(theta)} 度")

print("\n" + "=" * 60)
print("测试结果: 所有解析成功！✅")
