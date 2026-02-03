import subprocess
import re
import math

def _get_odom_improved(self, debug=False):
    """
    获取里程计数据（改进版）
    支持科学计数法，提供详细的错误信息
    """
    try:
        # 执行命令获取 odom 数据
        cmd = ["ros2", "topic", "echo", "/odom", "--once"]
        if debug:
            print(f"[Debug] 执行命令: {' '.join(cmd)}")
        
        output = subprocess.check_output(cmd, timeout=2.0, stderr=subprocess.PIPE).decode("utf-8")
        
        if debug:
            print(f"[Debug] 命令输出长度: {len(output)} 字符")
            print(f"[Debug] 输出前500字符:\n{output[:500]}")
        
        # 改进的正则表达式，支持科学计数法
        # 匹配: 123, -123, 1.23, -1.23, 1.0e-09, 1.0E+09, -1.5e-10
        number_pattern = r"([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)"
        
        # 匹配位置信息
        pos_pattern = f"position:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}"
        pos_match = re.search(pos_pattern, output, re.DOTALL)
        
        if not pos_match:
            if debug:
                print("[Debug] 未能匹配位置信息")
                print(f"[Debug] 使用的模式: {pos_pattern}")
            return None
        
        # 匹配方向（四元数）
        quat_pattern = f"orientation:.*?x:\\s*{number_pattern}.*?y:\\s*{number_pattern}.*?z:\\s*{number_pattern}.*?w:\\s*{number_pattern}"
        quat_match = re.search(quat_pattern, output, re.DOTALL)
        
        if not quat_match:
            if debug:
                print("[Debug] 未能匹配方向信息")
                print(f"[Debug] 使用的模式: {quat_pattern}")
            return None
        
        # 提取数值
        x = float(pos_match.group(1))
        y = float(pos_match.group(2))
        qx = float(quat_match.group(1))
        qy = float(quat_match.group(2))
        qz = float(quat_match.group(3))
        qw = float(quat_match.group(4))
        
        if debug:
            print(f"[Debug] 解析结果:")
            print(f"  位置: x={x}, y={y}")
            print(f"  四元数: x={qx}, y={qy}, z={qz}, w={qw}")
        
        # 四元数转欧拉角（yaw）
        theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        if debug:
            print(f"  theta (yaw): {theta} rad = {math.degrees(theta)} 度")
        
        return {"x": x, "y": y, "theta": theta}
        
    except subprocess.TimeoutExpired as e:
        if debug:
            print(f"[Debug] 命令超时: {e}")
        print("[Error] _get_odom: 读取 /odom 话题超时")
        return None
    except subprocess.CalledProcessError as e:
        if debug:
            print(f"[Debug] 命令执行失败: {e}")
            print(f"[Debug] stderr: {e.stderr}")
        print("[Error] _get_odom: 无法读取 /odom 话题，请确保 ROS 底盘驱动已启动")
        return None
    except ValueError as e:
        if debug:
            print(f"[Debug] 数值转换失败: {e}")
        print("[Error] _get_odom: 数据解析失败，数值格式错误")
        return None
    except Exception as e:
        if debug:
            print(f"[Debug] 未知错误: {type(e).__name__}: {e}")
        print(f"[Error] _get_odom: 未知错误 - {type(e).__name__}: {e}")
        return None

# 测试函数
class TestRobot:
    pass

robot = TestRobot()
result = _get_odom_improved(robot, debug=True)
print(f"\n最终返回: {result}")
