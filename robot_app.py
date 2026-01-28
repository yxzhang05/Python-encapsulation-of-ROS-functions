# 文件名: main.py
from robot_lib import Robot
import time

def main():
    # 1. 实例化机器人对象
    bot = Robot()

    try:
        # 2. 初始化 (传入车型: akm, diff, 或 mec)
        # 这里假设我们用的是麦轮
        success = bot.initialize("mec")
        
        if not success:
            print("初始化失败，程序退出")
            return

        # 3. 获取并打印电压
        vol = bot.get_battery_voltage()
        print(f"当前电池电压: {vol:.2f} V")

        if vol < 10.5:
            print("【警告】电量过低，请充电！")
        
        # 4. 启动键盘控制 (这会暂停代码执行，直到你按 Ctrl+C 退出键盘)
        # 如果你想直接写自动控制代码，就不调用这个，直接调用后面会写的 set_velocity 等函数
        bot.start_keyboard_control()

    except KeyboardInterrupt:
        print("用户强制停止")
    
    finally:
        # 5. 无论发生什么错误，最后都安全关闭
        bot.shutdown()

if __name__ == "__main__":
    main()