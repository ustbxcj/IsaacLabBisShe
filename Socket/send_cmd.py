#!/usr/bin/env python3
"""Socket command sender for testing - Angular velocity mode.

Usage:
    python send_cmd.py
"""

import socket
import time
import sys
import termios
import tty

# Configuration
HOST = '127.0.0.1'
PORT = 5555

class SocketCommandSender:
    """持续连接的命令发送器"""

    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        # 创建一次 socket，重复使用
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Connected to {host}:{port}")
        print("Mode: Angular Velocity (direct control)")

    def send_command(self, lin_x, lin_y, ang_z):
        """Send velocity command via UDP socket (angular velocity mode)."""
        message = f"{lin_x},{lin_y},{ang_z}"
        self.sock.sendto(message.encode('utf-8'), (self.host, self.port))
        print(f"Sent: lin_x={lin_x:.2f}, lin_y={lin_y:.2f}, ang_z={ang_z:.2f}")

    def close(self):
        """关闭 socket"""
        self.sock.close()
        print("Connection closed")

def get_key():
    """获取单个按键（不回车）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    """主函数：键盘控制模式"""
    sender = SocketCommandSender()

    print("="*50)
    print("Socket Command Sender (Keyboard Control)")
    print("="*50)
    print("Mode: Angular Velocity (direct control)")
    print("="*50)
    print("Controls:")
    print("  W - Forward")
    print("  S - Backward")
    print("  A - Left")
    print("  D - Right")
    print("  Q - Rotate Left")
    print("  E - Rotate Right")
    print("  Space - Stop")
    print("  ESC - Quit")
    print("="*50)
    print("\nPress keys to control robot...")
    print("NOTE: In angular velocity mode, Q/E continuously rotate.\n")

    # 当前命令状态
    current_cmd = [0.0, 0.0, 0.0]  # [lin_x, lin_y, ang_z]
    last_sent_cmd = None

    try:
        while True:
            key = get_key()

            if key == '\x1b':  # ESC
                print("\nQuitting...")
                break

            elif key.lower() == 'w':
                current_cmd = [0.5, 0.0, 0.0]
                print("Forward")

            elif key.lower() == 's':
                current_cmd = [-0.3, 0.0, 0.0]
                print("Backward")

            elif key.lower() == 'a':
                current_cmd = [0.0, 0.5, 0.0]
                print("Left")

            elif key.lower() == 'd':
                current_cmd = [0.0, -0.5, 0.0]
                print("Right")

            elif key.lower() == 'q':
                current_cmd = [0.0, 0.0, 0.5]
                print("Rotate Left")

            elif key.lower() == 'e':
                current_cmd = [0.0, 0.0, -0.5]
                print("Rotate Right")

            elif key == ' ':  # Space
                current_cmd = [0.0, 0.0, 0.0]
                print("Stop")

            # 只在命令改变时发送，避免重复发送相同命令
            if last_sent_cmd != current_cmd:
                sender.send_command(*current_cmd)
                last_sent_cmd = current_cmd.copy()

            # 短暂延迟
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        # 停止机器人
        sender.send_command(0.0, 0.0, 0.0)
        time.sleep(0.1)
        sender.close()

if __name__ == "__main__":
    main()
