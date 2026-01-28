#!/usr/bin/env python3
"""Minimal socket command sender for testing.

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

    def send_command(self, lin_x, lin_y, ang_z):
        """Send velocity command via UDP socket."""
        message = f"{lin_x},{lin_y},{ang_z}"
        self.sock.sendto(message.encode('utf-8'), (self.host, self.port))

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
    print("Controls:")
    print("  W - Forward")
    print("  S - Backward")
    print("  A - Left")
    print("  D - Right")
    print("  Q - Turn Left")
    print("  E - Turn Right")
    print("  Space - Stop")
    print("  ESC - Quit")
    print("="*50)
    print("\nPress keys to control robot...")

    # 当前命令状态
    current_cmd = [0.0, 0.0, 0.0]  # [lin_x, lin_y, ang_z]

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
                print("Turn Left")

            elif key.lower() == 'e':
                current_cmd = [0.0, 0.0, -0.5]
                print("Turn Right")

            elif key == ' ':  # Space
                current_cmd = [0.0, 0.0, 0.0]
                print("Stop")

            # 发送当前命令
            sender.send_command(*current_cmd)

            # 短暂延迟，避免发送过快
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        # 停止机器人
        sender.send_command(0.0, 0.0, 0.0)
        sender.close()

if __name__ == "__main__":
    main()
