#!/usr/bin/env python3
"""验证清理后的代码 - 快速测试"""

import socket
import time

print("="*60)
print("验证清理后的代码")
print("="*60)

# 测试 1：检查 Socket 是否启动
print("\n测试 1: 等待 Socket 启动...")
print("请在另一个终端运行 Isaac Lab，然后按 Enter 继续...")
input()

# 测试 2：发送命令
print("\n测试 2: 发送命令...")
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)

    test_commands = [
        ("0.5,0.0,0.0", "前进 0.5 m/s"),
        ("0.0,0.0,0.0", "停止"),
        ("0.0,0.5,0.0", "左移 0.5 m/s"),
    ]

    for cmd, desc in test_commands:
        print(f"  发送: {desc}")
        sock.sendto(cmd.encode(), ('127.0.0.1', 5555))
        time.sleep(2)

    sock.close()
    print("\n✓ 所有命令已发送")

except socket.error as e:
    print(f"\n✗ Socket 错误: {e}")
    print("  请确保 Isaac Lab 正在运行")
    exit(1)

# 测试 3：检查日志
print("\n测试 3: 检查日志...")
print("请检查 Isaac Lab 终端输出：")
print("  ✓ 应该看到: Socket listening on port 5555")
print("  ✓ 应该看到: [Socket] Command: 0.50, 0.00, 0.00")
print("  ✗ 不应该看到: [DEBUG] Fixed Command - Target: ...")
print("  ✗ 不应该看到: [DEBUG] Fixed Command - Actual: ...")
print("  ✗ 不应该看到: [DEBUG] feet_air_time - Command norm mean: ...")

print("\n" + "="*60)
print("验证完成！")
print("="*60)
print("\n如果看到上述输出，说明清理成功！")
