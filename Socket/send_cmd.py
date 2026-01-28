#!/usr/bin/env python3
"""Minimal socket command sender for testing.

Usage:
    python socket.py
"""

import socket
import time

# Configuration
HOST = '127.0.0.1'
PORT = 5555

def send_command(lin_x, lin_y, ang_z):
    """Send velocity command via UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message = f"{lin_x},{lin_y},{ang_z}"
    sock.sendto(message.encode('utf-8'), (HOST, PORT))
    sock.close()
    print(f"Sent: {message}")

if __name__ == "__main__":
    print("Minimal Socket Command Sender")
    print(f"Sending to {HOST}:{PORT}")
    print("-" * 40)

    # Send test commands
    send_command(0.5, 0.0, 0.0)   # Forward
    time.sleep(0.5)
    send_command(0.0, 0.0, 0.5)   # Turn left
    time.sleep(0.5)
    send_command(0.0, 0.0, 0.0)   # Stop

    print("-" * 40)
    print("Done!")
