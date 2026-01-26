#!/usr/bin/env python3
"""
ROS2 Velocity Command Publisher for Unitree Go2 Robot

This node publishes velocity commands for controlling the quadruped robot.
It can run in different modes:
- Direct mode: Set velocity directly via command line arguments
- Manual mode: User can input velocity commands via keyboard
- Auto mode: Generates random velocity commands periodically
- Sine wave mode: Generates smooth sine wave velocity commands

Usage:
    python ros2_velocity_publisher.py --mode direct --vel-x 0.5 --vel-y 0.0 --vel-z 0.1
    python ros2_velocity_publisher.py --mode manual
    python ros2_velocity_publisher.py --mode auto --rate 10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import math
import time
import sys
import select
import termios
import tty


class VelocityPublisher(Node):
    """ROS2 Node to publish velocity commands for Unitree Go2."""

    def __init__(self, mode='direct', publish_rate=20.0, vel_x=0.0, vel_y=0.0, vel_z=0.0):
        """
        Initialize the velocity publisher.

        Args:
            mode: Operating mode ('direct', 'manual', 'auto', 'sine')
            publish_rate: Publishing rate in Hz
            vel_x: Initial linear x velocity (for direct mode)
            vel_y: Initial linear y velocity (for direct mode)
            vel_z: Initial angular z velocity (for direct mode)
        """
        super().__init__('velocity_command_publisher')

        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.mode = mode
        self.publish_rate = publish_rate
        self.timer_period = 1.0 / publish_rate

        # State variables
        self.current_vel = {'x': float(vel_x), 'y': float(vel_y), 'z': float(vel_z)}
        self.start_time = time.time()

        # Create timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Velocity limits (matching the WalkTest config)
        self.max_lin_vel_x = 1.0
        self.max_lin_vel_y = 1.0
        self.max_ang_vel_z = 1.0

        self.get_logger().info(f'Velocity Publisher started in {mode} mode')
        self.get_logger().info(f'Publishing to /cmd_vel at {publish_rate} Hz')
        self.get_logger().info(f'Velocity: x={self.current_vel["x"]:.2f}, y={self.current_vel["y"]:.2f}, z={self.current_vel["z"]:.2f}')

        if mode == 'manual':
            self.print_manual_instructions()

    def print_manual_instructions(self):
        """Print keyboard control instructions."""
        print("\n" + "="*60)
        print("Manual Velocity Control")
        print("="*60)
        print("Use the following keys to control the robot:")
        print("  w/W : Increase forward velocity")
        print("  s/S : Decrease forward velocity")
        print("  a/A : Increase left velocity")
        print("  d/D : Increase right velocity")
        print("  q/Q : Increase counter-clockwise rotation")
        print("  e/E : Increase clockwise rotation")
        print("  r/R : Reset all velocities to zero")
        print("  ESC : Exit the program")
        print("="*60)
        print(f"\nCurrent velocity: x={self.current_vel['x']:.2f}, "
              f"y={self.current_vel['y']:.2f}, "
              f"z={self.current_vel['z']:.2f}")
        print("\nPress a key to start...")
        print("="*60 + "\n")

    def get_key(self):
        """Get a single keypress from terminal (non-blocking)."""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == '\x1b':  # ESC key
                    return 'ESC'
                return key
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def update_manual_velocity(self, key):
        """Update velocity based on key input."""
        step = 0.1

        if key is None:
            return

        key_lower = key.lower()

        if key_lower == 'w':
            self.current_vel['x'] += step
        elif key_lower == 's':
            self.current_vel['x'] -= step
        elif key_lower == 'a':
            self.current_vel['y'] += step
        elif key_lower == 'd':
            self.current_vel['y'] -= step
        elif key_lower == 'q':
            self.current_vel['z'] += step
        elif key_lower == 'e':
            self.current_vel['z'] -= step
        elif key_lower == 'r':
            self.current_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Clamp velocities to limits
        self.current_vel['x'] = max(min(self.current_vel['x'], self.max_lin_vel_x), -self.max_lin_vel_x)
        self.current_vel['y'] = max(min(self.current_vel['y'], self.max_lin_vel_y), -self.max_lin_vel_y)
        self.current_vel['z'] = max(min(self.current_vel['z'], self.max_ang_vel_z), -self.max_ang_vel_z)

        # Print current velocity
        sys.stdout.write(f"\rVelocity: x={self.current_vel['x']:6.2f}, "
                        f"y={self.current_vel['y']:6.2f}, "
                        f"z={self.current_vel['z']:6.2f}    ")
        sys.stdout.flush()

    def generate_auto_velocity(self):
        """Generate random velocity command."""
        import random
        self.current_vel = {
            'x': random.uniform(-self.max_lin_vel_x, self.max_lin_vel_x),
            'y': random.uniform(-self.max_lin_vel_y, self.max_lin_vel_y),
            'z': random.uniform(-self.max_ang_vel_z, self.max_ang_vel_z)
        }
        self.get_logger().info(f"Auto mode: Generated velocity - "
                              f"x={self.current_vel['x']:.2f}, "
                              f"y={self.current_vel['y']:.2f}, "
                              f"z={self.current_vel['z']:.2f}")

    def generate_sine_velocity(self):
        """Generate smooth sine wave velocity command."""
        elapsed = time.time() - self.start_time
        self.current_vel = {
            'x': 0.5 * math.sin(0.5 * elapsed),  # 0.5 Hz sine wave
            'y': 0.3 * math.sin(0.3 * elapsed),  # 0.3 Hz sine wave
            'z': 0.5 * math.sin(0.2 * elapsed)   # 0.2 Hz sine wave
        }
        if int(elapsed * 10) % 10 == 0:  # Print every second
            self.get_logger().info(f"Sine mode: Generated velocity - "
                                  f"x={self.current_vel['x']:.2f}, "
                                  f"y={self.current_vel['y']:.2f}, "
                                  f"z={self.current_vel['z']:.2f}")

    def timer_callback(self):
        """Timer callback for publishing velocity commands."""
        # Generate velocity based on mode
        if self.mode == 'direct':
            # Direct mode - just use the initial velocity values
            pass
        elif self.mode == 'manual':
            key = self.get_key()
            if key == 'ESC':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                return
            self.update_manual_velocity(key)
        elif self.mode == 'auto':
            self.generate_auto_velocity()
        elif self.mode == 'sine':
            self.generate_sine_velocity()

        # Create and publish Twist message
        msg = Twist()
        msg.linear.x = float(self.current_vel['x'])
        msg.linear.y = float(self.current_vel['y'])
        msg.linear.z = 0.0  # Not used for quadruped
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(self.current_vel['z'])

        self.publisher.publish(msg)


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='ROS2 Velocity Command Publisher')
    parser.add_argument('--mode', type=str, default='direct',
                       choices=['direct', 'manual', 'auto', 'sine'],
                       help='Operating mode: direct (cmd line), manual (keyboard), auto (random), or sine (smooth sine waves)')
    parser.add_argument('--rate', type=float, default=20.0,
                       help='Publishing rate in Hz (default: 20.0)')
    parser.add_argument('--vel-x', type=float, default=0.0,
                       help='Linear x velocity (default: 0.0)')
    parser.add_argument('--vel-y', type=float, default=0.0,
                       help='Linear y velocity (default: 0.0)')
    parser.add_argument('--vel-z', type=float, default=0.0,
                       help='Angular z velocity (default: 0.0)')

    parsed_args = parser.parse_args(args)

    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher(
        mode=parsed_args.mode,
        publish_rate=parsed_args.rate,
        vel_x=parsed_args.vel_x,
        vel_y=parsed_args.vel_y,
        vel_z=parsed_args.vel_z
    )

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        velocity_publisher.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
