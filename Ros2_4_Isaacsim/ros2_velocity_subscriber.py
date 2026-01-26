#!/usr/bin/env python3
"""
ROS2 Velocity Command Subscriber for Isaac Sim

This node subscribes to velocity commands from ROS2 and bridges them
to Isaac Lab's command system for the Unitree Go2 robot.

The node creates a shared memory interface that can be accessed by
the Isaac Sim environment to override the default velocity commands.

Usage:
    # Terminal 1: Start the ROS2 subscriber
    python ros2_velocity_subscriber.py

    # Terminal 2: Run Isaac Sim with the modified play script
    python play_ros2.py --task Template-Velocity-Test-Unitree-Go2-Play-v0 \
        --checkpoint /path/to/checkpoint.pt
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import numpy as np
import multiprocessing
import multiprocessing.shared_memory
import time
from dataclasses import dataclass


@dataclass
class VelocityCommand:
    """Data structure for velocity command."""
    lin_vel_x: float = 0.0
    lin_vel_y: float = 0.0
    ang_vel_z: float = 0.0
    heading: float = 0.0
    timestamp: float = 0.0


class Ros2CommandBridge:
    """
    Bridge for sharing ROS2 commands with Isaac Sim via shared memory.

    This class creates a shared memory region that can be accessed by
    both the ROS2 node and the Isaac Sim environment.
    """

    # Shared memory configuration
    SHM_NAME = "ros2_velocity_command"
    SHM_SIZE = 256  # bytes

    def __init__(self, create=True):
        """
        Initialize the shared memory bridge.

        Args:
            create: If True, create new shared memory. If False, attach to existing.
        """
        try:
            if create:
                self.shm = multiprocessing.shared_memory.SharedMemory(
                    name=self.SHM_NAME,
                    create=True,
                    size=self.SHM_SIZE
                )
                # Initialize with zeros
                self.shm.buf[:self.SHM_SIZE] = b'\x00' * self.SHM_SIZE
            else:
                self.shm = multiprocessing.shared_memory.SharedMemory(
                    name=self.SHM_NAME,
                    create=False
                )
            self.created = create
        except FileExistsError:
            if create:
                # Attach to existing and clear it
                self.shm = multiprocessing.shared_memory.SharedMemory(
                    name=self.SHM_NAME,
                    create=False
                )
                self.created = False
            else:
                raise

    def write_command(self, command: VelocityCommand):
        """Write velocity command to shared memory."""
        data = np.array([
            command.lin_vel_x,
            command.lin_vel_y,
            command.ang_vel_z,
            command.heading,
            command.timestamp
        ], dtype=np.float64)

        # Write to shared memory
        shm_buf = np.frombuffer(self.shm.buf, dtype=np.float64)
        shm_buf[:len(data)] = data

    def read_command(self) -> VelocityCommand:
        """Read velocity command from shared memory."""
        shm_buf = np.frombuffer(self.shm.buf, dtype=np.float64)

        return VelocityCommand(
            lin_vel_x=float(shm_buf[0]),
            lin_vel_y=float(shm_buf[1]),
            ang_vel_z=float(shm_buf[2]),
            heading=float(shm_buf[3]),
            timestamp=float(shm_buf[4])
        )

    def close(self):
        """Close the shared memory connection."""
        self.shm.close()

    def unlink(self):
        """Remove the shared memory (call from creator only)."""
        if self.created:
            self.shm.unlink()


class VelocitySubscriber(Node):
    """ROS2 Node to subscribe to velocity commands and bridge to Isaac Sim."""

    def __init__(self, command_timeout=1.0):
        """
        Initialize the velocity subscriber.

        Args:
            command_timeout: Timeout in seconds before resetting to zero velocity
        """
        super().__init__('velocity_command_subscriber')

        # Create shared memory bridge
        self.bridge = Ros2CommandBridge(create=True)

        # Create subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Parameters
        self.command_timeout = command_timeout
        self.last_command_time = time.time()

        # Status timer
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Velocity Subscriber started')
        self.get_logger().info(f'Listening to /cmd_vel topic')
        self.get_logger().info(f'Command timeout: {command_timeout} seconds')
        self.get_logger().info(f'Shared memory: {self.bridge.SHM_NAME}')

    def listener_callback(self, msg: Twist):
        """
        Callback for received velocity commands.

        Args:
            msg: Twist message from ROS2
        """
        # Extract velocity commands
        command = VelocityCommand(
            lin_vel_x=float(msg.linear.x),
            lin_vel_y=float(msg.linear.y),
            ang_vel_z=float(msg.angular.z),
            heading=0.0,  # Not used in velocity mode
            timestamp=time.time()
        )

        # Write to shared memory
        self.bridge.write_command(command)
        self.last_command_time = time.time()

    def status_callback(self):
        """Periodic status check and timeout handling."""
        elapsed = time.time() - self.last_command_time

        if elapsed > self.command_timeout:
            # Timeout - reset to zero velocity
            zero_command = VelocityCommand(
                lin_vel_x=0.0,
                lin_vel_y=0.0,
                ang_vel_z=0.0,
                heading=0.0,
                timestamp=time.time()
            )
            self.bridge.write_command(zero_command)
            self.get_logger().warn(f'Command timeout ({elapsed:.1f}s), resetting to zero velocity')

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Closing shared memory bridge...')
        self.bridge.unlink()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(description='ROS2 Velocity Command Subscriber')
    parser.add_argument('--timeout', type=float, default=1.0,
                       help='Command timeout in seconds (default: 1.0)')

    parsed_args = parser.parse_args(args)

    rclpy.init(args=args)
    velocity_subscriber = VelocitySubscriber(command_timeout=parsed_args.timeout)

    try:
        rclpy.spin(velocity_subscriber)
    except KeyboardInterrupt:
        velocity_subscriber.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        velocity_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
