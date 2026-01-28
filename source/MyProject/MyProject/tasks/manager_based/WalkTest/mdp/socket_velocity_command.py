# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Socket velocity command generator for Isaac Lab tasks.

This module provides a command term that receives velocity commands via UDP socket
instead of ROS2. This avoids the Python version compatibility issues with ROS2.
"""

from __future__ import annotations

import logging
import socket
import threading
import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv
    from .socket_velocity_command_cfg import SocketVelocityCommandCfg

logger = logging.getLogger(__name__)


class SocketVelocityCommand(CommandTerm):
    """Socket velocity command receiver.

    This command term receives velocity commands (lin_vel_x, lin_vel_y, ang_vel_z)
    via UDP socket. This is a simple alternative to ROS2 that doesn't require
    complex dependencies.

    The socket server listens on a specified port and expects CSV-formatted
    messages with velocity commands:

    - Without heading: "lin_x,lin_y,ang_z"
    - With heading: "lin_x,lin_y,ang_z,heading"

    If heading_command is True, the angular velocity is computed from the heading
    error similar to a proportional controller.
    """

    cfg: "SocketVelocityCommandCfg"
    """The configuration of the command generator."""

    def __init__(self, cfg: "SocketVelocityCommandCfg", env: ManagerBasedEnv):
        """Initialize the socket command generator.

        Args:
            cfg: The configuration of the command generator.
            env: The environment.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # obtain the robot asset
        self.robot: Articulation = env.scene[cfg.asset_name]

        # create buffers to store the command
        # -- command: x vel, y vel, yaw vel
        self.vel_command_b = torch.zeros(self.num_envs, 3, device=self.device)

        # heading target buffer (used if heading_command is True)
        self.heading_target = torch.zeros(self.num_envs, device=self.device)

        # Initialize socket server
        self._socket = None
        self._socket_thread = None
        self._command_lock = threading.Lock()
        # Current command: [lin_x, lin_y, ang_z_or_heading]
        # If heading_command is True, the 3rd value is heading target
        self._current_command = [0.0, 0.0, 0.0, 0.0]  # [lin_x, lin_y, ang_z, heading]
        self._socket_initialized = False

        # Initialize socket
        self._init_socket()

        # -- metrics
        self.metrics["error_vel_xy"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_vel_yaw"] = torch.zeros(self.num_envs, device=self.device)

        if self._socket_initialized:
            mode = "heading" if cfg.heading_command else "angular velocity"
            logger.info(f"SocketVelocityCommand initialized: listening on port {cfg.port} ({mode} mode)")
        else:
            logger.warning("SocketVelocityCommand: Socket initialization failed, using zero commands")

    def __del__(self):
        """Cleanup socket resources."""
        self._shutdown_socket()

    def __str__(self) -> str:
        """Return a string representation of the command generator."""
        msg = "SocketVelocityCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tSocket port: {self.cfg.port}\n"
        msg += f"\tCommand applied to all environments"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired base velocity command in the base frame. Shape is (num_envs, 3)."""
        return self.vel_command_b

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        # Update metrics based on current state
        max_command_time = 1.0
        max_command_step = max_command_time / self._env.step_dt if self._env.step_dt > 0 else 1.0

        self.metrics["error_vel_xy"] += (
            torch.norm(self.vel_command_b[:, :2] - self.robot.data.root_lin_vel_b[:, :2], dim=-1) / max_command_step
        )
        self.metrics["error_vel_yaw"] += (
            torch.abs(self.vel_command_b[:, 2] - self.robot.data.root_ang_vel_b[:, 2]) / max_command_step
        )

    def _resample_command(self, env_ids):
        # Commands are received continuously from socket
        pass

    def _update_command(self):
        """Update velocity command from socket.

        This function reads the latest command from socket and applies it to all environments.
        If heading_command is True, the angular velocity is computed from heading error.
        """
        with self._command_lock:
            lin_x = self._current_command[0]
            lin_y = self._current_command[1]
            value_z = self._current_command[2]  # Can be ang_z or heading depending on mode

        # Clip linear commands to configured ranges
        lin_x = float(torch.tensor(lin_x, device=self.device).clip(
            min=self.cfg.ranges.lin_vel_x[0],
            max=self.cfg.ranges.lin_vel_x[1]))
        lin_y = float(torch.tensor(lin_y, device=self.device).clip(
            min=self.cfg.ranges.lin_vel_y[0],
            max=self.cfg.ranges.lin_vel_y[1]))

        # Apply to all environments
        self.vel_command_b[:, 0] = lin_x
        self.vel_command_b[:, 1] = lin_y

        # Handle angular velocity or heading command
        if self.cfg.heading_command:
            # Heading command mode: compute angular velocity from heading error
            # value_z contains the target heading
            heading = float(torch.tensor(value_z, device=self.device).clip(
                min=self.cfg.ranges.heading[0],
                max=self.cfg.ranges.heading[1]))

            # Update heading target for all environments
            self.heading_target[:] = heading

            # Compute angular velocity from heading error (proportional control)
            # Similar to: omega_z = k * wrap_to_pi(target_heading - current_heading)
            heading_error = math_utils.wrap_to_pi(
                self.heading_target - self.robot.data.heading_w
            )
            ang_z = torch.clip(
                self.cfg.heading_control_stiffness * heading_error,
                min=self.cfg.ranges.ang_vel_z[0],
                max=self.cfg.ranges.ang_vel_z[1]
            )

            self.vel_command_b[:, 2] = ang_z
        else:
            # Angular velocity mode: use value_z directly
            ang_z = float(torch.tensor(value_z, device=self.device).clip(
                min=self.cfg.ranges.ang_vel_z[0],
                max=self.cfg.ranges.ang_vel_z[1]))
            self.vel_command_b[:, 2] = ang_z

        # Log for debugging (only log periodically to avoid spam)
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 100 == 0:  # Log every 100 iterations
            logger.info(f"Socket command updated: lin_x={lin_x:.3f}, lin_y={lin_y:.3f}, ang_z={ang_z:.3f}")

    def _set_debug_vis_impl(self, debug_vis: bool):
        # Debug visualization not implemented
        pass

    def _debug_vis_callback(self, event):
        # No debug visualization
        pass

    """
    Socket-specific functions
    """

    def _init_socket(self):
        """Initialize UDP socket server."""
        try:
            # Create UDP socket
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind(('127.0.0.1', self.cfg.port))

            # Start receive thread
            self._socket_thread = threading.Thread(target=self._socket_receive_loop, daemon=True)
            self._socket_thread.start()

            self._socket_initialized = True
            logger.info("="*60)
            logger.info(f"✓ Socket server initialized on port {self.cfg.port}")
            logger.info(f"✓ Listening for commands on 127.0.0.1:{self.cfg.port}")
            logger.info("="*60)

        except Exception as e:
            logger.error("="*60)
            logger.error("✗ Socket initialization failed")
            logger.error(f"Error: {e}")
            logger.error("="*60)
            self._socket_initialized = False

    def _socket_receive_loop(self):
        """Receive loop running in separate thread."""
        while True:
            try:
                # Receive data
                data, addr = self._socket.recvfrom(1024)

                # Parse command
                # Format without heading: "lin_x,lin_y,ang_z"
                # Format with heading: "lin_x,lin_y,heading"
                message = data.decode('utf-8').strip()

                try:
                    # Parse the command
                    parts = message.split(',')

                    if len(parts) == 3:
                        # Standard format: lin_x, lin_y, ang_z (or heading if heading_command=True)
                        lin_x = float(parts[0])
                        lin_y = float(parts[1])
                        value_z = float(parts[2])

                        with self._command_lock:
                            # Store 4 values: [lin_x, lin_y, value_z, heading]
                            # If heading_command mode, value_z is heading, otherwise it's ang_z
                            self._current_command = [lin_x, lin_y, value_z, value_z]

                    elif len(parts) == 4:
                        # Extended format: lin_x, lin_y, ang_z, heading
                        lin_x = float(parts[0])
                        lin_y = float(parts[1])
                        ang_z = float(parts[2])
                        heading = float(parts[3])

                        with self._command_lock:
                            self._current_command = [lin_x, lin_y, ang_z, heading]
                    else:
                        logger.warning(f"Invalid command format: expected 3 or 4 values, got {len(parts)}")
                        continue

                    # Log for debugging
                    if not hasattr(self, '_recv_counter'):
                        self._recv_counter = 0
                    self._recv_counter += 1
                    if self._recv_counter % 50 == 0:
                        cmd_mode = "heading" if self.cfg.heading_command else "angular velocity"
                        logger.info(
                            f"Received command #{self._recv_counter}: "
                            f"lin_x={self._current_command[0]:.3f}, "
                            f"lin_y={self._current_command[1]:.3f}, "
                            f"{cmd_mode}={self._current_command[2]:.3f}"
                        )

                except ValueError as e:
                    logger.warning(f"Invalid command format: {message}, Error: {e}")

            except Exception as e:
                logger.error(f"Error receiving data: {e}")

    def _shutdown_socket(self):
        """Shutdown socket and cleanup resources."""
        try:
            if self._socket:
                self._socket.close()
            logger.info("Socket server shutdown complete")
        except Exception as e:
            logger.error(f"Error shutting down socket: {e}")
