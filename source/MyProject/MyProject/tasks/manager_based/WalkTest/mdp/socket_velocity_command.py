# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Socket velocity command - changes ranges to fix command value."""

import logging
import socket
import threading

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand

logger = logging.getLogger(__name__)


class SocketVelocityCommand(UniformVelocityCommand):
    """Socket command receiver - updates cfg.ranges to fix command values.

    When socket receives "0.5,0.0,0.0", sets ranges to:
        lin_vel_x=(0.5, 0.5), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0)
    So random sampling always returns the same value.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._current_command = [0.0, 0.0, 0.0]
        self._command_lock = threading.Lock()
        self._init_socket()

    def _init_socket(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', self.cfg.port))
            threading.Thread(target=self._receive_loop, args=(sock,), daemon=True).start()
            logger.info(f"✓ Socket listening on port {self.cfg.port}")
        except Exception as e:
            logger.error(f"✗ Socket init failed: {e}")

    def _receive_loop(self, sock):
        while True:
            try:
                data, _ = sock.recvfrom(1024)
                parts = data.decode().strip().split(',')
                if len(parts) >= 3:
                    # Update cfg socket values
                    self.cfg.socket_vx = float(parts[0])
                    self.cfg.socket_vy = float(parts[1])
                    self.cfg.socket_wz = float(parts[2])

                    # Update ranges to fix the command value
                    self.cfg.ranges.lin_vel_x = (self.cfg.socket_vx, self.cfg.socket_vx)
                    self.cfg.ranges.lin_vel_y = (self.cfg.socket_vy, self.cfg.socket_vy)
                    self.cfg.ranges.ang_vel_z = (self.cfg.socket_wz, self.cfg.socket_wz)

                    logger.info(f"Command: {self.cfg.socket_vx:.2f}, {self.cfg.socket_vy:.2f}, {self.cfg.socket_wz:.2f}")
            except Exception as e:
                logger.error(f"Receive error: {e}")
