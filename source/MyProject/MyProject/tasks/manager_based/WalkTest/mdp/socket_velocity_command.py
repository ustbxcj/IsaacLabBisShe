# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Socket velocity command - changes ranges to fix command value."""

import logging
import math
import socket
import threading
import torch

from isaaclab.envs.mdp.commands.velocity_command import UniformVelocityCommand

logger = logging.getLogger(__name__)


class SocketVelocityCommand(UniformVelocityCommand):
    """Socket command receiver - updates cfg.ranges to fix command values.

    When socket receives "0.5,0.0,0.0", sets ranges to:
        lin_vel_x=(0.5, 0.5), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0)
    So random sampling always returns the same value.
    
    To improve policy compatibility with fixed commands, small noise can be added to simulate
    the training distribution.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._current_command = [0.0, 0.0, 0.0]
        self._command_lock = threading.Lock()
        self._step_counter = 0
        
        # Command transformation settings
        self.add_command_noise = getattr(cfg, 'add_command_noise', True)
        self.noise_scale = getattr(cfg, 'noise_scale', 0.05)
        self.use_sinusoidal_variation = getattr(cfg, 'use_sinusoidal_variation', False)
        self.variation_amp = getattr(cfg, 'variation_amp', 0.03)
        
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

    def _is_fixed_command(self):
        """Check if current command is fixed (range min == max)."""
        return (
            abs(self.cfg.ranges.lin_vel_x[1] - self.cfg.ranges.lin_vel_x[0]) < 1e-6 and
            abs(self.cfg.ranges.lin_vel_y[1] - self.cfg.ranges.lin_vel_y[0]) < 1e-6 and
            abs(self.cfg.ranges.ang_vel_z[1] - self.cfg.ranges.ang_vel_z[0]) < 1e-6
        )

    def _resample_command(self, env_ids):
        """Override to add transformation to fixed commands for better policy compatibility."""
        
        # Check if this is a fixed command
        is_fixed = self._is_fixed_command()
        
        if is_fixed and (self.add_command_noise or self.use_sinusoidal_variation):
            # Get target values
            target_vx = self.cfg.socket_vx
            target_vy = self.cfg.socket_vy
            target_wz = self.cfg.socket_wz
            
            num_envs = len(env_ids)
            r = torch.empty(num_envs, device=self.device)
            
            if self.use_sinusoidal_variation:
                # Add time-varying component (smooth variation)
                self._step_counter +=1
                phase = 2 * math.pi * self._step_counter / 100.0
                
                # Per-environment variation with shared phase
                noise_x = self.variation_amp * math.sin(phase) + r.uniform_(-0.01, 0.01)
                noise_y = self.variation_amp * math.cos(phase * 1.3) + r.uniform_(-0.01, 0.01)
                noise_z = self.variation_amp * math.sin(phase * 0.7) + r.uniform_(-0.01, 0.01)
            else:
                # Add random noise PER ENVIRONMENT
                noise_x = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)
                noise_y = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)
                noise_z = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)
            
            self.vel_command_b[env_ids, 0] = target_vx + noise_x
            self.vel_command_b[env_ids, 1] = target_vy + noise_y
            self.vel_command_b[env_ids, 2] = target_wz + noise_z
            
            # DEBUG: Log command statistics
            actual_vx_mean = self.vel_command_b[env_ids, 0].mean().item()
            actual_vy_mean = self.vel_command_b[env_ids, 1].mean().item()
            actual_wz_mean = self.vel_command_b[env_ids, 2].mean().item()
            actual_vx_std = self.vel_command_b[env_ids, 0].std().item()
            actual_vy_std = self.vel_command_b[env_ids, 1].std().item()
            actual_wz_std = self.vel_command_b[env_ids, 2].std().item()
            logger.info(f"[DEBUG] Fixed Command - Target: ({target_vx:.3f}, {target_vy:.3f}, {target_wz:.3f}), Noise scale: ±{self.noise_scale:.3f}")
            logger.info(f"[DEBUG] Fixed Command - Actual: vx={actual_vx_mean:.3f}±{actual_vx_std:.3f}, vy={actual_vy_mean:.3f}±{actual_vy_std:.3f}, wz={actual_wz_mean:.3f}±{actual_wz_std:.3f}")
            
            # Handle heading command if enabled
            if self.cfg.heading_command:
                self.heading_target[env_ids] = r.uniform_(*self.cfg.ranges.heading)
                self.is_heading_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_heading_envs
            
            # Update standing envs
            r = torch.empty(num_envs, device=self.device)
            self.is_standing_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_standing_envs
            # Zero out commands for standing environments
            standing_mask = self.is_standing_env[env_ids]
            self.vel_command_b[env_ids][standing_mask] = 0.0
            
        else:
            # Use standard uniform sampling from parent class
            super()._resample_command(env_ids)
