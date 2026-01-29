# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.utils import configclass

from .walk_rough_env_cfg import VelocityGo2WalkRoughEnvCfg
from .mdp import SocketVelocityCommandCfg
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp


@configclass
class VelocityGo2WalkFlatEnvCfg(VelocityGo2WalkRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # override rewards to match official flat terrain config
        self.rewards.flat_orientation_l2.weight = -2.5
        self.rewards.feet_air_time.weight = 0.25

        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None


@configclass
class VelocityGo2WalkFlatEnvCfg_Play(VelocityGo2WalkFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
        # self.commands.base_velocity = mdp.UniformVelocityCommandCfg(
        #     asset_name="robot",
        #     resampling_time_range=(10.0, 10.0),
        #     rel_standing_envs=0.00,
        #     rel_heading_envs=1.0,
        #     heading_command=False,
        #     heading_control_stiffness=0.5,
        #     debug_vis=True,
        #     ranges=mdp.UniformVelocityCommandCfg.Ranges(
        #         lin_vel_x=(0.4, 0.6), lin_vel_y=(-0.1, 0.1), ang_vel_z=(-0.1, 0.1), heading=(-math.pi, math.pi)
        #     ),
        # )


@configclass
class VelocityGo2WalkFlatEnvCfg_Ros(VelocityGo2WalkFlatEnvCfg_Play):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # Define socket command values (updated when socket receives "lin_x,lin_y,ang_z")
        socket_vx = 0.0  # Default, socket will update this
        socket_vy = 0.0  # Default, socket will update this
        socket_wz = 0.0  # Default, socket will update this
        self.scene.num_envs = 1
        # Socket velocity command - inherits UniformVelocityCommand behavior
        # When socket receives "0.5,0.0,0.0", it updates:
        #   socket_vx=0.5, socket_vy=0.0, socket_wz=0.0
        # And ranges become: lin_vel_x=(0.5,0.5), lin_vel_y=(0.0,0.0), ang_vel_z=(0.0,0.0)
        self.commands.base_velocity = SocketVelocityCommandCfg(
            asset_name="robot",
            port=5555,
            resampling_time_range=(5.0, 5.0),  # Match training behavior (10s), but with noise need faster updates
            rel_standing_envs=0.00,
            rel_heading_envs=1.0,
            heading_command=False,              # 使用角速度模式（直接控制）
            heading_control_stiffness=0.5,
            debug_vis=True,
            socket_vx=socket_vx,
            socket_vy=socket_vy,
            socket_wz=socket_wz,
            add_command_noise=True,           # Enable noise for policy compatibility
            noise_scale=0.25,                # Increased noise magnitude (±0.25 m/s)
            use_sinusoidal_variation=False,    # Use random noise
            ranges=SocketVelocityCommandCfg.Ranges(
                lin_vel_x=(socket_vx, socket_vx),
                lin_vel_y=(socket_vy, socket_vy),
                ang_vel_z=(socket_wz, socket_wz),
                heading=(-math.pi, math.pi),
            ),
        )
