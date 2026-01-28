# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from isaaclab.utils import configclass

from .walk_rough_env_cfg import VelocityGo2WalkRoughEnvCfg
from .mdp import SocketVelocityCommandCfg


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


@configclass
class VelocityGo2WalkFlatEnvCfg_Ros(VelocityGo2WalkFlatEnvCfg_Play):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # Replace with socket velocity command
        # This listens on UDP port 5555 for velocity commands (format: "lin_x,lin_y,ang_z")
        self.commands.base_velocity = SocketVelocityCommandCfg(
            asset_name="robot",
            port=5555,
            heading_command=False,
            heading_control_stiffness=0.5,
            debug_vis=True,
            ranges=SocketVelocityCommandCfg.Ranges(
                lin_vel_x=(-1.0, 1.0),
                lin_vel_y=(-1.0, 1.0),
                ang_vel_z=(-1.0, 1.0),
                heading=(-math.pi, math.pi),
            ),
        )
