# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Socket velocity command configuration."""

from isaaclab.envs.mdp.commands.commands_cfg import UniformVelocityCommandCfg
from isaaclab.utils import configclass
from .socket_velocity_command import SocketVelocityCommand


@configclass
class SocketVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = SocketVelocityCommand
    port: int = 5555

    # Socket command values (updated when socket receives commands)
    socket_vx: float = 0.0
    socket_vy: float = 0.0
    socket_wz: float = 0.0
