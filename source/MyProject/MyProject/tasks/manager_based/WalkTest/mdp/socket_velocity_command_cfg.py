# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for socket velocity command."""

import math
from dataclasses import MISSING

from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass

from .socket_velocity_command import SocketVelocityCommand

__all__ = ["SocketVelocityCommandCfg"]


@configclass
class SocketVelocityCommandCfg(CommandTermCfg):
    """Configuration for the socket velocity command generator."""

    class_type: type = SocketVelocityCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    port: int = 5555
    """UDP port to listen for velocity commands. Defaults to 5555."""

    heading_command: bool = False
    """Whether to use heading command or angular velocity command. Defaults to False.

    If True, the angular velocity is computed from the heading error. The heading
    should be provided in the socket command as a 4th value: "lin_x,lin_y,ang_z,heading".
    Otherwise, only uses the angular velocity directly.
    """

    heading_control_stiffness: float = 0.5
    """Scale factor to convert the heading error to angular velocity command. Defaults to 0.5.

    Only used if :attr:`heading_command` is True.
    """

    debug_vis: bool = True
    """Whether to enable debug visualization. Defaults to True."""

    rel_standing_envs: float = 0.02
    """Probability of resampling a standing environment (zero velocity). Defaults to 0.02 (2%).

    This randomly sets some environments to have zero velocity command, which is important
    for training stability. The original implementation uses this value.
    """

    @configclass
    class Ranges:
        """Ranges for clipping the velocity commands."""

        lin_vel_x: tuple[float, float] = (-1.0, 1.0)
        """Range for the linear-x velocity command (in m/s)."""

        lin_vel_y: tuple[float, float] = (-1.0, 1.0)
        """Range for the linear-y velocity command (in m/s)."""

        ang_vel_z: tuple[float, float] = (-1.0, 1.0)
        """Range for the angular-z velocity command (in rad/s)."""

        heading: tuple[float, float] = (-math.pi, math.pi)
        """Range for the heading command (in rad). Defaults to [-pi, pi].

        This parameter is only used if :attr:`heading_command` is True.
        """

    ranges: Ranges = MISSING
    """Ranges for clipping the velocity commands."""

    def __post_init__(self):
        """Post initialization."""
        # Set the resampling time range to 10 seconds (matches original implementation)
        # This ensures standing environments are periodically refreshed for stability
        # Socket commands are still updated every step in _update_command()
        self.resampling_time_range = (10.0, 10.0)
