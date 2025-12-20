# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass


from MyProject.tasks.manager_based.NaviationTest.naviation_test_env_cfg import LocomotionNaviationTestEnvCfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort: skip


@configclass
class NaviFunTaskEnvCfg(LocomotionNaviationTestEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()


@configclass
class NaviFunTaskEnvCfg_Play(NaviFunTaskEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
