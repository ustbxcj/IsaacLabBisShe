# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##


gym.register(
    id="Template-Naviation-Test-Unitree-Go2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.naviation_test_env_cfg:LocomotionNaviationTestEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:NaviationTestPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_nav_test_ppo_cfg.yaml",
    },
)


gym.register(
    id="Template-Naviation-Test-Unitree-Go2-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.naviation_test_env_cfg:LocomotionNaviationTestEnvCfg_Play",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:NaviationTestPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_nav_test_ppo_cfg.yaml",
    },
)


gym.register(
    id="Template-Naviation-Fun-Unitree-Go2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.naviation_fun_env_cfg:NaviFunTaskEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_fun_cfg:NaviationFunPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_nav_fun_ppo_cfg.yaml",
    },
)


gym.register(
    id="Template-Naviation-Fun-Unitree-Go2-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.naviation_fun_env_cfg:NaviFunTaskEnvCfg_Play",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_fun_cfg:NaviationFunPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_nav_fun_ppo_cfg.yaml",
    },
)
