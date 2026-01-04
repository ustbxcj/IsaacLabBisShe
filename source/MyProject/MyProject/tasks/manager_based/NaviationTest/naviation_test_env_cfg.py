# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

# import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp

##
# Pre-defined configs
##
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort: skip

##
# Scene definition
##

import MyProject.tasks.manager_based.NaviationTest.mdp as mdp
from MyProject.tasks.manager_based.WalkTest.walk_test_env_cfg import LocomotionVelocityTestEnvCfg
from MyProject.tasks.manager_based.NaviationTest.config.terrain import (
    EASY_PIT_TERRAINS_CFG,
    MEDIUM_PIT_TERRAINS_CFG,
    HARD_PIT_TERRAINS_CFG,
    MIXED_PIT_TERRAINS_CFG,
)

LOW_LEVEL_ENV_CFG = LocomotionVelocityTestEnvCfg()
#分层的强化学习的方式，低层的强化学习为之前已经训练好的在平地上行走的策略
#如果需要训练好的话，这个层次的策略也应该训练好一点


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    # Single difficulty: EASY_PIT_TERRAINS_CFG, MEDIUM_PIT_TERRAINS_CFG, HARD_PIT_TERRAINS_CFG
    # Mixed sampling: MIXED_PIT_TERRAINS_CFG (40% easy, 40% medium, 20% hard)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=MEDIUM_PIT_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )
    # robots
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # sensors
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    # lights
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    pose_command = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(8.0, 8.0),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(
            pos_x=(-1.5, 1.5),  # 减小范围，让目标更容易到达
            pos_y=(-1.5, 1.5),  # 减小范围
            heading=(-math.pi, math.pi)
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True)
    pre_trained_policy_action: mdp.PreTrainedPolicyActionCfg = mdp.PreTrainedPolicyActionCfg(
        asset_name="robot",
        policy_path="/home/xcj/work/IsaacLab/BiShe/MyProject/ModelBackup/Rough_Walk_policy_Transfer.pt",
        # policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/ANYmal-C/Blind/policy.pt",
        #This
        # policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/ANYmal-C/Blind/policy.pt",
        #在模型加载着一块，IsaacLab中的自带的RSL—RL训练代码的模型是checkpoint文件
        #而这个给出的示例代码则是TorchScript文件
        #在NewTools文件夹下的NewTools/model_trans.py可以转换模型
        low_level_decimation=4,
        low_level_actions=LOW_LEVEL_ENV_CFG.actions.joint_pos,
        low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        # 基础运动状态 (6维)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)  # 3维: 线速度
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)  # 3维: 角速度

        # 姿态信息 (3维)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)  # 3维: 重力投影向量

        # 目标命令 (4维)
        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "pose_command"})  # 4维: x, y, z, heading

        # 高度扫描 - 对爬台阶至关重要！
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        # 上一时刻的动作 (3维: 高层策略输出给低层的速度命令)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False  # 高层策略不添加噪声
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.0, 0.0),
                "y": (-0.0, 0.0),
                "z": (-0.0, 0.0),
                "roll": (-0.0, 0.0),
                "pitch": (-0.0, 0.0),
                "yaw": (-0.0, 0.0),
            },
        },
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP - Pit Climbing Task."""

    # ========== Primary Task Rewards (爬坑任务) ==========

    # Progress reward (reduction in distance to target) - 最重要！
    progress = RewTerm(
        func=mdp.progress_reward,
        weight=5.0,  # 增大权重，鼓励向目标移动
        params={"command_name": "pose_command"},
    )

    # Position tracking rewards (coarse and fine-grained)
    position_tracking = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=1.0,
        params={"std": 2.0, "command_name": "pose_command"},
    )

    position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.5,  # 减小权重，避免远距离时影响太大
        params={"std": 0.5, "command_name": "pose_command"},  # 增大std，更宽容
    )

    # Orientation tracking
    orientation_tracking = RewTerm(
        func=mdp.heading_command_error_abs,
        weight=-0.2,  # 减小权重，允许更多朝向探索
        params={"command_name": "pose_command"},
    )

    # Velocity aligned with target direction
    velocity_alignment = RewTerm(
        func=mdp.velocity_toward_target,
        weight=1.0,  # 增大权重
        params={"command_name": "pose_command"},
    )

    # Height climbing reward (near obstacles) - 爬坑关键！
    height_climbing = RewTerm(
        func=mdp.height_progress_near_obstacle,
        weight=10.0,  # 大幅增加权重，鼓励爬升
        params={"command_name": "pose_command", "height_threshold": 0.1, "distance_threshold": 1.5},
    )

    # ========== Regularization Penalties (减小权重，允许探索) ==========

    # Joint position deviation (from default pose)
    joint_position_penalty = RewTerm(
        func=mdp.joint_position_penalty,
        weight=-0.001,  # 减小10倍，允许更多关节运动
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )

    # Joint velocity penalty (encourages smooth motion)
    joint_velocity_penalty = RewTerm(
        func=mdp.joint_velocity_penalty,
        weight=-0.0001,  # 减小10倍
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )

    # Action rate penalty (encourages smooth control)
    action_rate_penalty = RewTerm(
        func=mdp.action_rate_penalty,
        weight=-0.001,  # 减小10倍，允许更大幅度动作
    )

    # Torque penalty (energy efficiency)
    torque_penalty = RewTerm(
        func=mdp.torque_penalty,
        weight=-0.00002,  # 减小10倍
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )

    # Body linear acceleration penalty (prevents jerky motion)
    body_acceleration_penalty = RewTerm(
        func=mdp.body_linear_acceleration_penalty,
        weight=-1.0e-7,  # 减小10倍
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base")},
    )

    # Orientation penalty (maintain upright posture)
    orientation_penalty = RewTerm(
        func=mdp.orientation_penalty,
        weight=-0.02,  # 减小5倍，允许更多姿态变化
    )

    # Vertical velocity penalty (discourages excessive jumping)
    vertical_velocity_penalty = RewTerm(
        func=mdp.vertical_lin_vel_penalty,
        weight=-0.1,  # 减小10倍！允许跳跃/攀爬动作
    )

    # Termination penalty
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-100.0)  # 减小权重


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)


##
# Environment configuration
##

@configclass
class LocomotionNaviationTestEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the navigation environment."""

    # environment settings
    # scene: SceneEntityCfg = LOW_LEVEL_ENV_CFG.scene
    scene: SceneEntityCfg = MySceneCfg(num_envs=4096, env_spacing=4)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    # mdp settings
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""

        self.sim.dt = LOW_LEVEL_ENV_CFG.sim.dt
        self.sim.render_interval = LOW_LEVEL_ENV_CFG.decimation
        self.decimation = LOW_LEVEL_ENV_CFG.decimation * 10
        self.episode_length_s = self.commands.pose_command.resampling_time_range[1]
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01  
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = (
                self.actions.pre_trained_policy_action.low_level_decimation * self.sim.dt
            )
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

class LocomotionNaviationTestEnvCfg_Play(LocomotionNaviationTestEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 4
        # disable randomization for play
        self.observations.policy.enable_corruption = False
