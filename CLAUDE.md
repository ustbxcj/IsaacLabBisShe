# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a hierarchical reinforcement learning project for Unitree Go2 quadruped robot navigation built on Isaac Lab. The system has two levels:
- **Low-level policy**: Pre-trained walking controller that handles locomotion
- **High-level policy**: Navigation controller that sends velocity/pose commands to the low-level policy

本课程设计项目是基于 Isaac Lab 的 Unitree Go2 四足机器人分层强化学习导航系统。系统包含两个层次：
- **低层策略**：预训练的行走控制器，负责基本运动
- **高层策略**：导航控制器，向低层策略发送速度/位姿命令

## Key Commands / 关键命令

### Installation / 安装
```bash
python -m pip install -e source/MyProject
```

### Training Workflow / 训练流程

#### 1. Train low-level walking policy / 训练低层行走策略
```bash
python scripts/rsl_rl/train.py --task Template-Velocity-Go2-Walk-Flat-v0 --headless
```

#### 2. Convert checkpoint to TorchScript format / 将检查点转换为 TorchScript 格式
The training produces a checkpoint file, but the navigation task requires TorchScript format.
训练产生的检查点文件需要转换为 TorchScript 格式才能用于导航任务。

Edit `NewTools/model_trans.py` to set:
编辑 `NewTools/model_trans.py` 设置：
- `CHECKPOINT_PATH`: Path to your trained checkpoint（训练模型的路径）
- `OUTPUT_TS_PATH`: Desired output path for TorchScript model（TorchScript 模型输出路径）

Then run:
然后运行：
```bash
python NewTools/model_trans.py
```

#### 3. Update navigation config with converted model / 使用转换后的模型更新导航配置
Edit `source/MyProject/MyProject/tasks/manager_based/NaviationTest/naviation_rough_env_cfg.py`:
编辑文件，在 `ActionsCfg.pre_trained_policy_action` 中更新 `policy_path` 为你的 TorchScript 模型路径。

#### 4. Train high-level navigation policy / 训练高层导航策略
```bash
python scripts/rsl_rl/train.py --task Template-Naviation-Rough-Go2-v0 --headless
```

### Play/Inference / 推理和可视化
```bash
# Walking policy / 行走策略
python scripts/rsl_rl/play.py --task Template-Velocity-Go2-Walk-Flat-Play-v0

# Navigation policy / 导航策略
python scripts/rsl_rl/play.py --task Template-Naviation-Rough-Go2-Play-v0
```

## Architecture / 架构

### Task Structure / 任务结构
Tasks are located in `source/MyProject/MyProject/tasks/manager_based/`:
任务位于以下目录：

- **WalkTest/**: Low-level locomotion task（低层运动任务）
  - `walk_flat_env_cfg.py`: Flat terrain configuration（平坦地形配置）
  - `walk_rough_env_cfg.py`: Rough terrain configuration（粗糙地形配置）
  - Inherits from Isaac Lab's official Go2 configuration with custom modifications（继承自 Isaac Lab 官方 Go2 配置并自定义）

- **NaviationTest/**: High-level navigation task（高层导航任务）
  - `naviation_flat_env_cfg.py`: Flat terrain navigation（平坦地形导航）
  - `naviation_rough_env_cfg.py`: Rough terrain navigation with obstacles（带障碍物的粗糙地形导航）

### Hierarchical RL Setup / 分层强化学习设置

The navigation task uses `PreTrainedPolicyAction` which:
导航任务使用 `PreTrainedPolicyAction`，其功能：

1. Takes high-level commands (target position, heading) as inputs（接受高层命令：目标位置和航向）
2. Outputs low-level velocity commands (vx, vy, ωz)（输出低层速度命令）
3. Passes these to the pre-trained walking policy（传递给预训练的行走策略）
4. The walking policy outputs joint positions to the robot（行走策略输出关节位置给机器人）

Key file: `NaviationTest/mdp/pre_trained_policy_action.py`
关键文件：`NaviationTest/mdp/pre_trained_policy_action.py`

### Environment Registration / 环境注册

Each task registers multiple Gym environment variants:
每个任务注册多个 Gym 环境变体：

- `-v0`: Training configuration（训练配置，4096 environments）
- `-Play-v0`: Inference/visualization with fewer environments（推理/可视化配置，50 environments）
- `-Ros-v0`: ROS-compatible version（ROS 兼容版本）

Registered in `__init__.py` files in each task directory.
在每个任务的 `__init__.py` 文件中注册。

Available tasks:
可用任务：
- `Template-Velocity-Go2-Walk-Flat-v0`: Walking on flat terrain（平地行走）
- `Template-Velocity-Go2-Walk-Rough-v0`: Walking on rough terrain（粗糙地形行走）
- `Template-Naviation-Flat-Go2-v0`: Navigation on flat terrain（平地导航）
- `Template-Naviation-Rough-Go2-v0`: Navigation on rough terrain with obstacles（带障碍物的粗糙地形导航）

### MDP Components / MDP 组件

Each task defines MDP components in its config file:
每个任务在其配置文件中定义 MDP 组件：

```python
@configclass
class TaskEnvCfg(ManagerBasedRLEnvCfg):
    scene: SceneCfg              # Scene and assets（场景和物体）
    actions: ActionsCfg          # Action specifications（动作规范）
    observations: ObservationsCfg # Sensory inputs（传感器输入）
    commands: CommandsCfg        # Target generation（目标生成）
    rewards: RewardsCfg          # Reward functions（奖励函数）
    terminations: TerminationsCfg # Episode end conditions（回合结束条件）
    events: EventCfg            # Randomization and resets（随机化和重置）
    curriculum: CurriculumCfg   # Difficulty progression（难度递进）
```

#### Commands / 命令
- **WalkTest**: `UniformVelocityCommandCfg` - Generates velocity targets (vx, vy, ωz)（生成速度目标）
- **NaviationTest**: `UniformPose2dCommandCfg` - Generates position and heading targets（生成位置和航向目标）

#### Actions / 动作
- **WalkTest**: Direct joint position control（直接关节位置控制）
- **NaviationTest**: `PreTrainedPolicyActionCfg` - Hierarchical action wrapper（分层动作包装器）

#### Observations / 观测空间

**Low-level policy (WalkTest):** ~200 dimensions including ~200 维，包括：
- Base linear/angular velocity（基座线速度/角速度）
- Projected gravity（重力投影）
- Velocity commands（速度命令）
- Joint positions/velocities（关节位置/速度）
- Height scan（地形高度扫描）
- Actions（动作）

**High-level policy (NaviationTest):** 176 dimensions including 176 维，包括：
- Base linear/angular velocity（基座线速度/角速度）
- Projected gravity（重力投影）
- Pose commands (target position/heading)（位姿命令：目标位置/航向）
- Height scan (critical for climbing!)（高度扫描：爬高台的关键！）
- Last action（上一步动作）

### Reward Functions / 奖励函数

Navigation rewards include sophisticated functions based on research paper:
导航奖励包含基于研究论文的复杂函数：

**"Hierarchical Reinforcement Learning for Agile Quadrupedal Locomotion with Demonstrations"**

Key reward terms:
主要奖励项：
- `climb_progress_reward`: Progress toward obstacle using tanh kernel（使用 tanh 核的障碍物进展奖励）
- `position_command_error_tanh`: Position tracking with coarse/fine variants（位置跟踪，粗粒度/细粒度）
- `heading_command_error_abs`: Heading tracking（航向跟踪）
- Regularization penalties: joint positions, velocities, torques, orientation, etc.（正则化惩罚：关节位置、速度、力矩、姿态等）

Configuration in `naviation_rough_env_cfg.py` under `RewardsCfg` class.
配置在 `naviation_rough_env_cfg.py` 的 `RewardsCfg` 类中。

### Terrain Configuration / 地形配置

The project uses Isaac Lab's terrain system:
项目使用 Isaac Lab 地形系统：

- **Flat terrain**: Uses `terrain_type="plane"`（平地：使用平面地形类型）
- **Rough terrain**: Uses `ROUGH_TERRAINS_CFG` with customizable sub-terrains（粗糙地形：使用可配置子地形的粗糙地形配置）

Custom terrain modifications in `walk_rough_env_cfg.py`:
自定义地形修改：
```python
# Scale down terrain for smaller robot（为更小的机器人缩放地形）
scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01
```

Navigation task adds platform obstacles for climbing:
导航任务添加平台障碍物用于爬高训练：
```python
platform = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Platform",
    spawn=sim_utils.CuboidCfg(size=(1.0, 1.0, 0.26)),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(2.0, 0.0, 0.13)),
)
```

### Configuration Inheritance / 配置继承

The project follows Isaac Lab's inheritance pattern:
项目遵循 Isaac Lab 的继承模式：

1. **Rough terrain** is the base configuration（粗糙地形是基础配置）
2. **Flat terrain** inherits from rough and overrides specific settings（平坦地形继承自粗糙地形并覆盖特定设置）
3. **Play configs** inherit from training configs and reduce environments（Play 配置继承自训练配置并减少环境数）

Example:
示例：
```python
@configclass
class VelocityGo2WalkFlatEnvCfg(VelocityGo2WalkRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Override specific settings（覆盖特定设置）
        self.scene.terrain.terrain_type = "plane"
        self.rewards.flat_orientation_l2.weight = -2.5
```

## Model Conversion / 模型转换

The `NewTools/model_trans.py` script:
`NewTools/model_trans.py` 脚本：

1. Loads a checkpoint from RSL-RL training（从 RSL-RL 训练加载检查点）
2. Extracts the actor network（提取 actor 网络）
3. Traces it with `torch.jit.trace()` using dummy observations（使用虚拟观测进行追踪）
4. Saves as TorchScript `.pt` file（保存为 TorchScript 文件）

**Critical requirement**: The hierarchical action manager requires TorchScript models.
**关键要求**：分层动作管理器需要 TorchScript 格式的模型。

When updating the policy path in navigation config:
在导航配置中更新策略路径时：
1. Train and convert walking policy（训练并转换行走策略）
2. Update `policy_path` in `naviation_rough_env_cfg.py`（更新 `naviation_rough_env_cfg.py` 中的 `policy_path`）
3. Train navigation policy（训练导航策略）

## Important Notes / 重要说明

- The low-level policy must be trained first and converted before training navigation（低层策略必须先训练并转换后才能训练导航）
- Model paths in config files use absolute paths - update them for your system（配置文件中的模型路径使用绝对路径 - 需要根据系统更新）
- The decimation factor in navigation is typically higher than in low-level policy（导航中的 decimation 因素通常高于低层策略）
- Terrain configuration uses `ROUGH_TERRAINS_CFG` for varied obstacle training（地形配置使用 `ROUGH_TERRAINS_CFG` 进行多样化的障碍物训练）
- Height scanner provides critical terrain perception for both policies（高度扫描器为两种策略提供关键的地形感知）

## File Structure / 文件结构

```
IsaacLabBisShe/
├── source/MyProject/          # Main project code（主要项目代码）
│   └── MyProject/tasks/manager_based/
│       ├── WalkTest/          # Low-level walking（低层行走）
│       │   ├── walk_flat_env_cfg.py
│       │   ├── walk_rough_env_cfg.py
│       │   └── agents/        # PPO configurations（PPO 配置）
│       └── NaviationTest/     # High-level navigation（高层导航）
│           ├── naviation_flat_env_cfg.py
│           ├── naviation_rough_env_cfg.py
│           ├── mdp/           # Custom reward functions（自定义奖励函数）
│           │   └── pre_trained_policy_action.py
│           └── agents/        # PPO configurations（PPO 配置）
├── scripts/rsl_rl/            # Training scripts（训练脚本）
│   ├── train.py
│   └── play.py
├── NewTools/
│   └── model_trans.py         # Model conversion tool（模型转换工具）
└── ModelBackup/               # Trained models（训练好的模型）
    ├── WalkPolicy/
    ├── TransPolicy/
    └── NaviationPolicy/
```

## Custom Modifications / 自定义修改

The project includes several custom modifications compared to official Isaac Lab configs:
与官方 Isaac Lab 配置相比，本项目包含以下自定义修改：

1. **Terrain scaling**: Adjusted box heights and noise ranges for Go2's smaller size（地形缩放：为 Go2 更小的尺寸调整了盒子高度和噪声范围）
2. **Disabled events**: Turned off push_robot and base_com randomization for stability（禁用事件：为稳定性关闭了 push_robot 和 base_com 随机化）
3. **Custom rewards**: Navigation rewards for climbing obstacles（自定义奖励：用于爬越障碍物的导航奖励）
4. **Hierarchical interface**: PreTrainedPolicyAction for two-level control（分层接口：用于两级控制的 PreTrainedPolicyAction）
