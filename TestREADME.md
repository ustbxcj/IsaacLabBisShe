Use this project 
1. git clone https://github.com/ustbxcj/IsaacLabBisShe
2. enter the folder that IsaacLabBisShe, 
   And the input the order 
   'python -m pip install -e source/MyProject'
3. first train a policy to walk  and the order is that 
   'python train.py --task Template-Velocity-Test-Unitree-Go2-v0 --headless'
   And then we will get a low_policy to walk(I called it as p1)
4. This policy cannot use directly because it is the format of "checkpoint" ,
   The demo program to naviation needs a format called "torchscript"
   use the model_trans.py to transfer then generate a ploicy called p2
5. Use the p2 to modify the path in the naviation_test_env_cfg.py
6. Train the final policy follow as this order
   ‘python train.py --task’

   # CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a hierarchical reinforcement learning project for Unitree Go2 quadruped robot navigation built on Isaac Lab. The system has two levels:
- **Low-level policy**: Pre-trained walking controller that handles locomotion
- **High-level policy**: Navigation controller that sends velocity/pose commands to the low-level policy

## Key Commands

### Installation
```bash
python -m pip install -e source/MyProject
```

### Training Workflow

1. **Train low-level walking policy:**
```bash
python scripts/rsl_rl/train.py --task Template-Velocity-Test-Unitree-Go2-v0 --headless
```

2. **Convert checkpoint to TorchScript format:**
The training produces a checkpoint file, but the navigation task requires TorchScript format. Edit `NewTools/model_trans.py` to set:
- `CHECKPOINT_PATH`: Path to your trained checkpoint
- `OUTPUT_TS_PATH`: Desired output path for TorchScript model

Then run:
```bash
python NewTools/model_trans.py
```

3. **Update navigation config with converted model:**
Edit `source/MyProject/MyProject/tasks/manager_based/NaviationTest/naviation_test_env_cfg.py` and update the `policy_path` in `ActionsCfg.pre_trained_policy_action` to point to your TorchScript model.

4. **Train high-level navigation policy:**
```bash
python scripts/rsl_rl/train.py --task Template-Naviation-Test-Unitree-Go2-v0 --headless
```

### Play/Inference
```bash
python scripts/rsl_rl/play.py --task Template-Velocity-Test-Unitree-Go2-Play-v0
python scripts/rsl_rl/play.py --task Template-Naviation-Test-Unitree-Go2-Play-v0
```

## Architecture

### Task Structure
Tasks are located in `source/MyProject/MyProject/tasks/manager_based/`:
- **WalkTest/**: Low-level locomotion task (velocity tracking on rough terrain)
- **NaviationTest/**: High-level navigation task (pose/position tracking with obstacle climbing)

### Hierarchical RL Setup
The navigation task (`NaviationTest`) uses `PreTrainedPolicyActionCfg` which:
1. Takes high-level commands (target position, heading) as observations
2. Outputs low-level velocity commands
3. Passes these to the pre-trained walking policy
4. The walking policy outputs joint positions to the robot

Key file: `source/MyProject/MyProject/tasks/manager_based/NaviationTest/mdp/pre_trained_policy_action.py`

### MDP Components
Each task defines MDP components in its config file (e.g., `naviation_test_env_cfg.py`):
- **Commands**: Target generation (e.g., `UniformPose2dCommandCfg`)
- **Actions**: Either joint positions or hierarchical policy actions
- **Observations**: Sensor data, proprioception, commands
- **Rewards**: Defined in `mdp/rewards.py`
- **Terminations**: Episode end conditions in `mdp/terminations.py`
- **Events**: Randomization and resets

### 观测空间配置 (Observation Space)

#### 高层策略观测（NaviationTest）
高层策略需要丰富的环境感知信息来学习爬高台技能：

| 观测项 | 维度 | 说明 |
|--------|------|------|
| base_lin_vel | 3 | 机器人线速度 (vx, vy, vz) |
| base_ang_vel | 3 | 机器人角速度 (ωx, ωy, ωz) |
| projected_gravity | 3 | 重力投影向量（姿态信息） |
| pose_command | 4 | 目标位置和航向 (x, y, z, heading) |
| height_scan | 160 | 前方地形高度扫描（爬高台关键！） |
| last_action | 3 | 上一时刻动作 (vx_cmd, vy_cmd, ωz_cmd) |
| **总计** | **176** | 观测空间维度 |

**关键观测说明**：
- **height_scan**（最重要）：使用16×10网格射线扫描，让策略"看到"前方高台
- **base_ang_vel**：提供旋转状态信息，对姿态调整至关重要
- **last_action**：帮助策略输出平滑的动作

配置位置：`naviation_test_env_cfg.py` -> `ObservationsCfg.PolicyCfg`

#### 低层策略观测（WalkTest）
低层策略控制关节运动，需要更详细的本体感知信息：

| 观测项 | 维度 | 说明 |
|--------|------|------|
| base_lin_vel | 3 | 线速度 |
| base_ang_vel | 3 | 角速度 |
| projected_gravity | 3 | 重力投影 |
| velocity_commands | 3 | 速度命令 |
| joint_pos | 12 | 关节位置相对值 |
| joint_vel | 12 | 关节速度相对值 |
| actions | 12 | 上一步关节动作 |
| height_scan | 160 | 地形高度扫描 |
| **总计** | **~200** | 观测空间维度 |

### Reward Functions
Navigation rewards (`NaviationTest/mdp/rewards.py`):

#### 主要任务奖励 (Primary Task Rewards)
- `climb_progress_reward`: **爬台阶进展奖励** - 基于论文实现的tanh内核奖励函数
  - 公式: `tanh((max_forward_dist - current_dist) / sigma)`
  - 奖励机器人向高台目标的进展，范围[-1, 1]
  - 参数: `max_forward_distance=2.0m`, `sigma=0.25`

- `position_command_error_tanh`: **位置跟踪奖励** - 使用tanh内核
  - 粗粒度版本 (std=2.0): 大范围位置跟踪
  - 细粒度版本 (std=0.2): 接近目标时的精细跟踪

- `heading_command_error_abs`: **航向跟踪奖励** - 姿态角度跟踪

- `progress_reward`: **进展奖励** - 奖励到目标距离的减少

- `velocity_toward_target`: **速度对齐奖励** - 奖励朝向目标方向的速度

- `height_progress_near_obstacle`: **高度爬升奖励** - 在障碍物附近奖励高度增益

#### 正则化惩罚项 (Regularization Penalties)
基于论文添加的惩罚项，用于引导机器人学习自然、稳定的动作：

- `joint_position_penalty`: **关节位置惩罚** - 惩罚偏离默认关节配置
  - 权重: -0.01
  - 保持机器人姿态自然

- `joint_velocity_penalty`: **关节速度惩罚** - 惩罚过大的关节速度
  - 权重: -0.001
  - 鼓励平滑运动

- `action_rate_penalty`: **动作率惩罚** - 惩罚动作变化率
  - 权重: -0.01
  - 鼓励平滑控制，避免抖动

- `torque_penalty`: **力矩惩罚** - 惩罚过大的关节力矩
  - 权重: -0.0002
  - 促进能效优化

- `body_linear_acceleration_penalty`: **身体加速度惩罚** - 惩罚身体线加速度
  - 权重: -1.0e-6
  - 防止剧烈运动

- `orientation_penalty`: **姿态惩罚** - 惩罚非水平姿态（pitch和roll）
  - 权重: -0.1
  - 鼓励保持直立姿势

- `vertical_lin_vel_penalty`: **垂直速度惩罚** - 惩罚垂直方向速度
  - 权重: -1.0
  - 防止过度跳跃，鼓励稳定接地

### 奖励函数配置 (Reward Configuration)
所有奖励函数在 `naviation_test_env_cfg.py` 的 `RewardsCfg` 类中配置：

```python
@configclass
class RewardsCfg:
    # 主要任务奖励
    climb_progress = RewTerm(func=mdp.climb_progress_reward, weight=2.0)
    position_tracking = RewTerm(func=mdp.position_command_error_tanh, weight=1.0)
    height_climbing = RewTerm(func=mdp.height_progress_near_obstacle, weight=5.0)

    # 正则化惩罚
    joint_position_penalty = RewTerm(func=mdp.joint_position_penalty, weight=-0.01)
    orientation_penalty = RewTerm(func=mdp.orientation_penalty, weight=-0.1)
    vertical_velocity_penalty = RewTerm(func=mdp.vertical_lin_vel_penalty, weight=-1.0)
    # ... 更多惩罚项
```

### 训练调试建议

#### 监控指标
- **climb_progress**: 主要关注此奖励，值应该逐渐接近1.0
- **height_climbing**: 在接近高台时应该增加
- **position_tracking**: 应该随着训练稳定上升

#### 权重调整策略
1. **爬得太慢**: 增加 `climb_progress` 权重 (当前2.0 → 3.0-5.0)
2. **容易翻倒**: 增加 `orientation_penalty` 权重 (当前-0.1 → -0.5)
3. **动作剧烈**: 增加 `action_rate_penalty` 权重 (当前-0.01 → -0.05)
4. **姿势不自然**: 增加 `joint_position_penalty` 权重 (当前-0.01 → -0.05)
5. **过度跳跃**: 增加 `vertical_velocity_penalty` 权重 (当前-1.0 → -2.0)

#### 论文参考
奖励函数设计基于论文:
"Hierarchical Reinforcement Learning for Agile Quadrupedal Locomotion with Demonstrations"

关键公式:
- 进展奖励: `tanh((max_dist - current_dist) / σ)`
- 速度跟踪: `exp(-||v_cmd - v_actual||² / std²)`
- 角速度跟踪: `exp(-(ω_cmd - ω_actual)² / std²)`

### Environment Registration
Tasks are registered as Gym environments in `__init__.py` files:
- `Template-Velocity-Test-Unitree-Go2-v0`: Walking training
- `Template-Naviation-Test-Unitree-Go2-v0`: Navigation training
- Add `-Play-v0` suffix for inference/visualization

## Model Conversion Details

The `NewTools/model_trans.py` script:
1. Loads a checkpoint from RSL-RL training
2. Extracts the actor network
3. Traces it with `torch.jit.trace()` using dummy observations
4. Saves as TorchScript `.pt` file

This is necessary because the hierarchical action manager expects TorchScript models for the low-level policy.

## Important Notes

- The low-level policy must be trained first and converted before training navigation
- Model paths in config files use absolute paths - update them for your system
- The decimation factor in navigation is 10x the low-level decimation (see `__post_init__` in env configs)
- Terrain configuration uses `ROUGH_TERRAINS_CFG` for varied obstacle training
