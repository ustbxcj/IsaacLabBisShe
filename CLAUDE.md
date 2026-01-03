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

### Reward Functions
Navigation rewards (`NaviationTest/mdp/rewards.py`):
- `position_command_error_tanh`: Position tracking with tanh kernel
- `heading_command_error_abs`: Orientation tracking
- `progress_reward`: Rewards reducing distance to target
- `velocity_toward_target`: Rewards velocity aligned with target direction
- `height_progress_near_obstacle`: Rewards climbing behavior near obstacles

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
