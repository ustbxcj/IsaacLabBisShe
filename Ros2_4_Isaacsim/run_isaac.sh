#!/bin/bash
# Simple script to run Isaac Sim with ROS2 bridge
# Usage: ./run_isaac.sh

# Configuration
PROJECT_DIR="/home/xcj/work/IsaacLab/BiShe/MyProject"
CHECKPOINT="${PROJECT_DIR}/ModelBackup/Rough_Walk_policy4Mujoco.pt"
TASK="Template-Velocity-Test-Unitree-Go2-Play-v0"
NUM_ENVS=1

# Activate IsaacLab environment
echo "Activating env_isaaclab..."
eval "$(conda shell.bash hook 2>/dev/null)"
conda activate env_isaaclab

# Change to correct directory
cd "${PROJECT_DIR}/scripts/rsl_rl"

# Run Isaac Sim (single line, no backslashes)
python ../../Ros2_4_Isaacsim/play_ros2.py --task "$TASK" --checkpoint "$CHECKPOINT" --num_envs "$NUM_ENVS"
