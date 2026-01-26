#!/usr/bin/env python3
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
Play script with ROS2 bridge support for Isaac Lab

This script extends the standard play.py to support ROS2 velocity commands
via the shared memory bridge.

Usage:
    # Terminal 1: Start ROS2 subscriber
    python ros2_velocity_subscriber.py

    # Terminal 2: Start ROS2 publisher (manual control)
    python ros2_velocity_publisher.py --mode manual

    # Terminal 3: Run this script
    python play_ros2.py --task Template-Velocity-Test-Unitree-Go2-Play-v0 \
        --checkpoint /path/to/checkpoint.pt
"""

import argparse
import sys
import os

# Get the directory paths
ros2_bridge_path = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(ros2_bridge_path)
scripts_rsl_rl_dir = os.path.join(project_dir, "scripts", "rsl_rl")

# Add necessary directories to Python path
for path in [ros2_bridge_path, scripts_rsl_rl_dir]:
    if path not in sys.path:
        sys.path.insert(0, path)

from isaaclab.app import AppLauncher

# local imports (cli_args is in scripts/rsl_rl/)
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Play with ROS2 bridge support")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument(
    "--enable_ros2",
    action="store_true",
    default=True,
    help="Enable ROS2 command bridge (default: True).",
)

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import time
import torch
import numpy as np
import multiprocessing.shared_memory

from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import MyProject.tasks  # noqa: F401


class Ros2VelocityBridge:
    """Bridge to read ROS2 velocity commands from shared memory."""

    SHM_NAME = "ros2_velocity_command"
    SHM_SIZE = 256

    def __init__(self):
        """Initialize the shared memory bridge."""
        try:
            self.shm = multiprocessing.shared_memory.SharedMemory(
                name=self.SHM_NAME,
                create=False
            )
            self.enabled = True
            print("[ROS2 Bridge] Successfully connected to shared memory")
        except FileNotFoundError:
            print("[ROS2 Bridge] WARNING: Shared memory not found.")
            print("[ROS2 Bridge] Make sure ros2_velocity_subscriber.py is running!")
            self.enabled = False
            self.shm = None

    def read_command(self):
        """Read velocity command from shared memory."""
        if not self.enabled or self.shm is None:
            return None

        try:
            shm_buf = np.frombuffer(self.shm.buf, dtype=np.float64)
            return {
                'lin_vel_x': float(shm_buf[0]),
                'lin_vel_y': float(shm_buf[1]),
                'ang_vel_z': float(shm_buf[2]),
            }
        except Exception as e:
            print(f"[ROS2 Bridge] Error reading command: {e}")
            return None

    def close(self):
        """Close the shared memory connection."""
        if self.shm is not None:
            self.shm.close()


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent and ROS2 bridge."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # set the environment seed
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # extract the neural network module
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    # extract the normalizer
    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt

    # Initialize ROS2 bridge
    ros2_bridge = None
    if args_cli.enable_ros2:
        try:
            ros2_bridge = Ros2VelocityBridge()
            if ros2_bridge.enabled:
                print("[INFO] ROS2 bridge enabled - velocity commands from /cmd_vel topic")
            else:
                print("[WARNING] ROS2 bridge disabled - using default commands")
        except Exception as e:
            print(f"[WARNING] Failed to initialize ROS2 bridge: {e}")
            print("[WARNING] Running in standard mode without ROS2")
            ros2_bridge = None

    # reset environment
    obs = env.get_observations()
    timestep = 0

    # Try to get command term once before the loop
    env_unwrapped = env.unwrapped
    cmd_term = None
    if hasattr(env_unwrapped, 'command_manager'):
        cmd_manager = env_unwrapped.command_manager

        # Try to get the term using get_term method
        try:
            cmd_term = cmd_manager.get_term('base_velocity')
            print("[INFO] Successfully accessed base_velocity command using get_term()")
            print(f"[DEBUG] cmd_term.command shape: {cmd_term.command.shape if hasattr(cmd_term.command, 'shape') else 'N/A'}")
        except Exception as e:
            print(f"[WARNING] Could not get base_velocity term: {e}")
            print(f"[WARNING] Active terms: {cmd_manager.active_terms}")
            cmd_term = None
    else:
        print("[WARNING] Environment has no command_manager")

    # print info
    print("="*60)
    print("Isaac Sim with ROS2 Bridge Started")
    print("="*60)
    if ros2_bridge and ros2_bridge.enabled:
        print("ROS2 Bridge: ENABLED")
        print("  - Robot will ONLY respond to ROS2 /cmd_vel commands")
        print("  - Control methods:")
        print("    1. ros2_velocity_publisher.py (keyboard: w/a/s/d/q/e)")
        print("    2. Your custom ROS2 node publishing to /cmd_vel")
    else:
        print("ROS2 Bridge: DISABLED")
        print("  - Robot will stay stationary (zero velocity)")
        print("  - To enable ROS2 control, start ros2_velocity_subscriber.py")
    print("="*60)

    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()

        # Override velocity commands only if cmd_term is available
        if cmd_term is not None:
            # Determine which command to use
            if ros2_bridge and ros2_bridge.enabled:
                # Try to read ROS2 command
                ros2_cmd = ros2_bridge.read_command()
                if ros2_cmd is not None:
                    # Use ROS2 command
                    lin_vel_x = ros2_cmd['lin_vel_x']
                    lin_vel_y = ros2_cmd['lin_vel_y']
                    ang_vel_z = ros2_cmd['ang_vel_z']
                else:
                    # ROS2 enabled but no data yet - use zero velocity
                    lin_vel_x = 0.0
                    lin_vel_y = 0.0
                    ang_vel_z = 0.0
            else:
                # ROS2 not enabled - use zero velocity (robot stays still)
                lin_vel_x = 0.0
                lin_vel_y = 0.0
                ang_vel_z = 0.0

            # Update the command for all environments
            num_envs = env_unwrapped.num_envs
            cmd_term.command[:, 0] = lin_vel_x
            cmd_term.command[:, 1] = lin_vel_y
            cmd_term.command[:, 2] = ang_vel_z

        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, dones, _ = env.step(actions)
            # reset recurrent states for episodes that have terminated
            policy_nn.reset(dones)
        if args_cli.video:
            timestep += 1
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    if ros2_bridge:
        ros2_bridge.close()
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
