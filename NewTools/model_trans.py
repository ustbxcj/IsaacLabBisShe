import torch
import gymnasium as gym
import sys
import os

# =========================
# 0. 添加项目根目录到 Python 路径
# =========================
# 获取当前文件的目录（/home/xcj/work/IsaacLab/BiShe/MyProject/NewTools）
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取项目根目录（/home/xcj/work/IsaacLab/BiShe/MyProject）
project_root = os.path.dirname(os.path.dirname(current_dir))
# 将项目根目录添加到 sys.path
sys.path.insert(0, project_root)

# =========================
# 1. 启动 Isaac Sim（必须在导入任何 IsaacLab 相关模块之前）
# =========================
from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# =========================
# 2. 现在可以安全导入 IsaacLab 相关模块
# =========================
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

# =========================
# 3. 导入你的 task / cfg（使用正确的相对路径）
# =========================
# 注意：现在 source 目录已经在 sys.path 中
from MyProject.tasks.manager_based.WalkTest.walk_test_env_cfg import (
    LocomotionVelocityTestEnvCfg,
)
from MyProject.tasks.manager_based.WalkTest.agents.rsl_rl_ppo_cfg import UnitreeGo2TestPPORunnerCfg

TASK_NAME = "Template-Velocity-Test-Unitree-Go2-v0"
CHECKPOINT_PATH = "/home/xcj/work/BiShe/IsaacLabBisShe/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-19_13-50-31/policy.pt"
OUTPUT_TS_PATH = "/home/xcj/work/BiShe/IsaacLabBisShe/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-19_13-50-31/policynew.pt"

# =========================
# 4. 构建 env 和 agent cfg
# =========================
env_cfg = LocomotionVelocityTestEnvCfg()
agent_cfg = UnitreeGo2TestPPORunnerCfg()

env = gym.make(TASK_NAME, cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

# =========================
# 5. 创建 runner 并加载模型
# =========================
runner = OnPolicyRunner(
    env,
    agent_cfg.to_dict(),
    device="cpu",      # 导出用 CPU 更稳
)

runner.load(CHECKPOINT_PATH)

# =========================
# 6. 拿到 actor
# =========================
actor = runner.alg.actor_critic.actor
actor.eval()

# =========================
# 7. 构造 dummy observation
# =========================
obs_dim = actor.obs_dim
dummy_obs = torch.randn(1, obs_dim)

# =========================
# 8. TorchScript 导出
# =========================
traced_actor = torch.jit.trace(actor, dummy_obs, strict=False)
torch.jit.save(traced_actor, OUTPUT_TS_PATH)

print(f"✅ TorchScript policy saved to: {OUTPUT_TS_PATH}")

# =========================
# 9. 关闭仿真
# =========================
simulation_app.close()