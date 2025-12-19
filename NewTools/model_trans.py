import torch
import gymnasium as gym

from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

from isaaclab.app import AppLauncher

# =========================
# 1. 启动 Isaac Sim（必须）
# =========================
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# =========================
# 2. 导入你的 task / cfg
# =========================
# ⚠️ 根据你工程实际路径修改
from source.MyProject.MyProject.tasks.manager_based.WalkTest.walk_test_env_cfg import (
    LocomotionVelocityTestEnvCfg,
)
from source.MyProject.MyProject.tasks.manager_based.WalkTest.agents.rsl_rl_ppo_cfg import UnitreeGo2TestPPORunnerCfg

TASK_NAME = "Template-Velocity-Test-Unitree-Go2-v0"
CHECKPOINT_PATH = "/home/robot/work/BiShe/IsaacLabBisShe/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-19_13-50-31/policy.pt"
OUTPUT_TS_PATH = "/home/robot/work/BiShe/IsaacLabBisShe/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-19_13-50-31/policynew.pt"

# =========================
# 3. 构建 env 和 agent cfg
# =========================
env_cfg = LocomotionVelocityTestEnvCfg()
agent_cfg = UnitreeGo2TestPPORunnerCfg()

env = gym.make(TASK_NAME, cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

# =========================
# 4. 创建 runner 并加载模型
# =========================
runner = OnPolicyRunner(
    env,
    agent_cfg.to_dict(),
    device="cpu",      # 导出用 CPU 更稳
)

runner.load(CHECKPOINT_PATH)

# =========================
# 5. 拿到 actor
# =========================
actor = runner.alg.actor_critic.actor
actor.eval()

# =========================
# 6. 构造 dummy observation
# =========================
obs_dim = actor.obs_dim
dummy_obs = torch.randn(1, obs_dim)

# =========================
# 7. TorchScript 导出
# =========================
traced_actor = torch.jit.trace(actor, dummy_obs, strict=False)
torch.jit.save(traced_actor, OUTPUT_TS_PATH)

print(f"✅ TorchScript policy saved to: {OUTPUT_TS_PATH}")

# =========================
# 8. 关闭仿真
# =========================
simulation_app.close()