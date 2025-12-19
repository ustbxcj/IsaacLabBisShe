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
CHECKPOINT_PATH = "/home/xcj/work/IsaacLab/BiShe/MyProject/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-16_22-29-23/model_299.pt"
OUTPUT_TS_PATH = "/home/xcj/work/IsaacLab/BiShe/MyProject/scripts/rsl_rl/logs/rsl_rl/unitree_go2_test/2025-12-16_22-29-23/model_new.pt"

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
# 6. 通过 policy 属性获取 actor
# =========================
print("runner.alg 类型:", type(runner.alg))
print("runner.alg.policy 类型:", type(runner.alg.policy))

# 检查 policy 对象的属性
if hasattr(runner.alg.policy, 'actor'):
    actor = runner.alg.policy.actor
    print("✅ 通过 runner.alg.policy.actor 获取 actor")
elif hasattr(runner.alg.policy, 'actor_critic'):
    actor = runner.alg.policy.actor_critic.actor
    print("✅ 通过 runner.alg.policy.actor_critic.actor 获取 actor")
elif hasattr(runner.alg.policy, 'actor_network'):
    actor = runner.alg.policy.actor_network
    print("✅ 通过 runner.alg.policy.actor_network 获取 actor")
else:
    # 如果以上都不行，打印 policy 的属性查看结构
    print("runner.alg.policy 的属性:", dir(runner.alg.policy))
    
    # 尝试直接使用 policy 作为 actor（在某些实现中，policy 本身就是 actor）
    actor = runner.alg.policy
    print("⚠️ 使用 runner.alg.policy 作为 actor")

actor.eval()

# =========================
# 7. 构造 dummy observation
# =========================
# 首先检查观察维度
obs_dim = None

# 尝试不同方法获取观察维度
if hasattr(actor, 'obs_dim'):
    obs_dim = actor.obs_dim
    print(f"从 actor.obs_dim 获取观察维度: {obs_dim}")
elif hasattr(runner.alg, 'obs_dim'):
    obs_dim = runner.alg.obs_dim
    print(f"从 runner.alg.obs_dim 获取观察维度: {obs_dim}")
elif hasattr(runner.alg.policy, 'obs_dim'):
    obs_dim = runner.alg.policy.obs_dim
    print(f"从 runner.alg.policy.obs_dim 获取观察维度: {obs_dim}")
else:
    # 从环境的观察空间获取
    if hasattr(env, 'observation_space'):
        obs_space = env.observation_space
        if hasattr(obs_space, 'shape'):
            obs_dim = obs_space.shape[0]
            print(f"从环境 observation_space 获取观察维度: {obs_dim}")
    else:
        # 尝试从打印的日志中推断（之前看到是48维）
        obs_dim = 48
        print(f"⚠️ 使用默认观察维度: {obs_dim}")

dummy_obs = torch.randn(1, obs_dim)
print(f"构造 dummy_obs 形状: {dummy_obs.shape}")

# =========================
# 8. 测试 actor 前向传播
# =========================
try:
    with torch.no_grad():
        test_output = actor(dummy_obs)
    print(f"✅ Actor 前向传播成功，输出形状: {test_output.shape}")
except Exception as e:
    print(f"❌ Actor 前向传播失败: {e}")
    # 尝试检查 actor 的输入参数
    print("Actor 的结构:")
    print(actor)
    print("\nActor 的 forward 方法签名:")
    import inspect
    try:
        print(inspect.signature(actor.forward))
    except:
        print("无法获取 forward 方法签名")

# =========================
# 9. TorchScript 导出
# =========================
try:
    traced_actor = torch.jit.trace(actor, dummy_obs, strict=False)
    torch.jit.save(traced_actor, OUTPUT_TS_PATH)
    print(f"✅ TorchScript policy saved to: {OUTPUT_TS_PATH}")
    
    # 测试导出的模型
    loaded_actor = torch.jit.load(OUTPUT_TS_PATH)
    test_output = loaded_actor(dummy_obs)
    print(f"✅ 模型导出成功，测试输出形状: {test_output.shape}")
    
except Exception as e:
    print(f"❌ torch.jit.trace 导出失败: {e}")
    print("尝试使用 torch.jit.script 导出...")
    
    try:
        scripted_actor = torch.jit.script(actor)
        torch.jit.save(scripted_actor, OUTPUT_TS_PATH)
        print(f"✅ 使用 torch.jit.script 导出成功: {OUTPUT_TS_PATH}")
    except Exception as e2:
        print(f"❌ torch.jit.script 也失败: {e2}")
        print("尝试备用导出方法...")
        
        # 备用方法：直接保存模型状态字典
        torch.save({
            'actor_state_dict': actor.state_dict(),
            'obs_dim': obs_dim,
            'actor_architecture': str(actor)
        }, OUTPUT_TS_PATH.replace('.pt', '_state_dict.pt'))
        print(f"✅ 保存模型状态字典到: {OUTPUT_TS_PATH.replace('.pt', '_state_dict.pt')}")

# =========================
# 10. 关闭仿真
# =========================
simulation_app.close()