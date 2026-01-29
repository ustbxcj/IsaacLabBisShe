# Socket 命令通信实现说明

## 目录

1. [概述](#概述)
2. [设计思路](#设计思路)
3. [核心实现原理](#核心实现原理)
4. [文件结构](#文件结构)
5. [工作流程](#工作流程)
6. [配置说明](#配置说明)
7. [使用方法](#使用方法)
8. [噪声注入机制](#噪声注入机制)
9. [代码实现详解](#代码实现详解)

---

## 概述

本项目实现了一个基于 UDP Socket 的速度命令接收系统，用于控制 Isaac Lab 中的 Unitree Go2 四足机器人。该系统通过**继承 `UniformVelocityCommand`** 并**动态修改采样范围**来实现外部控制，相比 ROS2 方案更加简单，无外部依赖。

### 核心特性

- **继承设计**：`SocketVelocityCommand` 继承 `UniformVelocityCommand`，复用所有现有逻辑
- **范围固定**：通过将采样范围设为 `(v, v)` 来固定命令值
- **双模式**：支持角速度模式（`heading_command=False`）和航向模式（`heading_command=True`）
- **噪声注入**：支持在固定命令上添加噪声，提高策略兼容性
- **线程安全**：独立线程接收命令，不影响仿真主循环
- **无缝集成**：完全兼容 Isaac Lab 的训练和推理流程
- **调试支持**：内置 DEBUG 日志输出，方便问题诊断

### 核心思路

**问题**：`UniformVelocityCommand` 使用随机采样生成命令，如何让它使用外部 socket 命令？

**解决方案**：
1. 初始化时使用随机范围 `(-1.0, 1.0)`，机器人随机行走
2. Socket 收到命令 `"0.5,0.0,0.0"` 后，将范围改为 `(0.5, 0.5)`
3. 因为采样范围上下界相同，所以随机采样总是返回这个固定值
4. 所有环境统一执行相同的命令
5. **可选**：在固定命令上添加噪声，模拟训练时的分布

---

## 设计思路

### 继承关系

```
UniformVelocityCommand (Isaac Lab 官方)
    ↑ 继承
SocketVelocityCommand (自定义)
    ├── 添加 socket 接收功能
    ├── 添加噪声注入功能
    ├── 覆盖 _resample_command()
    └── 动态修改 cfg.ranges
```

### 为什么继承而不是重写？

1. **复用现有逻辑**：`UniformVelocityCommand` 已实现：
   - 命令采样
   - 航向控制（`heading_command=True` 时）
   - Standing environments 处理（`rel_standing_envs`）
   - 命令更新机制

2. **最小修改原则**：只需修改命令来源，其他保持不变

3. **易于维护**：官方更新 `UniformVelocityCommand` 时，自动继承新特性

### 核心机制：范围固定

```python
# 初始化：随机范围
cfg.ranges.lin_vel_x = (-1.0, 1.0)
采样结果：随机值 [-1.0, 1.0]

# Socket 收到 "0.5,0.0,0.0" 后
cfg.ranges.lin_vel_x = (0.5, 0.5)  # 固定为 0.5
采样结果：始终是 0.5（因为上下界相同）
```

### 噪声注入机制

**问题**：固定命令与训练时的随机分布不匹配，策略可能不稳定。

**解决方案**：在固定命令上添加噪声，模拟训练时的分布。

```python
# 目标命令
target_vx = 0.5

# 添加噪声
noise_x = uniform(-0.25, 0.25)
actual_vx = 0.5 + noise_x  # 范围 [0.25, 0.75]

# 每个环境独立采样
vel_command_b[env_id, 0] = actual_vx
```

**效果**：
- 命令均值接近目标值
- 命令分布更接近训练时的随机分布
- 策略表现更稳定

---

## 核心实现原理

### 1. 配置文件设计

```python
@configclass
class SocketVelocityCommandCfg(UniformVelocityCommandCfg):
    """继承 UniformVelocityCommandCfg"""
    class_type: type = SocketVelocityCommand
    port: int = 5555

    # Socket 命令值（由 socket 更新）
    socket_vx: float = 0.0
    socket_vy: float = 0.0
    socket_wz: float = 0.0

    # 噪声注入配置
    add_command_noise: bool = True          # 是否添加噪声
    noise_scale: float = 0.25             # 噪声幅度
    use_sinusoidal_variation: bool = False  # 是否使用正弦变化
    variation_amp: float = 0.03           # 正弦变化幅度
```

### 2. 命令类实现

```python
class SocketVelocityCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)  # 调用父类初始化
        self._init_socket()          # 启动 socket 接收线程

    def _resample_command(self, env_ids):
        """覆盖父类方法，添加噪声注入"""
        # 检查是否为固定命令
        if self._is_fixed_command() and self.add_command_noise:
            # 添加噪声
            noise_x = uniform(-self.noise_scale, self.noise_scale)
            self.vel_command_b[env_ids, 0] = target_vx + noise_x
        else:
            # 使用父类方法
            super()._resample_command(env_ids)

    def _receive_loop(self, sock):
        while True:
            data, _ = sock.recvfrom(1024)
            parts = data.decode().strip().split(',')
            if len(parts) >= 3:
                # 更新 socket 命令值
                self.cfg.socket_vx = float(parts[0])
                self.cfg.socket_vy = float(parts[1])
                self.cfg.socket_wz = float(parts[2])

                # 关键：修改范围来固定命令值
                self.cfg.ranges.lin_vel_x = (self.cfg.socket_vx, self.cfg.socket_vx)
                self.cfg.ranges.lin_vel_y = (self.cfg.socket_vy, self.cfg.socket_vy)
                self.cfg.ranges.ang_vel_z = (self.cfg.socket_wz, self.cfg.socket_wz)
```

### 3. 工作流程

```
┌─────────────┐
│ send_cmd.py │  发送 "0.5,0.0,0.0"
└─────────────┘
       │
       ▼ UDP
┌──────────────────────────────┐
│ SocketVelocityCommand        │
│                              │
│ _receive_loop() [线程]:       │
│   cfg.socket_vx = 0.5         │
│   cfg.ranges.lin_vel_x =      │
│     (0.5, 0.5)  ← 固定范围    │
└──────────────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ _resample_command()         │
│                              │
│ if add_command_noise:         │
│   target = 0.5               │
│   noise = uniform(-0.25, 0.25)│
│   vel = 0.5 + noise          │
│   结果：0.25 ~ 0.75           │
│ else:                         │
│   vel = uniform(0.5, 0.5)     │
│   结果：始终 0.5               │
└──────────────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ 机器人                       │
│   所有环境执行命令：          │
│   vx ~ 0.5 (带噪声)          │
│   vy = 0.0                   │
│   wz = 0.0                   │
└──────────────────────────────┘
```

---

## 文件结构

```
source/MyProject/MyProject/tasks/manager_based/WalkTest/
├── walk_flat_env_cfg.py           # 环境配置
│   └── VelocityGo2WalkFlatEnvCfg_Ros  # Ros 版本配置
└── mdp/
    ├── socket_velocity_command.py      # Socket 命令类
    │   ├── SocketVelocityCommand       # 继承 UniformVelocityCommand
    │   ├── __init__()                   # 初始化 + 启动 socket
    │   ├── _init_socket()              # 创建 UDP 服务器
    │   ├── _receive_loop()             # 接收命令并更新 cfg.ranges
    │   ├── _resample_command()        # 添加噪声注入
    │   └── _is_fixed_command()         # 检查是否为固定命令
    │
    ├── socket_velocity_command_cfg.py  # Socket 配置类
    │   └── SocketVelocityCommandCfg    # 继承 UniformVelocityCommandCfg
    │
    └── rewards.py                    # 奖励函数（含 DEBUG 日志）

Socket/
└── send_cmd.py                   # 命令发送工具（键盘控制）
```

---

## 工作流程

### 1. 初始化阶段

```python
# walk_flat_env_cfg.py
self.commands.base_velocity = SocketVelocityCommandCfg(
    asset_name="robot",
    port=5555,
    resampling_time_range=(5.0, 5.0),  # 重采样间隔
    rel_standing_envs=0.0,              # 禁用随机静止
    heading_command=False,                # 角速度模式
    add_command_noise=True,               # 启用噪声
    noise_scale=0.25,                   # 噪声幅度 ±0.25 m/s
    socket_vx=0.0,                     # 默认值
    socket_vy=0.0,
    socket_wz=0.0,
    ranges=SocketVelocityCommandCfg.Ranges(
        lin_vel_x=(0.0, 0.0),   # 初始固定为 0
        lin_vel_y=(0.0, 0.0),
        ang_vel_z=(0.0, 0.0),
    ),
)
```

**行为**：机器人初始静止，等待 socket 命令

### 2. Socket 命令接收阶段

```python
# send_cmd.py 发送 "0.5,0.0,0.0"
↓
# _receive_loop() 接收
self.cfg.socket_vx = 0.5
self.cfg.ranges.lin_vel_x = (0.5, 0.5)  # ← 关键：固定范围
```

**行为**：所有机器人统一执行 `vx~0.5`（带噪声）

### 3. 命令重采样阶段

```python
# _resample_command() 被调用
if add_command_noise:
    # 添加噪声
    noise_x = uniform(-0.25, 0.25)
    vel_command_b[env_ids, 0] = 0.5 + noise_x  # 0.25 ~ 0.75
else:
    # 不添加噪声
    vel_command_b[env_ids, 0] = uniform(0.5, 0.5)  # 始终 0.5
```

**关键**：每 5 秒重采样一次，噪声在每次重采样时更新

---

## 配置说明

### 环境配置 (walk_flat_env_cfg.py)

```python
@configclass
class VelocityGo2WalkFlatEnvCfg_Ros(VelocityGo2WalkFlatEnvCfg_Play):
    def __post_init__(self) -> None:
        super().__post_init__()

        # Socket 命令值（定义变量，方便理解）
        socket_vx = 0.0  # 默认值，socket 会更新
        socket_vy = 0.0
        socket_wz = 0.0

        self.commands.base_velocity = SocketVelocityCommandCfg(
            asset_name="robot",
            port=5555,
            resampling_time_range=(5.0, 5.0),   # 每 5 秒重采样
            rel_standing_envs=0.0,              # 禁用随机静止
            heading_command=False,                # 角速度模式
            heading_control_stiffness=0.5,
            debug_vis=True,
            socket_vx=socket_vx,                 # 传递给配置
            socket_vy=socket_vy,
            socket_wz=socket_wz,
            add_command_noise=True,               # 启用噪声
            noise_scale=0.25,                    # ±0.25 m/s 噪声
            use_sinusoidal_variation=False,        # 使用随机噪声
            ranges=SocketVelocityCommandCfg.Ranges(
                lin_vel_x=(socket_vx, socket_vx),     # 使用变量
                lin_vel_y=(socket_vy, socket_vy),
                ang_vel_z=(socket_wz, socket_wz),
                heading=(-math.pi, math.pi),
            ),
        )
```

### 配置参数说明

| 参数 | 说明 | 默认值 | 推荐值 |
|------|------|--------|--------|
| `port` | UDP 监听端口 | 5555 | 5555 |
| `socket_vx/vy/wz` | Socket 命令值（由 socket 更新） | 0.0 | - |
| `heading_command` | True=航向模式，False=角速度模式 | False | False |
| `rel_standing_envs` | 随机静止环境的概率 | 0.0 | 0.0 |
| `resampling_time_range` | 命令重采样间隔（秒） | (5.0, 5.0) | (5.0, 5.0) |
| `add_command_noise` | 是否添加噪声 | True | True |
| `noise_scale` | 噪声幅度（±m/s） | 0.25 | 0.15-0.25 |
| `use_sinusoidal_variation` | 是否使用正弦变化 | False | False |
| `variation_amp` | 正弦变化幅度 | 0.03 | 0.03 |

### 角速度模式 vs 航向模式

#### 角速度模式（heading_command=False，推荐）

```
命令格式：lin_x, lin_y, ang_z
含义：
  - lin_x: 前进/后退速度（m/s）
  - lin_y: 左移/右移速度（m/s）
  - ang_z: 旋转角速度（rad/s）

示例：
  "0.5,0.0,0.0"   → 前进 0.5 m/s
  "0.0,0.5,0.0"   → 左移 0.5 m/s
  "0.0,0.0,0.5"   → 左转 0.5 rad/s
```

**优点**：
- 直观控制
- 适合实时控制
- 响应快

**推荐场景**：
- 手动控制
- 遥控机器人
- 需要快速响应的场景

#### 航向模式（heading_command=True）

```
命令格式：lin_x, lin_y, heading
含义：
  - lin_x: 前进/后退速度（m/s）
  - lin_y: 左移/右移速度（m/s）
  - heading: 目标航向角（rad）

示例：
  "0.5,0.0,1.57"  → 前进并转向 90°
  "0.5,0.0,0.0"   → 前进并保持当前航向
  "0.5,0.0,-1.57" → 前进并转向 -90°
```

**优点**：
- 精确控制航向
- 适合导航任务

**推荐场景**：
- 自主导航
- 需要精确定位的场景

---

## 使用方法

### 1. 启动仿真

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe

# 使用 conda 环境
/home/xcj/miniconda3/envs/env_isaaclab/bin/python scripts/rsl_rl/play.py \
    --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
    --checkpoint ModelBackup/WalkPolicy/WalkFlatNew.pt
```

**预期行为**：
- 初始：机器人静止（等待 socket 命令）
- Socket 命令后：统一执行命令

### 2. 发送命令

#### 方式 1：使用 send_cmd.py（推荐）

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe/Socket
python send_cmd.py
```

**键盘控制（角速度模式）**：
- `W` - 前进 (vx=0.5)
- `S` - 后退 (vx=-0.3)
- `A` - 左移 (vy=0.5)
- `D` - 右移 (vy=-0.5)
- `Q` - 左转 (wz=0.5)
- `E` - 右转 (wz=-0.5)
- `Space` - 停止
- `ESC` - 退出

#### 方式 2：使用 Python 脚本

```python
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 发送命令
sock.sendto(b"0.5,0.0,0.0", ('127.0.0.1', 5555))
print("Sent: vx=0.5, vy=0.0, wz=0.0")

sock.close()
```

#### 方式 3：使用 netcat

```bash
# 单次发送
echo "0.5,0.0,0.0" | nc -u 127.0.0.1 5555

# 持续发送（每 0.5 秒）
while true; do
    echo "0.5,0.0,0.0" | nc -u 127.0.0.1 5555
    sleep 0.5
done
```

### 3. 查看日志

```bash
# 查看实时日志
tail -f /tmp/isaaclab_*.log

# 查看 DEBUG 信息
grep "DEBUG" /tmp/isaaclab_*.log

# 查看命令日志
grep "Fixed Command" /tmp/isaaclab_*.log
```

**日志示例**：
```
✓ Socket listening on port 5555
[Socket] Command: 0.50, 0.00, 0.00
[DEBUG] Fixed Command - Target: (0.500, 0.000, 0.000), Noise scale: ±0.250
[DEBUG] Fixed Command - Actual: vx=0.512±0.145, vy=0.003±0.146, wz=0.002±0.144
```

---

## 噪声注入机制

### 为什么需要噪声？

**问题**：
- 训练时：命令在 `(-1.0, 1.0)` 范围内均匀分布（宽分布）
- 推理时：固定命令 `vx=0.5`（窄分布）
- 策略对绝对观测值敏感（`actor_obs_normalization=False`）
- 结果：策略表现不稳定，可能停止

**解决方案**：
在固定命令上添加噪声，模拟训练时的分布。

### 噪声类型

#### 1. 随机噪声（推荐）

```python
add_command_noise=True
noise_scale=0.25  # ±0.25 m/s
```

**特点**：
- 每个环境独立采样
- 每次重采样时更新
- 命令在 `[目标 - 0.25, 目标 + 0.25]` 范围内

**效果**：
```
目标：vx=0.5
实际：vx ~ Uniform(0.25, 0.75)
均值：~0.5（接近目标）
标准差：~0.144（噪声幅度/√3）
```

#### 2. 正弦变化

```python
add_command_noise=True
use_sinusoidal_variation=True
variation_amp=0.03  # ±0.03 m/s
```

**特点**：
- 平滑变化
- 周期性波动
- 所有环境共享相位

**效果**：
```
目标：vx=0.5
实际：vx = 0.5 + 0.03 * sin(2π * t / 100)
范围：[0.47, 0.53]
变化：平滑，周期性
```

### 噪声参数调优

| 噪声幅度 | 效果 | 适用场景 |
|---------|------|---------|
| `0.05` | 噪声小，接近固定命令 | 需要精确控制 |
| `0.15` | 噪声适中（推荐） | 平衡稳定性和控制精度 |
| `0.25` | 噪声大，接近训练分布 | 策略不稳定时 |
| `0.50` | 噪声很大 | 极端情况，不推荐 |

### 重采样频率

**配置**：
```python
resampling_time_range=(5.0, 5.0)  # 每 5 秒
```

**对比**：
| 重采样频率 | 训练时 | 推理时 | 效果 |
|-----------|--------|--------|------|
| 0.5 秒 | - | 太快 | 不稳定，无法稳定 |
| 5.0 秒 | 10.0 秒 | 接近 | 平衡稳定性和响应性 |
| 10.0 秒 | 10.0 秒 | 相同 | 最稳定，但响应慢 |

**推荐**：
- 稳定性优先：`(8.0, 8.0)` 或 `(10.0, 10.0)`
- 响应性优先：`(3.0, 3.0)` 或 `(5.0, 5.0)`
- 平衡：`(5.0, 5.0)`（当前配置）

---

## 代码实现详解

### 1. socket_velocity_command.py

#### 类结构

```python
class SocketVelocityCommand(UniformVelocityCommand):
    """继承 UniformVelocityCommand，添加 socket 和噪声功能"""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)    # 调用父类初始化
        self._init_socket()            # 启动 socket 服务器
        self._command_lock = threading.Lock()

    def _init_socket(self):
        """创建 UDP socket 和接收线程"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', cfg.port))
        threading.Thread(target=self._receive_loop, args=(sock,), daemon=True).start()
        print(f"✓ Socket listening on port {cfg.port}")

    def _receive_loop(self, sock):
        """接收循环（独立线程）"""
        while True:
            data, _ = sock.recvfrom(1024)
            parts = data.decode().strip().split(',')
            if len(parts) >= 3:
                with self._command_lock:
                    # 更新 socket 命令值
                    self.cfg.socket_vx = float(parts[0])
                    self.cfg.socket_vy = float(parts[1])
                    self.cfg.socket_wz = float(parts[2])

                    # 关键：修改采样范围来固定命令值
                    self.cfg.ranges.lin_vel_x = (self.cfg.socket_vx, self.cfg.socket_vx)
                    self.cfg.ranges.lin_vel_y = (self.cfg.socket_vy, self.cfg.socket_vy)
                    self.cfg.ranges.ang_vel_z = (self.cfg.socket_wz, self.cfg.socket_wz)

                print(f"[Socket] Command: {self.cfg.socket_vx:.2f}, {self.cfg.socket_vy:.2f}, {self.cfg.socket_wz:.2f}")

    def _resample_command(self, env_ids):
        """覆盖父类方法，添加噪声注入"""
        # 检查是否为固定命令
        is_fixed = self._is_fixed_command()

        if is_fixed and (self.add_command_noise or self.use_sinusoidal_variation):
            # 获取目标值
            target_vx = self.cfg.socket_vx
            target_vy = self.cfg.socket_vy
            target_wz = self.cfg.socket_wz

            num_envs = len(env_ids)

            if self.use_sinusoidal_variation:
                # 正弦变化
                self._step_counter += 1
                phase = 2 * math.pi * self._step_counter / 100.0
                r = torch.empty(num_envs, device=self.device)
                noise_x = self.variation_amp * math.sin(phase) + r.uniform_(-0.01, 0.01)
                noise_y = self.variation_amp * math.cos(phase * 1.3) + r.uniform_(-0.01, 0.01)
                noise_z = self.variation_amp * math.sin(phase * 0.7) + r.uniform_(-0.01, 0.01)
            else:
                # 随机噪声
                noise_x = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)
                noise_y = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)
                noise_z = torch.empty(num_envs, device=self.device).uniform_(-self.noise_scale, self.noise_scale)

            # 应用到所有环境
            self.vel_command_b[env_ids, 0] = target_vx + noise_x
            self.vel_command_b[env_ids, 1] = target_vy + noise_y
            self.vel_command_b[env_ids, 2] = target_wz + noise_z

            # DEBUG 日志
            if len(env_ids) > 1:
                actual_vx_std = self.vel_command_b[env_ids, 0].std().item()
                actual_vy_std = self.vel_command_b[env_ids, 1].std().item()
                actual_wz_std = self.vel_command_b[env_ids, 2].std().item()
            else:
                actual_vx_std = actual_vy_std = actual_wz_std = 0.0

            actual_vx_mean = self.vel_command_b[env_ids, 0].mean().item()
            actual_vy_mean = self.vel_command_b[env_ids, 1].mean().item()
            actual_wz_mean = self.vel_command_b[env_ids, 2].mean().item()

            print(f"[DEBUG] Fixed Command - Target: ({target_vx:.3f}, {target_vy:.3f}, {target_wz:.3f}), Noise scale: ±{self.noise_scale:.3f}")
            print(f"[DEBUG] Fixed Command - Actual: vx={actual_vx_mean:.3f}±{actual_vx_std:.3f}, vy={actual_vy_mean:.3f}±{actual_vy_std:.3f}, wz={actual_wz_mean:.3f}±{actual_wz_std:.3f}")
        else:
            # 使用父类方法
            super()._resample_command(env_ids)

    def _is_fixed_command(self):
        """检查是否为固定命令（range min == max）"""
        return (
            abs(self.cfg.ranges.lin_vel_x[1] - self.cfg.ranges.lin_vel_x[0]) < 1e-6 and
            abs(self.cfg.ranges.lin_vel_y[1] - self.cfg.ranges.lin_vel_y[0]) < 1e-6 and
            abs(self.cfg.ranges.ang_vel_z[1] - self.cfg.ranges.ang_vel_z[0]) < 1e-6
        )
```

#### 关键设计点

1. **继承而非重写**：复用 `UniformVelocityCommand` 的所有逻辑
2. **范围固定机制**：通过修改采样范围来控制命令值
3. **噪声注入**：在固定命令上添加噪声，提高策略兼容性
4. **线程安全**：使用 `_command_lock` 保护共享数据
5. **无侵入性**：不修改 `UniformVelocityCommand` 的核心逻辑
6. **调试支持**：内置 DEBUG 日志输出

### 2. socket_velocity_command_cfg.py

```python
@configclass
class SocketVelocityCommandCfg(UniformVelocityCommandCfg):
    """继承 UniformVelocityCommandCfg"""

    class_type: type = SocketVelocityCommand
    port: int = 5555

    # Socket 命令值（由 socket 更新）
    socket_vx: float = 0.0
    socket_vy: float = 0.0
    socket_wz: float = 0.0

    # 噪声注入配置
    add_command_noise: bool = True
    noise_scale: float = 0.25
    use_sinusoidal_variation: bool = False
    variation_amp: float = 0.03
```

#### 为什么添加这些参数？

1. **socket_vx/vy/wz**：显式存储 socket 命令值，易于访问和调试
2. **add_command_noise**：开关噪声注入功能
3. **noise_scale**：控制噪声幅度
4. **use_sinusoidal_variation**：选择噪声类型（随机/正弦）
5. **variation_amp**：控制正弦变化的幅度

### 3. send_cmd.py

```python
class SocketCommandSender:
    """持续连接的命令发送器"""

    def __init__(self, host='127.0.0.1', port=5555):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, lin_x, lin_y, ang_z):
        """发送速度命令"""
        message = f"{lin_x},{lin_y},{ang_z}"
        self.sock.sendto(message.encode('utf-8'), (self.host, self.port))
```

#### 特点

1. **持续连接**：创建一次 socket，重复使用
2. **键盘控制**：支持 WASDQE 键控制
3. **即时响应**：按键立即发送命令
4. **易于使用**：无需网络知识

---

## 总结

### 核心优势

1. **简洁**：继承设计，代码量少（~140 行）
2. **兼容**：完全兼容 `UniformVelocityCommand` 的所有特性
3. **灵活**：支持角速度/航向双模式
4. **稳定**：噪声注入提高策略兼容性
5. **直观**：通过范围固定机制，逻辑清晰
6. **可调试**：内置 DEBUG 日志输出

### 代码对比

| 方案 | 代码量 | 复杂度 | 维护性 | 噪声支持 |
|------|--------|--------|--------|---------|
| **继承方案（当前）** | ~140 行 | 低 | 高（复用父类） | ✅ 支持 |
| 重写方案 | ~300 行 | 高 | 低（需同步更新） | ❌ 不支持 |

### 关键设计模式

1. **继承复用**：通过继承 `UniformVelocityCommand` 复用现有逻辑
2. **范围固定**：通过修改采样范围来控制命令值
3. **最小修改**：只添加必要的 socket 和噪声功能，其他保持不变
4. **噪声注入**：在固定命令上添加噪声，模拟训练分布

### 性能优化

1. **线程安全**：使用 `_command_lock` 保护共享数据
2. **独立线程**：socket 接收不影响仿真主循环
3. **批量更新**：一次更新所有环境的命令
4. **条件日志**：只在需要时输出 DEBUG 信息

---

## 附录

### A. 问题排查

#### Q1: 机器人不移动

**可能原因**：
1. 未发送 socket 命令，机器人使用默认值 0.0
2. 端口被占用
3. `add_command_noise=False` 且命令为 0.0

**解决方法**：
```bash
# 检查端口
netstat -tuln | grep 5555

# 检查日志
grep "Socket" /tmp/isaaclab_*.log

# 发送命令
python send_cmd.py
```

#### Q2: 机器人停止移动

**可能原因**：
1. `add_command_noise=False` 且使用固定命令
2. `resampling_time_range` 太短（< 3.0 秒）
3. `noise_scale` 太小（< 0.1）
4. 训练时使用宽分布，推理时使用窄分布

**解决方法**：
```python
# 启用噪声
add_command_noise=True
noise_scale=0.25  # 增大噪声

# 增加重采样时间
resampling_time_range=(8.0, 8.0)
```

#### Q3: 命令不生效

**检查步骤**：
1. 确认 socket 连接正常
2. 检查日志输出 `[Socket] Command:`
3. 确认 `cfg.ranges` 已更新
4. 检查 DEBUG 日志中的实际命令值

#### Q4: 机器人左右摇摆

**可能原因**：
1. `noise_scale` 太大，vy 噪声过大
2. `rel_standing_envs` 不为 0

**解决方法**：
```python
# 减小噪声
noise_scale=0.15  # 从 0.25 减小

# 禁用随机静止
rel_standing_envs=0.0
```

### B. 调试技巧

#### 1. 查看 DEBUG 日志

```bash
# 查看命令重采样日志
grep "DEBUG Fixed Command" /tmp/isaaclab_*.log

# 查看实际命令值
grep "Actual:" /tmp/isaaclab_*.log
```

**健康指标**：
- ✅ Target 接近目标值（如 0.500）
- ✅ Actual 均值接近 Target（如 0.512）
- ✅ Actual 标准差接近 `noise_scale/√3`（如 0.144）
- ❌ Target 频繁回到 0.000
- ❌ Actual 均值偏离 Target 超过 ±0.2
- ❌ Actual 标准差远小于 `noise_scale/√3`

#### 2. 测试 socket 连接

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

try:
    sock.sendto(b"0.5,0.0,0.0", ('127.0.0.1', 5555))
    print("✓ Socket 连接成功")
except socket.error as e:
    print(f"✗ Socket 连接失败: {e}")
finally:
    sock.close()
```

#### 3. 监控端口

```bash
# 检查端口是否监听
netstat -ulpn | grep 5555

# 或使用 ss
ss -ulpn | grep 5555
```

### C. 参考文档

- [Isaac Lab 官方文档](https://isaac-sim.github.io/IsaacLab/main/)
- [UniformVelocityCommand 源码](source/isaaclab/isaaclab/envs/mdp/commands/velocity_command.py)

### D. 性能优化建议

1. **多环境测试**：使用 `num_envs > 1` 测试，验证噪声效果
2. **调整重采样时间**：根据稳定性调整 `resampling_time_range`
3. **优化噪声幅度**：根据观察调整 `noise_scale`
4. **监控日志**：定期检查 DEBUG 日志，确保命令正常

---

## 更新日志

### v2.0 (2025-01-29)

**新增功能**：
- ✅ 噪声注入机制（`add_command_noise`, `noise_scale`）
- ✅ 正弦变化选项（`use_sinusoidal_variation`, `variation_amp`）
- ✅ DEBUG 日志输出
- ✅ 线程安全保护（`_command_lock`）

**改进**：
- 🔄 修复 std() 计算在单环境时的警告
- 🔄 优化重采样频率（从 10.0 秒改为 5.0 秒）
- 🔄 改进日志输出（使用 `print` 替代 `logger`）

**修复**：
- 🐛 修复固定命令导致机器人停止的问题
- 🐛 修复单环境时的标准差计算问题

### v1.0 (初始版本)

**核心功能**：
- ✅ Socket 命令接收
- ✅ 范围固定机制
- ✅ 继承 `UniformVelocityCommand`
- ✅ 支持角速度/航向双模式
