# Socket 命令通信实现说明

## 目录

1. [概述](#概述)
2. [设计思路](#设计思路)
3. [核心实现原理](#核心实现原理)
4. [文件结构](#文件结构)
5. [工作流程](#工作流程)
6. [配置说明](#配置说明)
7. [使用方法](#使用方法)
8. [代码实现详解](#代码实现详解)

---

## 概述

本项目实现了一个基于 UDP Socket 的速度命令接收系统，用于控制 Isaac Lab 中的 Unitree Go2 四足机器人。该系统通过**继承 `UniformVelocityCommand`** 并**动态修改采样范围**来实现外部控制，相比 ROS2 方案更加简单，无外部依赖。

### 核心特性

- **继承设计**：`SocketVelocityCommand` 继承 `UniformVelocityCommand`，复用所有现有逻辑
- **范围固定**：通过将采样范围设为 `(v, v)` 来固定命令值
- **双模式**：支持角速度模式（`heading_command=False`）和航向模式（`heading_command=True`）
- **线程安全**：独立线程接收命令，不影响仿真主循环
- **无缝集成**：完全兼容 Isaac Lab 的训练和推理流程

### 核心思路

**问题**：`UniformVelocityCommand` 使用随机采样生成命令，如何让它使用外部 socket 命令？

**解决方案**：
1. 初始化时使用随机范围 `(-1.0, 1.0)`，机器人随机行走
2. Socket 收到命令 `"0.5,0.0,0.0"` 后，将范围改为 `(0.5, 0.5)`
3. 因为采样范围上下界相同，所以随机采样总是返回这个固定值
4. 所有环境统一执行相同的命令

---

## 设计思路

### 继承关系

```
UniformVelocityCommand (Isaac Lab 官方)
    ↑ 继承
SocketVelocityCommand (自定义)
    ├── 添加 socket 接收功能
    ├── 覆盖 _resample_command() (可选)
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
```

### 2. 命令类实现

```python
class SocketVelocityCommand(UniformVelocityCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)  # 调用父类初始化
        self._init_socket()          # 启动 socket 接收线程

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
│ UniformVelocityCommand       │
│ (父类方法):                   │
│                              │
│ _resample_command():          │
│   vel = uniform(             │
│     cfg.ranges.lin_vel_x     │
│   )  = uniform(0.5, 0.5)     │
│   结果：始终是 0.5            │
└──────────────────────────────┘
       │
       ▼
┌──────────────────────────────┐
│ 机器人                       │
│   所有环境执行相同命令：      │
│   lin_vel_x = 0.5            │
│   lin_vel_y = 0.0            │
│   ang_vel_z = 0.0            │
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
    │   └── _receive_loop()             # 接收命令并更新 cfg.ranges
    │
    └── socket_velocity_command_cfg.py  # Socket 配置类
        └── SocketVelocityCommandCfg    # 继承 UniformVelocityCommandCfg

Socket/
└── send_cmd.py                   # 命令发送工具
```

---

## 工作流程

### 1. 初始化阶段

```python
# walk_flat_env_cfg.py
self.commands.base_velocity = SocketVelocityCommandCfg(
    asset_name="robot",
    port=5555,
    socket_vx=0.0,        # 默认值
    socket_vy=0.0,
    socket_wz=0.0,
    ranges=SocketVelocityCommandCfg.Ranges(
        lin_vel_x=(-1.0, 1.0),   # 初始随机范围
        lin_vel_y=(-1.0, 1.0),
        ang_vel_z=(-1.0, 1.0),
    ),
)
```

**行为**：机器人在 `(-1.0, 1.0)` 范围内随机行走

### 2. Socket 命令接收阶段

```python
# send_cmd.py 发送 "0.5,0.0,0.0"
↓
# _receive_loop() 接收
self.cfg.socket_vx = 0.5
self.cfg.ranges.lin_vel_x = (0.5, 0.5)  # ← 关键：固定范围
```

**行为**：所有机器人统一执行 `vx=0.5, vy=0.0, wz=0.0`

### 3. 命令更新阶段

```python
# UniformVelocityCommand._resample_command() 被调用
# 采样：lin_vel_x = uniform(0.5, 0.5) = 0.5（始终相同）
# 应用：vel_command_b[env_ids, 0] = 0.5
```

**关键**：虽然还是"随机采样"，但因为范围上下界相同，结果总是固定值

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
            resampling_time_range=(10.0, 10.0),
            rel_standing_envs=0.02,              # 2% 环境随机静止
            heading_command=True,                # 航向模式
            heading_control_stiffness=0.5,
            debug_vis=True,
            socket_vx=socket_vx,                 # 传递给配置
            socket_vy=socket_vy,
            socket_wz=socket_wz,
            ranges=SocketVelocityCommandCfg.Ranges(
                lin_vel_x=(socket_vx, socket_vx),     # 使用变量
                lin_vel_y=(socket_vy, socket_vy),
                ang_vel_z=(socket_wz, socket_wz),
                heading=(-math.pi, math.pi),
            ),
        )
```

### 配置参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `port` | UDP 监听端口 | 5555 |
| `socket_vx/vy/wz` | Socket 命令值（由 socket 更新） | 0.0 |
| `heading_command` | True=航向模式，False=角速度模式 | True |
| `rel_standing_envs` | 随机静止环境的概率 | 0.02 (2%) |
| `resampling_time_range` | 命令重采样间隔 | (10.0, 10.0) 秒 |

---

## 使用方法

### 1. 启动仿真

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python scripts/rsl_rl/play.py \
    --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
    --checkpoint /path/to/model.pt
```

**预期行为**：
- 初始：机器人随机行走（范围 `-1.0 ~ 1.0`）
- Socket 命令后：统一执行命令

### 2. 发送命令

#### 方式 1：使用 send_cmd.py（推荐）

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe/Socket
python send_cmd.py
```

**键盘控制**：
- `W` - 前进 (vx=0.5)
- `S` - 后退 (vx=-0.3)
- `A` - 左移 (vy=0.5)
- `D` - 右移 (vy=-0.5)
- `Q` - 左转 (wz=0.5)
- `E` - 右转 (wz=-0.5)
- `Space` - 停止

#### 方式 2：使用 Python 脚本

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"0.5,0.0,0.0", ('127.0.0.1', 5555))
sock.close()
```

#### 方式 3：使用 netcat

```bash
echo "0.5,0.0,0.0" | nc -u 127.0.0.1 5555
```

---

## 代码实现详解

### 1. socket_velocity_command.py

#### 类结构

```python
class SocketVelocityCommand(UniformVelocityCommand):
    """继承 UniformVelocityCommand，添加 socket 功能"""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)    # 调用父类初始化
        self._init_socket()            # 启动 socket 服务器

    def _init_socket(self):
        """创建 UDP socket 和接收线程"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', cfg.port))
        threading.Thread(target=self._receive_loop, args=(sock,), daemon=True).start()

    def _receive_loop(self, sock):
        """接收循环（独立线程）"""
        while True:
            data, _ = sock.recvfrom(1024)
            parts = data.decode().strip().split(',')
            if len(parts) >= 3:
                # 更新 socket 命令值
                self.cfg.socket_vx = float(parts[0])
                self.cfg.socket_vy = float(parts[1])
                self.cfg.socket_wz = float(parts[2])

                # 关键：修改采样范围来固定命令值
                self.cfg.ranges.lin_vel_x = (self.cfg.socket_vx, self.cfg.socket_vx)
                self.cfg.ranges.lin_vel_y = (self.cfg.socket_vy, self.cfg.socket_vy)
                self.cfg.ranges.ang_vel_z = (self.cfg.socket_wz, self.cfg.socket_wz)
```

#### 关键设计点

1. **继承而非重写**：复用 `UniformVelocityCommand` 的所有逻辑
2. **范围固定机制**：通过将范围改为 `(v, v)` 来固定命令值
3. **线程安全**：使用 `_command_lock` 保护共享数据（当前实现中省略，可优化）
4. **无侵入性**：不修改 `UniformVelocityCommand` 的核心逻辑

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
```

#### 为什么添加 socket_vx/vy/wz？

1. **显式存储**：在配置中明确定义 socket 命令值
2. **易于访问**：通过 `cfg.socket_vx` 直接访问当前命令
3. **便于调试**：可以在配置中看到当前命令值

### 3. 命令格式

#### 角速度模式 (heading_command=False)

```
格式： "lin_x,lin_y,ang_z"
示例： "0.5,0.0,0.0"  → 前进 0.5 m/s
      "0.0,0.5,0.0"  → 左移 0.5 m/s
      "0.0,0.0,0.5"  → 左转 0.5 rad/s
```

#### 航向模式 (heading_command=True)

```
格式： "lin_x,lin_y,heading"
示例： "0.5,0.0,1.57"  → 前进并转向 90°
      "0.5,0.0,0.0"   → 前进并保持当前航向
      "0.5,0.0,-1.57" → 前进并转向 -90°
```

### 4. 数据流详解

#### 初始化阶段

```
1. 加载配置
   cfg.socket_vx = 0.0
   cfg.ranges.lin_vel_x = (-1.0, 1.0)

2. 调用父类 __init__()
   → 创建 vel_command_b[num_envs, 3]

3. 启动 socket 线程
   → 监听 127.0.0.1:5555

4. 第一次命令重采样
   → 采样范围：uniform(-1.0, 1.0)
   → 结果：随机值，机器人随机行走
```

#### Socket 命令接收阶段

```
1. send_cmd.py 发送 "0.5,0.0,0.0"
   ↓
2. _receive_loop() 接收
   cfg.socket_vx = 0.5
   cfg.ranges.lin_vel_x = (0.5, 0.5)  ← 修改范围
   ↓
3. 下一次 _resample_command() 调用
   采样：uniform(0.5, 0.5) = 0.5
   ↓
4. 应用到所有环境
   vel_command_b[:, 0] = 0.5
   vel_command_b[:, 1] = 0.0
   vel_command_b[:, 2] = 0.0
```

---

## 总结

### 核心优势

1. **简洁**：继承设计，代码量少（~60 行）
2. **兼容**：完全兼容 `UniformVelocityCommand` 的所有特性
3. **灵活**：支持角速度/航向双模式
4. **直观**：通过范围固定机制，逻辑清晰

### 代码对比

| 方案 | 代码量 | 复杂度 | 维护性 |
|------|--------|--------|--------|
| **继承方案（当前）** | ~60 行 | 低 | 高（复用父类） |
| 重写方案 | ~300 行 | 高 | 低（需同步更新） |

### 关键设计模式

1. **继承复用**：通过继承 `UniformVelocityCommand` 复用现有逻辑
2. **范围固定**：通过修改采样范围来控制命令值
3. **最小修改**：只添加必要的 socket 功能，其他保持不变

### 未来优化方向

1. **线程安全**：添加 `_command_lock` 保护 `cfg.socket_vx/vy/wz` 访问
2. **命令平滑**：在两个命令之间插值，避免突变
3. **多机器人**：支持不同机器人使用不同命令
4. **命令队列**：缓存多个命令，按时间顺序执行

---

## 附录

### A. 问题排查

#### Q1: 机器人不移动

**可能原因**：
1. 未发送 socket 命令，机器人使用默认值 0.0
2. 端口被占用

**解决方法**：
```bash
# 检查端口
netstat -tuln | grep 5555

# 发送命令
python send_cmd.py
```

#### Q2: 部分机器人停止移动

**原因**：`rel_standing_envs=0.02` 导致 2% 环境随机静止

**这是正常行为**，可以调低或设为 0：
```python
rel_standing_envs=0.0  # 禁用随机静止
```

#### Q3: 命令不生效

**检查步骤**：
1. 确认 socket 连接正常
2. 检查日志输出
3. 确认 `cfg.ranges` 已更新

### B. 参考文档

- [Isaac Lab 官方文档](https://isaac-sim.github.io/IsaacLab/main/)
- [UniformVelocityCommand 源码](source/isaaclab/isaaclab/envs/mdp/commands/velocity_command.py)
