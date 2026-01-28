# Socket 命令通信实现说明

## 目录

1. [概述](#概述)
2. [架构设计](#架构设计)
3. [文件结构](#文件结构)
4. [核心实现](#核心实现)
5. [工作流程](#工作流程)
6. [配置说明](#配置说明)
7. [使用方法](#使用方法)
8. [代码逻辑详解](#代码逻辑详解)

---

## 概述

本项目实现了一个基于 UDP Socket 的速度命令接收系统，用于控制 Isaac Lab 中的 Unitree Go2 四足机器人。该系统相比 ROS2 方案更加简单，无外部依赖，避免了 Python 版本兼容性问题。

### 核心特性

- **UDP 通信**：使用 UDP Socket 接收速度命令
- **双模式支持**：角速度模式 / 航向模式
- **线程安全**：独立线程接收命令，不影响仿真主循环
- **无缝集成**：完全兼容 Isaac Lab 的训练和推理流程

---

## 架构设计

### 整体架构图

```
┌─────────────────┐         UDP          ┌──────────────────────┐
│  发送程序        │  ────────────────>  │  SocketVelocityCommand │
│ (send_cmd.py)   │  "lin_x,lin_y,ang_z" │  (接收线程)            │
└─────────────────┘                   └──────────────────────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │ 命令缓冲区    │
                                          │ _current_cmd  │
                                          └──────────────┘
                                                 │
                                                 ▼ (每个仿真步)
                                          ┌──────────────┐
                                          │ _update_cmd  │
                                          └──────────────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │ vel_command_b│
                                          └──────────────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │   机器人      │
                                          │   (所有环境)  │
                                          └──────────────┘
```

### 模块关系

```
Isaac Lab Environment
│
├── CommandManager
│   └── SocketVelocityCommand (自定义命令类)
│       ├── socket_velocity_command.py (实现)
│       └── socket_velocity_command_cfg.py (配置)
│
└── Robot
    └── 接收 vel_command_b 中的速度命令
```

---

## 文件结构

### 新增/修改的文件

```
source/MyProject/MyProject/tasks/manager_based/WalkTest/
├── __init__.py                    # 导出任务注册
├── walk_flat_env_cfg.py           # 环境配置 (已修改)
└── mdp/
    ├── __init__.py                # MDP 模块导出 (已修改)
    ├── socket_velocity_command.py       # Socket 命令类 (新增)
    ├── socket_velocity_command_cfg.py   # Socket 配置类 (新增)
    ├── curriculums.py            # 课程相关
    ├── rewards.py                # 奖励函数
    └── terminations.py           # 终止条件

Socket/                            # 外部工具目录
└── send_cmd.py                   # 命令发送工具 (新增)
```

---

## 核心实现

### 1. Socket 命令类 (`socket_velocity_command.py`)

#### 类结构

```python
class SocketVelocityCommand(CommandTerm):
    """继承自 Isaac Lab 的 CommandTerm 基类"""

    def __init__(self, cfg, env):
        """初始化：
        - 创建命令缓冲区
        - 启动 Socket 服务器
        - 启动接收线程
        """

    def _update_command(self):
        """每个仿真步调用：
        - 从缓冲区读取最新命令
        - 根据模式处理（角速度/航向）
        - 裁剪到配置范围
        - 更新 vel_command_b
        """

    def _socket_receive_loop(self):
        """独立线程运行：
        - 持续监听 UDP 端口
        - 解析接收到的消息
        - 更新 _current_command 缓冲区
        """
```

#### 核心代码逻辑

**初始化流程**：

```python
def __init__(self, cfg, env):
    # 1. 创建命令缓冲区 (用于所有环境)
    self.vel_command_b = torch.zeros(self.num_envs, 3, device=self.device)

    # 2. 航向目标缓冲区 (航向模式使用)
    self.heading_target = torch.zeros(self.num_envs, device=self.device)

    # 3. 当前命令缓冲区 (线程安全)
    self._current_command = [0.0, 0.0, 0.0, 0.0]  # [lin_x, lin_y, ang_z, heading]

    # 4. 初始化 Socket 服务器
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self._socket.bind(('127.0.0.1', cfg.port))

    # 5. 启动接收线程 (daemon=True)
    self._socket_thread = threading.Thread(
        target=self._socket_receive_loop,
        daemon=True
    )
    self._socket_thread.start()
```

**命令更新流程**：

```python
def _update_command(self):
    # 1. 从线程安全缓冲区读取命令
    with self._command_lock:
        lin_x = self._current_command[0]
        lin_y = self._current_command[1]
        value_z = self._current_command[2]  # ang_z 或 heading

    # 2. 裁剪到配置范围
    lin_x = clip(lin_x, self.cfg.ranges.lin_vel_x)
    lin_y = clip(lin_y, self.cfg.ranges.lin_vel_y)

    # 3. 根据模式处理
    if self.cfg.heading_command:
        # 航向模式：计算角速度
        heading = clip(value_z, self.cfg.ranges.heading)
        self.heading_target[:] = heading

        # 计算航向误差
        heading_error = wrap_to_pi(
            self.heading_target - self.robot.data.heading_w
        )

        # 比例控制器计算角速度
        ang_z = clip(
            self.cfg.heading_control_stiffness * heading_error,
            self.cfg.ranges.ang_vel_z
        )
    else:
        # 角速度模式：直接使用
        ang_z = clip(value_z, self.cfg.ranges.ang_vel_z)

    # 4. 应用到所有环境
    self.vel_command_b[:, 0] = lin_x
    self.vel_command_b[:, 1] = lin_y
    self.vel_command_b[:, 2] = ang_z
```

**Socket 接收循环**：

```python
def _socket_receive_loop(self):
    while True:
        # 1. 接收 UDP 数据
        data, addr = self._socket.recvfrom(1024)
        message = data.decode('utf-8').strip()

        # 2. 解析命令
        parts = message.split(',')

        if len(parts) == 3:
            # 标准格式: "lin_x,lin_y,ang_z"
            lin_x = float(parts[0])
            lin_y = float(parts[1])
            value_z = float(parts[2])

            with self._command_lock:
                self._current_command = [lin_x, lin_y, value_z, value_z]

        elif len(parts) == 4:
            # 扩展格式: "lin_x,lin_y,ang_z,heading"
            lin_x = float(parts[0])
            lin_y = float(parts[1])
            ang_z = float(parts[2])
            heading = float(parts[3])

            with self._command_lock:
                self._current_command = [lin_x, lin_y, ang_z, heading]
```

### 2. Socket 配置类 (`socket_velocity_command_cfg.py`)

```python
@configclass
class SocketVelocityCommandCfg(CommandTermCfg):
    """配置类"""

    class_type: type = SocketVelocityCommand

    asset_name: str = MISSING
    port: int = 5555
    heading_command: bool = False
    heading_control_stiffness: float = 0.5
    debug_vis: bool = True

    @configclass
    class Ranges:
        lin_vel_x: tuple[float, float] = (-1.0, 1.0)
        lin_vel_y: tuple[float, float] = (-1.0, 1.0)
        ang_vel_z: tuple[float, float] = (-1.0, 1.0)
        heading: tuple[float, float] = (-math.pi, math.pi)

    ranges: Ranges = MISSING
```

### 3. 环境配置修改 (`walk_flat_env_cfg.py`)

**修改前**：
```python
# 使用内部随机命令生成器
self.commands.base_velocity = mdp.UniformVelocityCommandCfg(
    asset_name="robot",
    resampling_time_range=(10.0, 10.0),
    ranges=mdp.UniformVelocityCommandCfg.Ranges(
        lin_vel_x=(-1.0, 1.0),
        lin_vel_y=(-1.0, 1.0),
        ang_vel_z=(-1.0, 1.0),
        heading=(-math.pi, math.pi)
    ),
)
```

**修改后**：
```python
# 使用 Socket 外部命令
self.commands.base_velocity = SocketVelocityCommandCfg(
    asset_name="robot",
    port=5555,
    heading_command=False,
    heading_control_stiffness=0.5,
    debug_vis=True,
    ranges=SocketVelocityCommandCfg.Ranges(
        lin_vel_x=(-1.0, 1.0),
        lin_vel_y=(-1.0, 1.0),
        ang_vel_z=(-1.0, 1.0),
        heading=(-math.pi, math.pi),
    ),
)
```

### 4. 模块导出修改 (`mdp/__init__.py`)

**修改前**：
```python
from isaaclab.envs.mdp import *
from .curriculums import *
from .rewards import *
from .terminations import *
```

**修改后**：
```python
from isaaclab.envs.mdp import *
from .curriculums import *
from .rewards import *
from .terminations import *
from .socket_velocity_command import *      # 新增
from .socket_velocity_command_cfg import *  # 新增
```

---

## 工作流程

### 启动流程

1. **Isaac Lab 启动**
   ```
   play.py → 加载 VelocityGo2WalkFlatEnvCfg_Ros 配置
        ↓
        创建 CommandManager
        ↓
        创建 SocketVelocityCommand 实例
        ↓
        __init__: 启动 Socket 服务器和接收线程
        ↓
        开始仿真循环
   ```

2. **Socket 初始化**
   ```
   SocketVelocityCommand.__init__()
        ↓
        创建 UDP Socket
        ↓
        绑定到 127.0.0.1:5555
        ↓
        启动接收线程 (daemon)
        ↓
        等待命令...
   ```

3. **仿真循环**
   ```
   每个仿真步 (约 0.2 秒):
        ↓
        CommandManager.compute()
        ↓
        SocketVelocityCommand._update_command()
        ↓
        读取 _current_command
        ↓
        处理 (角速度/航向)
        ↓
        更新 vel_command_b
        ↓
        机器人执行动作
   ```

### 命令接收流程

```
发送程序:
    send_command(0.5, 0.0, 0.0)
        ↓
    创建 Socket
        ↓
    发送 "0.5,0.0,0.0" 到 127.0.0.1:5555
        ↓
    关闭 Socket

接收线程:
    接收数据
        ↓
    解析 "0.5,0.0,0.0"
        ↓
    更新 _current_command = [0.5, 0.0, 0.0, 0.0]
        ↓
    继续监听...

主循环:
    读取 _current_command
        ↓
    处理并裁剪
        ↓
    更新 vel_command_b[:, 0] = 0.5
    vel_command_b[:, 1] = 0.0
    vel_command_b[:, 2] = 0.0
        ↓
    应用到所有环境
```

---

## 配置说明

### 关键参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | int | 5555 | UDP 监听端口 |
| `heading_command` | bool | False | True=航向模式, False=角速度模式 |
| `heading_control_stiffness` | float | 0.5 | 航向控制增益 |
| `debug_vis` | bool | True | 是否显示调试可视化 |
| `ranges.lin_vel_x` | tuple | (-1.0, 1.0) | 前后速度范围 (m/s) |
| `ranges.lin_vel_y` | tuple | (-1.0, 1.0) | 左右速度范围 (m/s) |
| `ranges.ang_vel_z` | tuple | (-1.0, 1.0) | 角速度范围 (rad/s) |
| `ranges.heading` | tuple | (-π, π) | 航向范围 (rad) |

### 角速度模式配置

```python
SocketVelocityCommandCfg(
    heading_command=False,  # 关键！
    heading_control_stiffness=0.5,  # 此模式下不使用
    ...
)
```

**命令格式**：`"lin_x,lin_y,ang_z"`
- 第 3 个值直接控制角速度

### 航向模式配置

```python
SocketVelocityCommandCfg(
    heading_command=True,   # 关键！
    heading_control_stiffness=0.5,  # 控制转向速度
    ...
)
```

**命令格式**：`"lin_x,lin_y,heading"`
- 第 3 个值是目标航向
- 角速度自动计算：`ang_z = k * heading_error`

---

## 使用方法

### 启动仿真

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python scripts/rsl_rl/play.py \
    --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
    --checkpoint /path/to/model.pt
```

预期输出：
```
============================================================
✓ Socket server initialized on port 5555
✓ Listening for commands on 127.0.0.1:5555
============================================================
SocketVelocityCommand initialized: listening on port 5555 (angular velocity mode)
```

### 发送命令

**使用最小测试工具**：
```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe/Socket
python send_cmd.py
```

输出：
```
Minimal Socket Command Sender
Sending to 127.0.0.1:5555
----------------------------------------
Sent: 0.5,0.0,0.0    # 前进
Sent: 0.0,0.0,0.5    # 左转
Sent: 0.0,0.0,0.0    # 停止
----------------------------------------
Done!
```

**使用 Python 脚本**：
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"0.5,0.0,0.0", ('127.0.0.1', 5555))
sock.close()
```

**使用 nc (netcat)**：
```bash
echo "0.5,0.0,0.0" | nc -u 127.0.0.1 5555
```

---

## 代码逻辑详解

### 1. 线程安全机制

**问题**：主循环和接收线程同时访问 `_current_command`

**解决方案**：使用 `threading.Lock`

```python
# 接收线程中
with self._command_lock:
    self._current_command = [lin_x, lin_y, value_z, heading]

# 主循环中
with self._command_lock:
    lin_x = self._current_command[0]
    lin_y = self._current_command[1]
    value_z = self._current_command[2]
```

### 2. 命令缓冲区设计

**结构**：
```python
_current_command = [lin_x, lin_y, ang_z_or_heading, heading]
#                   [0],    [1],    [2],             [3]
```

**双模式支持**：
- 角速度模式：`_current_command[2]` 是 ang_z
- 航向模式：`_current_command[2]` 是 heading

**处理逻辑**：
```python
value_z = self._current_command[2]

if self.cfg.heading_command:
    # 航向模式：value_z 是目标航向
    heading = value_z
    # 计算角速度...
else:
    # 角速度模式：value_z 是角速度
    ang_z = value_z
```

### 3. 航向控制算法

**数学原理**：

```python
# 1. 计算航向误差（考虑角度环绕）
heading_error = wrap_to_pi(target_heading - current_heading)
#                 [-π, π] 范围

# 2. 比例控制
ang_z = k * heading_error

# 3. 裁剪到角速度范围
ang_z = clip(ang_z, min_ang_vel_z, max_ang_vel_z)
```

**示例**：
```
当前航向: 0° (0 rad)
目标航向: 90° (1.57 rad)
heading_error = 1.57 rad
k = 0.5
ang_z = 0.5 * 1.57 = 0.785 rad/s

机器人会以 0.785 rad/s 的速度左转，直到达到目标航向
```

### 4. 命令应用机制

**关键点**：命令应用到**所有环境**

```python
# vel_command_b shape: (num_envs, 3)
self.vel_command_b[:, 0] = lin_x  # 所有环境的 x 速度
self.vel_command_b[:, 1] = lin_y  # 所有环境的 y 速度
self.vel_command_b[:, 2] = ang_z  # 所有环境的角速度
```

这意味着如果有 50 个并行环境，它们都会接收相同的命令。

### 5. Socket 生命周期

**创建**：`__init__` 中创建并绑定
```python
self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
self._socket.bind(('127.0.0.1', self.cfg.port))
```

**接收循环**：独立线程中持续运行
```python
def _socket_receive_loop(self):
    while True:
        data, _ = self._socket.recvfrom(1024)
        # 处理数据...
```

**关闭**：`__del__` 中清理
```python
def _shutdown_socket(self):
    if self._socket:
        self._socket.close()
```

**Daemon 线程**：
```python
self._socket_thread = threading.Thread(
    target=self._socket_receive_loop,
    daemon=True  # 主线程退出时自动结束
)
```

### 6. 数据流时序

```
时刻 t0: 发送 "0.5,0.0,0.0"
         ↓ (UDP 传输，<1ms)
      接收线程接收
         ↓
      更新 _current_command = [0.5, 0.0, 0.0, 0.0]
         ↓ (等待下一个仿真步)

时刻 t1: 仿真步开始
         ↓
      _update_command() 调用
         ↓
      读取 _current_command = [0.5, 0.0, 0.0, 0.0]
         ↓
      处理并裁剪
         ↓
      更新 vel_command_b[:, :] = [0.5, 0.0, 0.0]
         ↓
      机器人执行动作
```

---

## 关键设计决策

### 1. 为什么使用 UDP 而不是 TCP？

**选择 UDP 的原因**：
- ✅ **低延迟**：无需连接建立
- ✅ **简单**：不需要处理连接状态
- ✅ **容错**：丢包影响小（下次命令会纠正）
- ✅ **轻量**：代码实现简单

**适用场景**：
- 本地通信（同一台机器）
- 高频命令发送
- 对可靠性要求不高

### 2. 为什么使用独立线程？

**问题**：如果阻塞式接收，会卡住仿真循环

**解决**：
- 主线程：仿真循环（高优先级）
- 接收线程：持续监听（后台运行）

**好处**：
- 接收命令不影响仿真性能
- 命令实时性高
- 代码结构清晰

### 3. 为什么需要命令缓冲区？

**问题**：线程间数据传递

**方案对比**：

| 方案 | 优点 | 缺点 |
|------|------|------|
| Queue | 线程安全，有缓冲 | 稍复杂 |
| Lock + List | 简单，最新命令覆盖 | 旧命令丢失 |

**选择**：Lock + List
- 速度命令只需要最新值
- 旧命令被覆盖符合预期
- 实现简单高效

### 4. 为什么支持两种模式？

**角速度模式**：
- 优点：直观，手动控制方便
- 缺点：需要持续发送命令

**航向模式**：
- 优点：精确控制朝向，适合导航
- 缺点：需要比例控制器

**使用场景**：
- 角速度模式：键盘控制、实时遥操作
- 航向模式：自主导航、路径跟踪

---

## 总结

### 实现的核心功能

1. ✅ **UDP Socket 服务器**：监听 5555 端口接收命令
2. ✅ **双模式支持**：角速度模式和航向模式
3. ✅ **线程安全**：独立线程接收，lock 保护共享数据
4. ✅ **命令裁剪**：自动限制在配置范围内
5. ✅ **无缝集成**：完全兼容 Isaac Lab 架构

### 代码关键点

1. **继承 CommandTerm**：符合 Isaac Lab 扩展机制
2. **线程接收**：使用 daemon 线程持续监听
3. **缓冲区设计**：4 个值存储不同模式数据
4. **比例控制**：航向模式下的控制算法
5. **全局应用**：命令应用到所有并行环境

### 修改的文件

- ✅ `mdp/__init__.py`：导出 Socket 相关类
- ✅ `mdp/socket_velocity_command.py`：核心实现
- ✅ `mdp/socket_velocity_command_cfg.py`：配置类
- ✅ `walk_flat_env_cfg.py`：使用 Socket 命令
- ✅ `Socket/send_cmd.py`：测试工具

### 数据流

```
外部程序 → UDP → SocketVelocityCommand → vel_command_b → 机器人
```

### 扩展性

该实现可以轻松扩展：
- 添加认证：检查命令来源
- 添加协议：JSON、二进制等
- 添加反馈：发送机器人状态
- 添加多机器人：不同端口或 ID
