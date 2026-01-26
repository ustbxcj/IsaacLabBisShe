# ROS2 Bridge for IsaacSim - Unitree Go2

通过发布ROS2话题直接控制IsaacSim中的Unitree Go2四足机器人。

## 🚀 快速开始（3个终端）

### 终端1 - ROS2订阅者
```bash
conda activate ros2_env
cd /home/xcj/work/IsaacLab/BiShe/MyProject/Ros2_4_Isaacsim
python ros2_velocity_subscriber.py
```

### 终端2 - ROS2发布者（设置速度）
```bash
conda activate ros2_env
cd /home/xcj/work/IsaacLab/BiShe/MyProject/Ros2_4_Isaacsim
python ros2_velocity_publisher.py --vel-x 0.5
```

### 终端3 - Isaac Sim
```bash
conda activate env_isaaclab
cd /home/xcj/work/IsaacLab/BiShe/MyProject/Ros2_4_Isaacsim
./run_isaac.sh
```

## 📝 核心文件

| 文件 | 说明 |
|------|------|
| `ros2_velocity_publisher.py` | ROS2发布者，发布速度命令 |
| `ros2_velocity_subscriber.py` | ROS2订阅者，桥接到Isaac Sim |
| `play_ros2.py` | 集成ROS2的播放脚本 |
| `run_isaac.sh` | Isaac Sim启动脚本 |

## 🎮 控制方式

### 命令行参数控制

```bash
# 前进 0.5 m/s
python ros2_velocity_publisher.py --vel-x 0.5

# 后退 0.3 m/s
python ros2_velocity_publisher.py --vel-x -0.3

# 左转 0.2 rad/s
python ros2_velocity_publisher.py --vel-z 0.2

# 右转 0.2 rad/s
python ros2_velocity_publisher.py --vel-z -0.2

# 左右平移 0.3 m/s
python ros2_velocity_publisher.py --vel-y 0.3

# 停止
python ros2_velocity_publisher.py

# 组合：前进+旋转
python ros2_velocity_publisher.py --vel-x 0.5 --vel-z 0.1
```

### 参数说明

- `--vel-x`: 前进/后退速度（m/s，范围 -1.0 到 1.0）
- `--vel-y`: 左右平移速度（m/s，范围 -1.0 到 1.0）
- `--vel-z`: 旋转速度（rad/s，范围 -1.0 到 1.0）
- `--rate`: 发布频率（Hz，默认 20）

## 🔧 工作原理

```
你的命令 → ros2_velocity_publisher.py → /cmd_vel 话题
                                            ↓
                            ros2_velocity_subscriber.py
                                            ↓
                                    共享内存
                                            ↓
                                play_ros2.py → Isaac Sim环境
```

**关键特性**：
- ✅ ROS2桥接启用：机器人响应 `/cmd_vel` 命令
- ⚠️ ROS2桥接未启用：机器人保持静止
- 🎯 完全覆盖默认的随机命令生成

## 💻 自定义ROS2控制

创建你自己的ROS2节点：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)  # 20Hz

    def publish_command(self):
        msg = Twist()
        msg.linear.x = 0.5   # 你的控制逻辑
        msg.angular.z = 0.2
        self.publisher.publish(msg)

rclpy.init()
node = MyController()
rclpy.spin(node)
```

## 🔍 故障排查

### 机器人不动

**检查步骤**：
1. 确认3个终端都在运行
2. 检查Isaac Sim输出是否显示 `ROS2 Bridge: ENABLED`
3. 检查话题数据：`ros2 topic echo /cmd_vel`

### ROS2桥接未启用

**原因**：`ros2_velocity_subscriber.py` 未运行

**解决**：先启动订阅者，再启动Isaac Sim

### ImportError

**确保在正确的conda环境**：
- ROS2节点 → `ros2_env`
- Isaac Sim → `env_isaaclab`

## 📚 参考资料

- Isaac Lab: https://isaac-sim.github.io/IsaacLab/
- ROS2文档: https://docs.ros.org/en/humble/
- Unitree Go2: https://github.com/unitreerobotics/unitree_ros2
