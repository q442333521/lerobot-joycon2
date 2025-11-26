# PAROL6 Integration Guide

Complete guide for integrating PAROL6 6-DOF robotic arm with LeRobot and ROS2.

## 项目概述 (Project Overview)

PAROL6 是一款类似 UR5 的 6 自由度机械臂。本项目为 PAROL6 提供了完整的 LeRobot 集成，包括：

- **LeRobot 机器人配置**：完整的 PAROL6 机器人类
- **ROS2 适配器**：与 ROS2 生态系统的无缝集成
- **Robosuite 仿真**：使用 UR5e 模型进行仿真测试
- **测试套件**：完整的测试代码和示例

PAROL6 is a 6-DOF robotic arm similar to UR5. This project provides complete LeRobot integration including:

- **LeRobot Robot Configuration**: Complete PAROL6 robot class
- **ROS2 Adapter**: Seamless integration with ROS2 ecosystem
- **Robosuite Simulation**: Simulation testing using UR5e model
- **Test Suite**: Complete test code and examples

## 项目结构 (Project Structure)

```
lerobot-joycon2/
├── lerobot/common/robots/parol6/     # PAROL6 机器人配置
│   ├── __init__.py
│   ├── config_parol6.py              # 机器人配置类
│   ├── parol6.py                     # 机器人主类
│   └── README.md                     # 详细文档
│
├── ros2_adapter/                      # ROS2 适配器
│   ├── parol6_adapter/
│   │   ├── __init__.py
│   │   ├── parol6_ros2_bridge.py    # ROS2 桥接类
│   │   └── parol6_ros2_node.py      # ROS2 节点
│   ├── launch/
│   │   └── parol6_bringup.launch.py # ROS2 启动文件
│   ├── setup.py
│   └── README.md
│
├── scripts/parol6/                    # 启动和测试脚本
│   ├── start_parol6_ros2.sh         # ROS2 启动脚本
│   └── test_parol6_lerobot.py       # LeRobot 测试
│
├── tests/parol6/                      # 测试代码
│   ├── test_parol6_robosuite.py     # Robosuite 仿真测试
│   └── test_ros2_integration.py     # ROS2 集成测试
│
└── PAROL6_INTEGRATION.md             # 本文档
```

## 快速开始 (Quick Start)

### 1. 安装依赖 (Installation)

```bash
# 激活 lerobot 环境
conda activate lerobot

# 安装 LeRobot (如果还未安装)
pip install -e .

# 安装 ROS2 适配器
cd ros2_adapter
pip install -e .
cd ..

# 安装 Robosuite (用于仿真测试)
pip install robosuite
```

### 2. ROS2 安装 (Optional, for ROS2 integration)

```bash
# Ubuntu 22.04 安装 ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-sensor-msgs

# 添加到环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 基础测试 (Basic Testing)

#### 测试 LeRobot 集成 (Test LeRobot Integration)

```bash
# 连接测试 (需要真实机器人)
python scripts/parol6/test_parol6_lerobot.py --test connection

# 如果没有真实机器人，可以跳过此测试
```

#### 测试 Robosuite 仿真 (Test Robosuite Simulation)

```bash
# 运行 Robosuite 仿真测试
python tests/parol6/test_parol6_robosuite.py --test all

# 单独测试基础环境
python tests/parol6/test_parol6_robosuite.py --test basic

# 测试 PAROL6 集成
python tests/parol6/test_parol6_robosuite.py --test integration
```

#### 测试 ROS2 集成 (Test ROS2 Integration)

```bash
# 确保 ROS2 已 source
source /opt/ros/humble/setup.bash

# 运行 ROS2 测试
python tests/parol6/test_ros2_integration.py --test all
```

## 使用方法 (Usage)

### 方式 1: 直接使用 LeRobot (Without ROS2)

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

# 创建配置
config = Parol6Config(
    id="my_parol6",
    port="/dev/ttyUSB0",  # 根据实际情况修改
    baudrate=115200,
)

# 连接机器人
robot = Parol6(config)
robot.connect()

# 读取观测
obs = robot.get_observation()
print(f"当前关节位置: {obs}")

# 发送动作
action = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": -1.57,
    "elbow.pos": 1.57,
    "wrist_1.pos": -1.57,
    "wrist_2.pos": -1.57,
    "wrist_3.pos": 0.0,
}
robot.send_action(action)

# 断开连接
robot.disconnect()
```

### 方式 2: 使用 ROS2 集成 (With ROS2)

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

# 启用 ROS2 桥接
config = Parol6Config(
    id="my_parol6",
    use_ros2_bridge=True,
    ros2_namespace="/parol6",
    control_frequency=100,
)

# 其余使用方法相同
robot = Parol6(config)
robot.connect()
# ...
robot.disconnect()
```

### 方式 3: Robosuite 仿真 (Robosuite Simulation)

```python
import robosuite as suite
import numpy as np

# 创建环境 (使用 UR5e 模拟 PAROL6)
env = suite.make(
    env_name="Lift",
    robots="UR5e",  # UR5e 与 PAROL6 类似
    has_renderer=True,
    control_freq=20,
)

# 重置环境
obs = env.reset()

# 运行控制循环
for i in range(1000):
    # PAROL6 风格的动作 (6 关节 + 夹爪)
    action = np.random.randn(7) * 0.1
    obs, reward, done, info = env.step(action)
    env.render()

    if done:
        obs = env.reset()

env.close()
```

## ROS2 集成详解 (ROS2 Integration Details)

### 启动 ROS2 节点 (Start ROS2 Node)

```bash
# 使用启动脚本 (推荐)
./scripts/parol6/start_parol6_ros2.sh sim /parol6

# 或直接使用 launch 文件
source /opt/ros/humble/setup.bash
ros2 launch ros2_adapter/launch/parol6_bringup.launch.py \
    use_sim:=false \
    namespace:=/parol6
```

### ROS2 话题 (Topics)

#### 发布的话题 (Published)

- `/parol6/joint_states` - 关节状态 (位置、速度、力矩)
- `/parol6/joint_trajectory_controller/state` - 控制器状态

#### 订阅的话题 (Subscribed)

- `/parol6/joint_trajectory_controller/joint_trajectory` - 轨迹命令
- `/parol6/joint_commands` - 直接位置命令

### 监控和控制 (Monitoring and Control)

```bash
# 查看关节状态
ros2 topic echo /parol6/joint_states

# 发送位置命令
ros2 topic pub /parol6/joint_commands sensor_msgs/msg/JointState \
    "{name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
    position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}"
```

## 与现有 LeRobot 工作流集成 (Integration with LeRobot Workflow)

### 遥操作 (Teleoperation)

```bash
# 使用 Joycon 遥控 PAROL6
python -m lerobot.teleoperate \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --teleop.type=joycon
```

### 数据采集 (Data Recording)

```bash
python -m lerobot.record \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --robot.cameras="{ \
        OBS_IMAGE_1: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30} \
    }" \
    --dataset.single_task="抓取并放置橙色方块" \
    --dataset.repo_id=Datasets/parol6_grasp_put \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.num_episodes=50 \
    --teleop.type=joycon
```

### 训练模型 (Training)

```bash
# 使用 SmolVLA 训练
python lerobot/scripts/train.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.repo_id=Datasets/parol6_grasp_put \
    --batch_size=8 \
    --steps=20000 \
    --output_dir=outputs/train/parol6_smolvla \
    --job_name=parol6_training \
    --policy.device=cuda
```

### 推理测试 (Inference)

```bash
python -m lerobot.record \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --robot.cameras="{ \
        OBS_IMAGE_1: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30} \
    }" \
    --dataset.single_task="抓取并放置橙色方块" \
    --dataset.repo_id=Datasets/eval_parol6 \
    --policy.path=outputs/train/parol6_smolvla/checkpoints/last/pretrained_model
```

## 仿真测试流程 (Simulation Testing Workflow)

推荐在 Robosuite 仿真环境中先测试算法，再部署到真实机器人：

### 1. 在 Robosuite 中开发和测试

```python
# 使用 UR5e 作为 PAROL6 的代理
import robosuite as suite

env = suite.make(
    env_name="Lift",  # 或其他任务
    robots="UR5e",
    has_renderer=True,
)

# 测试你的控制算法
# ...
```

### 2. 在仿真中验证策略

```python
# 加载训练好的策略
from your_policy import Policy

policy = Policy.load("path/to/checkpoint")

obs = env.reset()
for i in range(1000):
    action = policy.predict(obs)
    obs, reward, done, info = env.step(action)
    env.render()
```

### 3. 部署到真实 PAROL6

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

robot = Parol6(Parol6Config(id="real", port="/dev/ttyUSB0"))
robot.connect()

# 使用相同的策略
obs = robot.get_observation()
action = policy.predict(obs)
robot.send_action(action)
```

## 故障排除 (Troubleshooting)

### 串口连接问题

```bash
# 检查设备
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录或重启

# 临时设置权限
sudo chmod 666 /dev/ttyUSB0
```

### ROS2 问题

```bash
# 检查 ROS2 是否 source
echo $ROS_DISTRO

# 重新 source
source /opt/ros/humble/setup.bash

# 检查节点
ros2 node list

# 检查话题
ros2 topic list
```

### Robosuite 渲染问题

```bash
# 如果遇到 GLX 错误，切换到独立显卡
sudo prime-select nvidia
sudo reboot

# 或使用离屏渲染
env = suite.make(
    ...,
    has_renderer=False,
    has_offscreen_renderer=True,
)
```

## 下一步 (Next Steps)

1. **创建 URDF 模型**：为 PAROL6 创建 URDF 文件以支持 RViz 可视化
2. **MoveIt2 集成**：集成 MoveIt2 进行运动规划
3. **Gazebo 仿真**：创建 Gazebo 仿真环境
4. **多机器人支持**：支持多个 PAROL6 协同工作

## 参考资料 (References)

- [LeRobot](https://github.com/huggingface/lerobot)
- [PAROL6 Robot](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm)
- [Robosuite](https://robosuite.ai/)
- [ROS2 Documentation](https://docs.ros.org/)

## 支持 (Support)

如有问题，请创建 GitHub Issue 或联系开发团队。

For questions, please create a GitHub issue or contact the development team.
