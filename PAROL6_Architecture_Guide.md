# PAROL6 Architecture Diagram Guide

## 架构图说明 (Architecture Diagram Description)

本架构图展示了 PAROL6 机器人与 LeRobot 和 ROS2 的完整集成架构。

This architecture diagram shows the complete integration of PAROL6 robot with LeRobot and ROS2.

## 图层说明 (Layer Description)

### 1. 应用层 (Application Layer) - 蓝色系
**颜色**: 蓝色 (#1976D2, #E3F2FD)
**用途**: LeRobot 工作流程

**模块**:
- **Teleoperate** (遥操作)
  - File: `teleoperate.py`
  - Function: `main()`
  - 支持 Joycon 和键盘控制

- **Record** (数据采集)
  - File: `record.py`
  - Function: `record_episode()`
  - 配置: episode_time_s: 30, num_episodes: 50

- **Train** (训练)
  - File: `train.py`
  - Function: `train()`
  - 配置: policy: smolvla_base, batch_size: 8, steps: 20000

- **Inference** (推理)
  - File: `record.py`
  - Function: `inference()`
  - 使用训练好的策略

- **Simulation** (仿真) - 绿色
  - File: `test_parol6_robosuite.py`
  - 使用 Robosuite (UR5e 作为代理)

### 2. 机器人接口层 (Robot Interface Layer) - 橙色系
**颜色**: 橙色 (#FF6F00, #FFF3E0)
**用途**: PAROL6 机器人类和配置

**模块**:
- **PAROL6 Configuration**
  - File: `config_parol6.py`
  - Class: `Parol6Config`
  - 关键参数:
    - port: `/dev/ttyUSB0`
    - baudrate: `115200`
    - use_ros2_bridge: `bool`
    - max_relative_target: `0.5`
    - control_frequency: `100Hz`
    - joint_limits: `dict`
    - enable_position_filter: `true`
    - position_filter_coeff: `0.8`

- **PAROL6 Robot Class**
  - File: `parol6.py`
  - Class: `Parol6`
  - 主要函数:
    - `connect()` - 初始化连接
    - `disconnect()` - 关闭连接
    - `get_observation()` - 读取状态
    - `send_action()` - 发送命令
    - `calibrate()` - 验证限制

- **Joint Configuration**
  - 6 个关节:
    - shoulder_pan
    - shoulder_lift
    - elbow
    - wrist_1
    - wrist_2
    - wrist_3
  - Home Position: `[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]`

- **Camera Integration**
  - File: `cameras/opencv.py`
  - 配置: 640x480 @ 30fps
  - Function: `async_read()`, `connect()`, `disconnect()`

- **Safety Features**
  - File: `utils.py`
  - Function: `ensure_safe_goal_position()`
  - 检查: 关节限制、最大相对目标、位置裁剪、速度限制

### 3. 通信层 (Communication Layer) - 紫色系
**颜色**: 紫色 (#7B1FA2, #F3E5F5)
**用途**: 与硬件/ROS2 的通信

**模块**:
- **Serial Communication** (串口通信)
  - Library: pyserial
  - 配置:
    - Port: `/dev/ttyUSB0`
    - Baudrate: `115200`
    - Timeout: `1.0s`
  - Protocol:
    - CMD: `MOVE,j0,j1,...,j5`
    - Response: `OK/ERROR`
  - Functions:
    - `_send_joint_positions()`
    - `_read_joint_positions()`

- **ROS2 Bridge**
  - File: `parol6_ros2_bridge.py`
  - Class: `Parol6ROS2Bridge`
  - 配置:
    - namespace: `/parol6`
    - control_frequency: `100Hz`
  - Functions:
    - `connect()` - 初始化 ROS2
    - `disconnect()` - 关闭
    - `send_joint_positions(pos)`
    - `get_joint_positions()` → np.array
    - `get_joint_velocities()` → np.array
    - `_joint_state_callback(pos, vel)`

- **ROS2 Node**
  - File: `parol6_ros2_node.py`
  - Class: `Parol6ROS2Node`
  - Publishers (发布的话题):
    - `/parol6/joint_states` (sensor_msgs/JointState)
    - `/parol6/joint_trajectory_controller/state` (control_msgs/JointTrajectoryControllerState)
  - Subscribers (订阅的话题):
    - `/parol6/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)
    - `/parol6/joint_commands` (sensor_msgs/JointState)
  - Functions:
    - `publish_joint_state()`
    - `joint_command_callback(msg)`
    - `publish_controller_state()`

### 4. 硬件/仿真层 (Hardware & Simulation Layer) - 绿色系
**颜色**: 绿色 (#43A047, #E8F5E9)
**用途**: 实际硬件或仿真环境

**模块**:
- **PAROL6 Hardware**
  - 规格:
    - DOF: 6 (类似 UR5)
    - Max velocity: 3.15 rad/s
    - Max accel: 10 rad/s²
    - Communication: USB/Serial
  - 关节:
    - All joints: ±180° range
    - Position control mode
  - Interface:
    - Serial port: `/dev/ttyUSB0`
    - Baudrate: `115200`

- **Robosuite Simulation**
  - File: `test_parol6_robosuite.py`
  - 配置:
    - env_name: Lift, Stack, etc.
    - robots: UR5e (proxy for PAROL6)
    - control_freq: 20Hz
    - has_renderer: true
  - Functions:
    - `test_robosuite_basic()`
    - `test_parol6_robosuite_integration()`
    - `test_robosuite_teleoperation()`

- **Launch Scripts**
  - Files:
    - `scripts/parol6/start_parol6_ros2.sh`
    - `ros2_adapter/launch/parol6_bringup.launch.py`
  - Tests:
    - `scripts/parol6/test_parol6_lerobot.py`
    - `tests/parol6/test_ros2_integration.py`
  - Examples:
    - `examples/parol6_example.py`

## 数据流向 (Data Flow)

### 1. 遥操作流程 (Teleoperation Flow)
```
User Input (Joycon/Keyboard)
    ↓
Application Layer (Teleoperate)
    ↓
Robot Interface (PAROL6 Class)
    ↓
Communication Layer (Serial or ROS2)
    ↓
Hardware (PAROL6 Robot)
```

### 2. 数据采集流程 (Data Recording Flow)
```
User Control (Teleoperate)
    ↓
Robot Observation (get_observation)
    ↓  ← Camera Images
    ↓  ← Joint Positions
Record Module (save to dataset)
```

### 3. 训练和推理流程 (Training & Inference Flow)
```
Dataset
    ↓
Train Module (SmolVLA)
    ↓
Trained Policy
    ↓
Inference Module
    ↓
Robot Actions
```

### 4. ROS2 通信流程 (ROS2 Communication Flow)
```
PAROL6 Class (send_action)
    ↓
ROS2 Bridge (send_joint_positions)
    ↓
ROS2 Node (publish to topics)
    ↓
ROS2 Topic: /parol6/joint_commands
    ↓
Hardware or Other ROS2 Nodes
```

## 配色方案说明 (Color Scheme)

### 主要配色 (Primary Colors)
- **蓝色系 (Blue)**: 应用层
  - 主色: #1976D2 (Material Design Blue 700)
  - 背景: #E3F2FD (Material Design Blue 50)
  - 文字: #FFFFFF (白色文字在深色背景上)

- **橙色系 (Orange)**: 机器人接口层
  - 主色: #FF6F00 (Material Design Orange A700)
  - 背景: #FFF3E0 (Material Design Orange 50)
  - 文字: #FFFFFF (白色)

- **紫色系 (Purple)**: 通信层
  - 主色: #7B1FA2 (Material Design Purple 700)
  - 背景: #F3E5F5 (Material Design Purple 50)
  - 文字: #FFFFFF (白色)

- **绿色系 (Green)**: 硬件/仿真层
  - 主色: #43A047 (Material Design Green 600)
  - 背景: #E8F5E9 (Material Design Green 50)
  - 文字: #FFFFFF (白色)

### 文字配色 (Text Colors)
- **标题文字**: #1A1A1A (深灰色，高对比度)
- **模块内文字**: #FFFFFF (白色，在深色背景上)
- **连线标签**: 各层主色 (与对应层级匹配)

### 连线配色 (Arrow Colors)
- **应用到机器人**: #1976D2 (蓝色)
- **机器人内部**: #FF6F00 (橙色，虚线)
- **机器人到通信**: #7B1FA2 (紫色)
- **通信到硬件**: #43A047 (绿色)
- **仿真相关**: #43A047 (绿色)

## 布局优化 (Layout Optimization)

### 1. 层级分离
- 每层之间间隔 40px，避免拥挤
- 层内模块间隔 20-40px

### 2. 连线走向
- 使用 orthogonalEdgeStyle (正交样式)
- 使用 curved=1 (圆角)
- 避免穿过其他模块
- 标签背景设为白色，提高可读性

### 3. 模块大小
- 根据内容动态调整
- 关键模块 (Robot Class, ROS2 Node) 较大
- 次要模块适中

### 4. 图例位置
- 位于底部
- 包含所有层级的颜色说明
- 包含关键信息和快速参考

## 如何使用架构图 (How to Use)

### 1. 在 draw.io 中打开
```bash
# 在线打开
访问: https://app.diagrams.net/
File → Open → 选择 PAROL6_Architecture.drawio

# 或使用 draw.io Desktop
下载并安装 draw.io
打开 PAROL6_Architecture.drawio 文件
```

### 2. 导出为图片
```
File → Export as → PNG/SVG/PDF
选择分辨率 (推荐 300 DPI)
导出
```

### 3. 编辑和自定义
- 双击任何模块编辑文本
- 拖动模块调整位置
- 右键模块可修改颜色和样式
- 点击连线可调整路径

## 关键特性标注 (Key Features Highlighted)

### 文件名标注
- 每个模块都标注了对应的源文件
- 便于快速定位代码

### 函数名标注
- 列出了关键函数和方法
- 包含参数和返回值说明

### 配置参数标注
- 详细的配置参数说明
- 包含默认值和推荐值

### 数据流标注
- 清晰的箭头指示数据流向
- 标签说明数据传输的内容

## 技术亮点 (Technical Highlights)

1. **双模式通信**: 支持串口和 ROS2 两种通信方式
2. **安全机制**: 多重安全检查（关节限制、位置过滤等）
3. **高频控制**: 100Hz 控制频率
4. **仿真支持**: Robosuite 环境测试
5. **完整工作流**: 从遥操作到训练推理的完整流程

## 参考文档 (Reference Documentation)

- [PAROL6_INTEGRATION.md](PAROL6_INTEGRATION.md) - 完整集成指南
- [PAROL6_PROJECT_SUMMARY.md](PAROL6_PROJECT_SUMMARY.md) - 项目总结
- [lerobot/common/robots/parol6/README.md](lerobot/common/robots/parol6/README.md) - PAROL6 文档
- [ros2_adapter/README.md](ros2_adapter/README.md) - ROS2 适配器文档

---

**版本**: 1.0
**最后更新**: 2025-11-17
**作者**: Claude Code
