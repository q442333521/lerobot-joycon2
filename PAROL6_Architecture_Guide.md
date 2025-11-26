# PAROL6 架构图说明文档

## 架构图概述

本架构图展示了 PAROL6 机器人与 LeRobot 和 ROS2 的完整集成架构，包含详细的文件、函数和配置参数标注。

## 图层说明

### 第1层：应用层 - 蓝色系
**颜色**: 蓝色 (#1976D2, #E3F2FD)
**用途**: LeRobot 工作流程

**模块说明**:

#### 1. 遥操作
- **文件**: `teleoperate.py`
- **函数**: `main()`
- **功能**:
  - Joycon 输入控制
  - 键盘控制

#### 2. 数据采集
- **文件**: `record.py`
- **函数**: `record_episode()`
- **配置参数**:
  - episode_time_s: 30
  - num_episodes: 50

#### 3. 模型训练
- **文件**: `train.py`
- **函数**: `train()`
- **配置参数**:
  - policy: smolvla_base
  - batch_size: 8
  - steps: 20000

#### 4. 模型推理
- **文件**: `record.py`
- **函数**: `inference()`
- **配置参数**:
  - policy.path: checkpoint
  - use_policy: true

#### 5. 仿真环境（绿色）
- **文件**: `test_parol6_robosuite.py`
- **函数**: `test_robosuite_basic()`
- **环境配置**:
  - Robosuite (UR5e 作为 PAROL6 代理)
  - 控制频率: 20Hz

### 第2层：机器人接口层 - 橙色系
**颜色**: 橙色 (#FF6F00, #FFF3E0)
**用途**: PAROL6 机器人类和配置

**模块说明**:

#### 1. PAROL6 配置
- **文件**: `config_parol6.py`
- **类**: `Parol6Config`
- **关键参数**:
  - `port`: /dev/ttyUSB0 - 串口设备路径
  - `baudrate`: 115200 - 波特率
  - `use_ros2_bridge`: bool - 是否使用ROS2桥接
  - `max_relative_target`: 0.5 - 最大相对移动距离
  - `control_frequency`: 100Hz - 控制频率
  - `joint_limits`: dict - 关节限制字典
  - `enable_position_filter`: true - 启用位置过滤
  - `position_filter_coeff`: 0.8 - 位置过滤系数

#### 2. PAROL6 机器人类
- **文件**: `parol6.py`
- **类**: `Parol6`
- **主要函数**:
  - `connect()` - 初始化连接
  - `disconnect()` - 关闭连接
  - `get_observation()` - 读取机器人状态
  - `send_action()` - 发送控制命令
  - `calibrate()` - 校准和验证限制
- **特性**:
  - 6自由度关节控制
  - 相机集成
  - 安全限制
  - 位置过滤

#### 3. 关节配置
- **6个关节**:
  - `shoulder_pan` - 肩部旋转
  - `shoulder_lift` - 肩部抬升
  - `elbow` - 肘关节
  - `wrist_1` - 腕关节1
  - `wrist_2` - 腕关节2
  - `wrist_3` - 腕关节3
- **初始位置**: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

#### 4. 相机集成
- **文件**: `cameras/opencv.py`
- **配置参数**:
  - type: opencv
  - width: 640
  - height: 480
  - fps: 30
- **函数**:
  - `async_read()` - 异步读取图像
  - `connect()` - 连接相机
  - `disconnect()` - 断开相机

#### 5. 安全功能
- **文件**: `utils.py`
- **函数**: `ensure_safe_goal_position()`
- **检查项**:
  - 关节限制检查
  - 最大相对目标限制
  - 位置裁剪
  - 速度限制

### 第3层：通信层 - 紫色系
**颜色**: 紫色 (#7B1FA2, #F3E5F5)
**用途**: 与硬件和ROS2的通信

**模块说明**:

#### 1. 串口通信
- **使用库**: pyserial
- **配置**:
  - 端口: /dev/ttyUSB0
  - 波特率: 115200
  - 超时: 1.0s
- **协议**:
  - 命令格式: `MOVE,j0,j1,...,j5`
  - 响应: `OK/ERROR`
- **函数**:
  - `_send_joint_positions()` - 发送关节位置
  - `_read_joint_positions()` - 读取关节位置

#### 2. ROS2 桥接
- **文件**: `parol6_ros2_bridge.py`
- **类**: `Parol6ROS2Bridge`
- **配置**:
  - namespace: /parol6
  - control_frequency: 100Hz
- **函数**:
  - `connect()` - 初始化ROS2连接
  - `disconnect()` - 关闭连接
  - `send_joint_positions(pos)` - 发送关节位置
  - `get_joint_positions()` → np.array - 获取当前位置
  - `get_joint_velocities()` → np.array - 获取当前速度
  - `_joint_state_callback(pos, vel)` - 关节状态回调

#### 3. ROS2 节点
- **文件**: `parol6_ros2_node.py`
- **类**: `Parol6ROS2Node`
- **发布的话题**:
  - `/parol6/joint_states` (sensor_msgs/JointState) - 关节状态
  - `/parol6/joint_trajectory_controller/state` (control_msgs/JointTrajectoryControllerState) - 控制器状态
- **订阅的话题**:
  - `/parol6/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory) - 轨迹命令
  - `/parol6/joint_commands` (sensor_msgs/JointState) - 直接位置命令
- **函数**:
  - `publish_joint_state()` - 发布关节状态
  - `joint_command_callback(msg)` - 处理命令回调
  - `publish_controller_state()` - 发布控制器状态

### 第4层：硬件与仿真层 - 绿色系
**颜色**: 绿色 (#43A047, #E8F5E9)
**用途**: 实际硬件或仿真环境

**模块说明**:

#### 1. PAROL6 硬件
- **规格参数**:
  - 自由度: 6 (类似 UR5)
  - 最大速度: 3.15 rad/s
  - 最大加速度: 10 rad/s²
  - 通信方式: USB/串口
- **关节信息**:
  - 所有关节: ±180° 运动范围
  - 控制模式: 位置控制
- **接口**:
  - 串口: /dev/ttyUSB0
  - 波特率: 115200

#### 2. Robosuite 仿真
- **文件**: `test_parol6_robosuite.py`
- **配置**:
  - env_name: Lift, Stack 等
  - robots: UR5e (作为PAROL6代理)
  - control_freq: 20Hz
  - has_renderer: true
- **函数**:
  - `test_robosuite_basic()` - 基础环境测试
  - `test_parol6_robosuite_integration()` - 集成测试
  - `test_robosuite_teleoperation()` - 遥操作测试

#### 3. 启动脚本
- **文件**:
  - `scripts/parol6/start_parol6_ros2.sh` - ROS2启动脚本
  - `ros2_adapter/launch/parol6_bringup.launch.py` - ROS2启动文件
- **测试文件**:
  - `scripts/parol6/test_parol6_lerobot.py` - LeRobot集成测试
  - `tests/parol6/test_ros2_integration.py` - ROS2集成测试
- **示例文件**:
  - `examples/parol6_example.py` - 使用示例

## 数据流向说明

### 1. 遥操作数据流
```
用户输入 (Joycon/键盘)
    ↓
应用层 (遥操作模块)
    ↓
机器人接口 (PAROL6 类)
    ↓
通信层 (串口或ROS2)
    ↓
硬件 (PAROL6 机器人)
```

### 2. 数据采集流程
```
用户控制 (遥操作)
    ↓
机器人观测 (get_observation)
    ↓  ← 相机图像
    ↓  ← 关节位置
数据记录模块 (保存到数据集)
```

### 3. 训练和推理流程
```
数据集
    ↓
训练模块 (SmolVLA)
    ↓
训练好的策略
    ↓
推理模块
    ↓
机器人动作
```

### 4. ROS2 通信流程
```
PAROL6 类 (send_action)
    ↓
ROS2 桥接 (send_joint_positions)
    ↓
ROS2 节点 (发布到话题)
    ↓
ROS2 话题: /parol6/joint_commands
    ↓
硬件或其他ROS2节点
```

### 5. 仿真测试流程
```
仿真环境 (Robosuite UR5e)
    ↓
测试验证
    ↓
机器人接口 (PAROL6 类)
    ↓
验证算法正确性
```

## 配色方案详解

### 主要配色（Material Design）

#### 蓝色系 - 应用层
- **主色**: #1976D2 (Material Design Blue 700)
- **背景**: #E3F2FD (Material Design Blue 50)
- **文字**: #FFFFFF (白色，高对比度)
- **边框**: #0D47A1 (Blue 900)

#### 橙色系 - 机器人接口层
- **主色**: #FF6F00 (Material Design Orange A700)
- **次色**: #F57C00 (Orange 700)
- **背景**: #FFF3E0 (Material Design Orange 50)
- **文字**: #FFFFFF (白色)
- **边框**: #E65100 (Orange 900)

#### 紫色系 - 通信层
- **主色**: #7B1FA2 (Material Design Purple 700)
- **背景**: #F3E5F5 (Material Design Purple 50)
- **文字**: #FFFFFF (白色)
- **边框**: #4A148C (Purple 900)

#### 绿色系 - 硬件/仿真层
- **主色**: #43A047 (Material Design Green 600)
- **次色**: #66BB6A (Green 400)
- **背景**: #E8F5E9 (Material Design Green 50)
- **文字**: #FFFFFF (白色)
- **边框**: #2E7D32 (Green 800)

### 文字配色规则

- **模块标题**: #FFFFFF (白色，在深色背景上)
- **层级标题**: 各层主色的深色版本
- **页面标题**: #1A1A1A (深灰色，高对比度)
- **图例文字**: #1A1A1A (深灰色)
- **连线标签**: 各层主色，背景白色

### 连线配色

| 连线类型 | 颜色 | 粗细 | 样式 |
|---------|------|------|------|
| 应用→机器人 | #1976D2 (蓝) | 3px | 实线圆角 |
| 机器人内部 | #FF6F00 (橙) | 2px | 虚线圆角 |
| 机器人→通信 | #7B1FA2 (紫) | 3px | 实线圆角 |
| 通信→硬件 | #43A047 (绿) | 3px | 实线圆角 |
| 仿真相关 | #43A047 (绿) | 3px | 实线圆角 |

## 布局优化说明

### 层级间距
```
Y坐标分布:
├── 第1层 (应用层):      100-280px  (高度: 180px)
├── 间隔:                280-320px  (间隔: 40px)
├── 第2层 (机器人层):    320-540px  (高度: 220px)
├── 间隔:                540-580px  (间隔: 40px)
├── 第3层 (通信层):      580-780px  (高度: 200px)
├── 间隔:                780-820px  (间隔: 40px)
├── 第4层 (硬件层):      820-980px  (高度: 160px)
├── 间隔:                980-1020px (间隔: 40px)
└── 图例:                1020-1170px (高度: 150px)
```

### 模块布局原则

1. **避免重叠**
   - 每层模块横向排列
   - 模块间距 20-40px
   - 重要模块（如机器人类）占据更大空间

2. **连线走向**
   - 使用正交样式 (orthogonalEdgeStyle)
   - 启用圆角 (curved=1)
   - 从空白区域走线
   - 避免穿过其他模块
   - 标签背景设为白色

3. **尺寸分配**
   - 内容多的模块：280-380px 宽
   - 标准模块：180-200px 宽
   - 配置模块：200-260px 宽

### 连线路径优化

所有连线都经过精心设计，确保：
- 不穿过其他模块
- 使用曲线拐角增加美观度
- 标签放置在空白处
- 标签有白色背景提高可读性

## 详细标注内容

### 文件名标注列表

每个模块都标注了对应的源文件：

**应用层文件**:
- `teleoperate.py` - 遥操作
- `record.py` - 数据采集和推理
- `train.py` - 模型训练
- `test_parol6_robosuite.py` - 仿真测试

**机器人层文件**:
- `config_parol6.py` - 配置类
- `parol6.py` - 机器人主类
- `cameras/opencv.py` - 相机接口
- `utils.py` - 工具函数

**通信层文件**:
- `parol6_ros2_bridge.py` - ROS2桥接
- `parol6_ros2_node.py` - ROS2节点

**硬件层文件**:
- `start_parol6_ros2.sh` - 启动脚本
- `parol6_bringup.launch.py` - ROS2启动文件
- `test_parol6_lerobot.py` - LeRobot测试
- `test_ros2_integration.py` - ROS2测试
- `parol6_example.py` - 示例代码

### 函数标注列表

**核心函数**:
- `connect()` - 初始化连接
- `disconnect()` - 关闭连接
- `get_observation()` - 读取观测
- `send_action()` - 发送动作
- `calibrate()` - 校准
- `send_joint_positions()` - 发送关节位置
- `get_joint_positions()` - 获取关节位置
- `get_joint_velocities()` - 获取关节速度

**ROS2 函数**:
- `publish_joint_state()` - 发布关节状态
- `joint_command_callback(msg)` - 命令回调
- `publish_controller_state()` - 发布控制器状态
- `_joint_state_callback(pos, vel)` - 状态回调

**测试函数**:
- `test_robosuite_basic()` - 基础测试
- `test_parol6_robosuite_integration()` - 集成测试
- `test_robosuite_teleoperation()` - 遥操作测试
- `record_episode()` - 采集数据
- `train()` - 训练模型
- `inference()` - 推理

### 配置参数列表

**机器人配置**:
- `port`: /dev/ttyUSB0 - 设备端口
- `baudrate`: 115200 - 通信波特率
- `use_ros2_bridge`: bool - ROS2模式开关
- `max_relative_target`: 0.5 - 安全移动限制
- `control_frequency`: 100Hz - 控制频率
- `joint_limits`: dict - 关节角度限制
- `enable_position_filter`: true - 位置平滑
- `position_filter_coeff`: 0.8 - 平滑系数

**训练配置**:
- `policy`: smolvla_base - 策略模型
- `batch_size`: 8 - 批次大小
- `steps`: 20000 - 训练步数

**采集配置**:
- `episode_time_s`: 30 - 每轮时长
- `num_episodes`: 50 - 总轮数

**相机配置**:
- `type`: opencv - 相机类型
- `width`: 640 - 图像宽度
- `height`: 480 - 图像高度
- `fps`: 30 - 帧率

**仿真配置**:
- `env_name`: Lift, Stack - 环境名称
- `robots`: UR5e - 机器人型号
- `control_freq`: 20Hz - 控制频率

## 如何使用架构图

### 1. 在 Draw.io 中打开

**在线版本（推荐）**:
```
1. 访问: https://app.diagrams.net/
2. 点击 File → Open from... → Device
3. 选择: PAROL6_Architecture.drawio
4. 即可查看和编辑
```

**桌面版本**:
```
1. 下载 Draw.io Desktop
   https://github.com/jgraph/drawio-desktop/releases
2. 安装并启动
3. File → Open
4. 选择: PAROL6_Architecture.drawio
```

### 2. 导出为图片

**PNG 格式**:
```
File → Export as → PNG
- 选择分辨率: 300 DPI (推荐)
- 勾选: Transparent Background (可选)
- 勾选: Include a copy of my diagram
导出完成
```

**SVG 格式**:
```
File → Export as → SVG
- SVG格式矢量图，无损缩放
- 适合嵌入网页或文档
```

**PDF 格式**:
```
File → Export as → PDF
- 适合打印和文档附件
- 保持高清晰度
```

### 3. 编辑和自定义

**编辑文本**:
- 双击任何模块直接编辑文本
- 支持HTML格式（如 `<b>加粗</b>`）

**调整布局**:
- 拖动模块调整位置
- 对齐工具：Arrange → Align
- 分布工具：Arrange → Distribute

**修改样式**:
- 右键模块 → Edit Style
- 或使用右侧样式面板
- 修改颜色、边框、字体等

**调整连线**:
- 点击连线选中
- 拖动中间点调整路径
- 右键 → Edit Style 修改样式

## 技术亮点标注

### 1. 双模式通信
架构图清晰展示了两种通信方式：
- **串口模式**: 直接USB/串口通信（紫色箭头）
- **ROS2模式**: 通过ROS2桥接和节点（紫色箭头）

### 2. 安全机制
多重安全检查在图中明确标注：
- 关节限制检查
- 最大相对目标限制
- 位置裁剪
- 速度限制

### 3. 高频控制
控制频率参数清晰标注：
- 机器人控制: 100Hz
- Robosuite: 20Hz

### 4. 仿真支持
Robosuite 环境作为独立模块展示：
- 使用 UR5e 作为 PAROL6 代理
- 完整的测试流程

### 5. 完整工作流
从遥操作到训练推理的完整流程可视化。

## 关键信息速查

### 图例说明
图底部包含完整图例：
- **应用层** (蓝色) - LeRobot工作流
- **机器人接口层** (橙色) - PAROL6类
- **通信层** (紫色) - 串口/ROS2
- **硬件/仿真层** (绿色) - 硬件和仿真

### 主要特性
- 6自由度关节控制 (类似 UR5)
- 双模式通信: 串口和ROS2
- 安全功能: 关节限制、位置过滤
- 控制频率: 100Hz
- 相机集成支持

### 重要文件
- `lerobot/common/robots/parol6/parol6.py`
- `ros2_adapter/parol6_adapter/parol6_ros2_bridge.py`
- `ros2_adapter/parol6_adapter/parol6_ros2_node.py`
- `tests/parol6/test_parol6_robosuite.py`

### 关键配置
- 端口: /dev/ttyUSB0
- 波特率: 115200
- ROS2命名空间: /parol6
- 最大相对目标: 0.5

### 测试命令
```bash
python tests/parol6/test_parol6_robosuite.py --test all
python tests/parol6/test_ros2_integration.py --test all
python examples/parol6_example.py --example basic
```

## 架构设计理念

### 1. 分层架构
- 清晰的4层架构
- 每层职责明确
- 便于维护和扩展

### 2. 模块化设计
- 每个模块独立
- 接口清晰
- 易于测试

### 3. 灵活性
- 支持多种通信方式
- 可选ROS2集成
- 支持仿真和真实环境

### 4. 安全性
- 多重安全检查
- 位置限制
- 平滑控制

## 参考文档

- [PAROL6_INTEGRATION.md](PAROL6_INTEGRATION.md) - 完整集成指南
- [PAROL6_PROJECT_SUMMARY.md](PAROL6_PROJECT_SUMMARY.md) - 项目总结
- [lerobot/common/robots/parol6/README.md](lerobot/common/robots/parol6/README.md) - PAROL6 使用文档
- [ros2_adapter/README.md](ros2_adapter/README.md) - ROS2 适配器文档

## 常见问题

### Q: 为什么使用 Material Design 配色？
A: Material Design 是 Google 推出的设计系统，色彩科学、对比度高、专业感强，适合技术文档。

### Q: 如何修改配色？
A: 在 Draw.io 中选中模块，右键 → Edit Style → fillColor/strokeColor，修改颜色代码。

### Q: 连线为什么这样走？
A: 使用正交圆角样式，避免穿过其他模块，从空白区域走线，提高可读性。

### Q: 可以添加更多模块吗？
A: 可以！拖动已有模块调整间距，然后添加新模块。保持配色和样式一致即可。

### Q: 如何导出高清图片？
A: File → Export as → PNG，选择 300 DPI 或更高，勾选 "Include a copy of my diagram"。

## 架构图统计

- **总层级**: 4 层 + 1 图例层
- **总模块**: 18 个功能模块
- **连线数**: 15+ 条数据流箭头
- **标注文件**: 12+ 个关键文件
- **标注函数**: 20+ 个核心函数
- **配置参数**: 15+ 个关键参数
- **画布尺寸**: 1600x1200px
- **推荐导出**: PNG/SVG/PDF (300 DPI)

---

**版本**: 2.0 (完整中文版)
**最后更新**: 2025-11-25
**作者**: Claude Code
