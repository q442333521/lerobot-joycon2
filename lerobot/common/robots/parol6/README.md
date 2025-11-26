# PAROL6 Robot Integration for LeRobot

PAROL6 is a 6-DOF robotic arm similar to UR5 in workspace and capabilities. This integration provides full support for PAROL6 in the LeRobot framework, including ROS2 bridge for seamless integration with the ROS2 ecosystem.

## Features

- **6-DOF Control**: Full control of all 6 joints (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- **ROS2 Integration**: Optional ROS2 bridge for integration with ROS2 control stack
- **Camera Support**: Multi-camera support for vision-based learning
- **Safety Features**:
  - Joint limits enforcement
  - Maximum relative target limiting
  - Position filtering for smooth motion
- **Dual Communication**: Support for both direct serial and ROS2 communication

## Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Degrees of Freedom | 6 |
| Workspace | Similar to UR5 |
| Joint Limits | See config file |
| Max Velocity | 3.15 rad/s (typical) |
| Max Acceleration | 10 rad/s² |
| Communication | Serial (USB) or ROS2 |

## Quick Start

### 1. Installation

```bash
# Install LeRobot (if not already installed)
pip install -e .

# For ROS2 support, install ROS2 adapter
cd ros2_adapter
pip install -e .
```

### 2. Basic Usage (Without ROS2)

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

# Create configuration
config = Parol6Config(
    id="my_parol6",
    port="/dev/ttyUSB0",
    baudrate=115200,
)

# Connect to robot
robot = Parol6(config)
robot.connect()

# Get observation
obs = robot.get_observation()
print(f"Current position: {obs}")

# Send action
action = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": -1.57,
    "elbow.pos": 1.57,
    "wrist_1.pos": -1.57,
    "wrist_2.pos": -1.57,
    "wrist_3.pos": 0.0,
}
robot.send_action(action)

# Disconnect
robot.disconnect()
```

### 3. Usage with ROS2

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

# Create configuration with ROS2 enabled
config = Parol6Config(
    id="my_parol6",
    use_ros2_bridge=True,
    ros2_namespace="/parol6",
    control_frequency=100,
)

# Connect (will start ROS2 bridge automatically)
robot = Parol6(config)
robot.connect()

# Use robot as normal
obs = robot.get_observation()
robot.send_action(action)

robot.disconnect()
```

### 4. Start ROS2 Node Separately

```bash
# Start ROS2 bridge node
./scripts/parol6/start_parol6_ros2.sh sim /parol6

# In another terminal, run your LeRobot script
python your_script.py
```

## Configuration

The `Parol6Config` class provides extensive configuration options:

```python
@dataclass
class Parol6Config(RobotConfig):
    # Communication
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200

    # ROS2 Settings
    use_ros2_bridge: bool = False
    ros2_namespace: str = "/parol6"

    # Safety
    max_relative_target: float = 0.5  # Max movement per step
    disable_torque_on_disconnect: bool = True

    # Control
    control_frequency: int = 100  # Hz
    enable_position_filter: bool = True
    position_filter_coeff: float = 0.8

    # Joint limits (rad)
    joint_limits: dict = ...

    # Cameras
    cameras: dict[str, CameraConfig] = {}
```

## ROS2 Topics

When using ROS2 bridge, the following topics are available:

### Published Topics

- `/parol6/joint_states` (sensor_msgs/JointState): Current joint positions and velocities
- `/parol6/joint_trajectory_controller/state` (control_msgs/JointTrajectoryControllerState): Controller state

### Subscribed Topics

- `/parol6/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory): Joint trajectory commands
- `/parol6/joint_commands` (sensor_msgs/JointState): Direct position commands

## Testing

### Test LeRobot Integration

```bash
# Test basic connection
python scripts/parol6/test_parol6_lerobot.py --test connection

# Test joint control
python scripts/parol6/test_parol6_lerobot.py --test control

# Test with ROS2
python scripts/parol6/test_parol6_lerobot.py --use-ros2 --test all
```

### Test Robosuite Simulation

```bash
# Install robosuite first
pip install robosuite

# Run simulation tests
python tests/parol6/test_parol6_robosuite.py --test all
```

### Test ROS2 Integration

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Run ROS2 tests
python tests/parol6/test_ros2_integration.py --test all
```

## Examples

### Teleoperation with Joycon

```bash
python -m lerobot.teleoperate \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --teleop.type=joycon
```

### Data Recording

```bash
python -m lerobot.record \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --robot.cameras="{ \
        OBS_IMAGE_1: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30} \
    }" \
    --dataset.single_task="Pick and place" \
    --dataset.repo_id=Datasets/parol6_pick_place \
    --dataset.num_episodes=50 \
    --teleop.type=joycon
```

### Training with SmolVLA

```bash
python lerobot/scripts/train.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.repo_id=Datasets/parol6_pick_place \
    --batch_size=8 \
    --steps=20000 \
    --output_dir=outputs/train/parol6_smolvla \
    --policy.device=cuda
```

### Inference

```bash
python -m lerobot.record \
    --robot.type=parol6 \
    --robot.port=/dev/ttyUSB0 \
    --robot.id=my_parol6 \
    --policy.path=outputs/train/parol6_smolvla/checkpoints/last/pretrained_model
```

## Robosuite Integration

PAROL6 is similar to UR5, so you can use Robosuite's UR5e model for simulation:

```python
import robosuite as suite

# Create environment
env = suite.make(
    env_name="Lift",
    robots="UR5e",  # UR5e is similar to PAROL6
    has_renderer=True,
    control_freq=20,
)

# Use with PAROL6 action space
obs = env.reset()
for i in range(1000):
    # PAROL6-style action (6 joints + gripper)
    action = parol6_controller.get_action(obs)
    obs, reward, done, info = env.step(action)
    env.render()
```

## Troubleshooting

### Serial Connection Issues

```bash
# Check if device is connected
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set permissions (temporary)
sudo chmod 666 /dev/ttyUSB0
```

### ROS2 Issues

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Check ROS2 nodes
ros2 node list

# Check topics
ros2 topic list

# Echo joint states
ros2 topic echo /parol6/joint_states
```

## Contributing

Contributions are welcome! Please see the main LeRobot contributing guidelines.

## License

Apache-2.0

## References

- [LeRobot](https://github.com/huggingface/lerobot)
- [PAROL6 Robot](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm)
- [Robosuite](https://robosuite.ai/)
- [ROS2](https://docs.ros.org/)
