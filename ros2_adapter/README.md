# PAROL6 ROS2 Adapter

ROS2 adapter for PAROL6 robot integration with LeRobot.

This package provides a ROS2 bridge that allows PAROL6 to communicate with the ROS2 ecosystem while maintaining compatibility with LeRobot's interface.

## Features

- **ROS2 Native**: Full ROS2 support with standard interfaces
- **LeRobot Compatible**: Seamless integration with LeRobot framework
- **Standard Topics**: Compatible with ROS2 control stack
- **Real-time Control**: 100Hz+ control loop
- **Thread-safe**: Safe concurrent access from multiple threads

## Architecture

```
┌─────────────────┐
│   LeRobot       │
│   Application   │
└────────┬────────┘
         │
         v
┌─────────────────┐
│   Parol6        │
│   Robot Class   │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  ROS2 Bridge    │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  ROS2 Node      │
└────────┬────────┘
         │
         v
┌─────────────────┐
│  PAROL6 Robot   │
│  Hardware       │
└─────────────────┘
```

## Installation

### Prerequisites

- ROS2 (Humble or later recommended)
- Python 3.8+
- LeRobot

### Install ROS2

```bash
# Ubuntu 22.04 (Humble)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-sensor-msgs
```

### Install ROS2 Adapter

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Install adapter
cd ros2_adapter
pip install -e .
```

## Usage

### 1. Standalone ROS2 Node

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run node directly
ros2 run parol6_ros2_adapter parol6_ros2_node

# Or use launch file
ros2 launch ros2_adapter/launch/parol6_bringup.launch.py
```

### 2. With Launch Script

```bash
# Start in simulation mode
./scripts/parol6/start_parol6_ros2.sh sim /parol6

# Start with real robot
./scripts/parol6/start_parol6_ros2.sh real /parol6
```

### 3. Programmatic Usage

```python
from ros2_adapter.parol6_adapter import Parol6ROS2Bridge
import numpy as np

# Create bridge
bridge = Parol6ROS2Bridge(
    namespace="/parol6",
    control_frequency=100
)

# Connect
bridge.connect()

# Send commands
positions = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
bridge.send_joint_positions(positions)

# Read state
current_pos = bridge.get_joint_positions()
current_vel = bridge.get_joint_velocities()

# Disconnect
bridge.disconnect()
```

## ROS2 Interface

### Topics

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/parol6/joint_states` | `sensor_msgs/JointState` | Current joint positions, velocities, efforts |
| `/parol6/joint_trajectory_controller/state` | `control_msgs/JointTrajectoryControllerState` | Controller state |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/parol6/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Joint trajectory commands |
| `/parol6/joint_commands` | `sensor_msgs/JointState` | Direct position commands |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim` | bool | `false` | Use simulation mode |
| `publish_frequency` | int | `100` | Joint state publish frequency (Hz) |
| `namespace` | string | `/parol6` | ROS2 namespace |

## Examples

### Monitor Joint States

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Echo joint states
ros2 topic echo /parol6/joint_states
```

### Send Position Commands

```bash
# Publish position command
ros2 topic pub /parol6/joint_commands sensor_msgs/msg/JointState \
    "{name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
    position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}"
```

### Use with MoveIt2

```python
# Coming soon: MoveIt2 integration example
```

## Integration with LeRobot

The ROS2 adapter integrates seamlessly with LeRobot:

```python
from lerobot.common.robots.parol6 import Parol6, Parol6Config

# Enable ROS2 in config
config = Parol6Config(
    id="my_parol6",
    use_ros2_bridge=True,
    ros2_namespace="/parol6",
)

# Use normally - ROS2 bridge is handled automatically
robot = Parol6(config)
robot.connect()

# All LeRobot operations work as expected
obs = robot.get_observation()
robot.send_action(action)

robot.disconnect()
```

## Testing

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run tests
python tests/parol6/test_ros2_integration.py --test all
```

## Visualization with RViz

```bash
# Launch RViz
ros2 run rviz2 rviz2

# In RViz:
# 1. Add -> RobotModel (requires URDF)
# 2. Add -> TF
# 3. Add -> JointState
```

## Troubleshooting

### ROS2 Not Found

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for permanent effect
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Topics Not Publishing

```bash
# Check if node is running
ros2 node list

# Check if topics exist
ros2 topic list

# Check topic info
ros2 topic info /parol6/joint_states
```

### Import Errors

```bash
# Make sure adapter is installed
pip install -e .

# Check Python path
python -c "import ros2_adapter; print(ros2_adapter.__file__)"
```

## Development

### Building

```bash
# Install in development mode
pip install -e .

# Run tests
pytest tests/
```

### Code Style

```bash
# Format code
black ros2_adapter/

# Lint
flake8 ros2_adapter/
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

Apache-2.0

## Support

For issues and questions:
- GitHub Issues: [lerobot-joycon2/issues](https://github.com/box2ai-robotics/lerobot-joycon2/issues)
- ROS2 Documentation: [docs.ros.org](https://docs.ros.org/)
