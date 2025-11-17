#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.common.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("parol6")
@dataclass
class Parol6Config(RobotConfig):
    """
    Configuration for PAROL6 6-DOF robotic arm.

    PAROL6 is similar to UR5 in terms of workspace and capabilities.
    It features 6 degrees of freedom with the following joints:
    - shoulder_pan: Base rotation
    - shoulder_lift: Shoulder lift
    - elbow: Elbow joint
    - wrist_1: First wrist rotation
    - wrist_2: Second wrist rotation
    - wrist_3: Third wrist rotation (end effector)
    """

    # Port to connect to the arm (serial or network)
    port: str = "/dev/ttyUSB0"

    # Baudrate for serial communication
    baudrate: int = 115200

    # Network IP for TCP/IP connection (if applicable)
    ip_address: str | None = None

    # Network port for TCP/IP connection
    network_port: int = 30002

    # Use ROS2 bridge for communication
    use_ros2_bridge: bool = False

    # ROS2 namespace
    ros2_namespace: str = "/parol6"

    # Disable torque on disconnect for safety
    disable_torque_on_disconnect: bool = True

    # Maximum relative target for safety
    # Limits the magnitude of the relative positional target vector
    max_relative_target: float | None = 0.5

    # Maximum joint velocities (rad/s) for each joint
    max_joint_velocities: list[float] = field(default_factory=lambda: [3.15, 3.15, 3.15, 3.2, 3.2, 3.2])

    # Maximum joint accelerations (rad/s^2)
    max_joint_accelerations: list[float] = field(default_factory=lambda: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0])

    # Joint limits (min, max) in radians
    # Based on typical PAROL6 specifications
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=lambda: {
        "shoulder_pan": (-3.14, 3.14),      # ±180°
        "shoulder_lift": (-1.57, 1.57),     # ±90°
        "elbow": (-2.35, 2.35),             # ±135°
        "wrist_1": (-3.14, 3.14),           # ±180°
        "wrist_2": (-3.14, 3.14),           # ±180°
        "wrist_3": (-3.14, 3.14),           # ±180°
    })

    # Cameras configuration
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Control frequency in Hz
    control_frequency: int = 100

    # Use degrees instead of radians (for compatibility)
    use_degrees: bool = False

    # Home position (in radians or degrees based on use_degrees)
    home_position: list[float] = field(default_factory=lambda: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

    # Enable position smoothing/filtering
    enable_position_filter: bool = True

    # Position filter coefficient (0-1, higher = more smoothing)
    position_filter_coeff: float = 0.8
