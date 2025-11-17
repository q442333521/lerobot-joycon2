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

import logging
import time
import numpy as np
from functools import cached_property
from typing import Any

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_parol6 import Parol6Config

logger = logging.getLogger(__name__)


class Parol6(Robot):
    """
    PAROL6 6-DOF Robotic Arm.

    Similar to UR5 in workspace and capabilities. Supports both direct serial
    communication and ROS2 bridge for integration with ROS2 ecosystem.

    Features:
    - 6 degrees of freedom
    - Position and velocity control
    - Camera integration
    - ROS2 bridge support
    - Safety limits and position filtering
    """

    config_class = Parol6Config
    name = "parol6"

    def __init__(self, config: Parol6Config):
        super().__init__(config)
        self.config = config

        # Joint names
        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ]

        # Initialize state
        self._current_position = np.array(config.home_position)
        self._target_position = np.array(config.home_position)
        self._current_velocity = np.zeros(6)
        self._is_connected = False

        # Position filter for smoothing
        self._filtered_position = np.array(config.home_position)

        # Communication interface
        if config.use_ros2_bridge:
            self._init_ros2_bridge()
        else:
            self._init_serial_connection()

        # Initialize cameras
        self.cameras = make_cameras_from_configs(config.cameras)

    def _init_serial_connection(self):
        """Initialize serial connection to PAROL6."""
        import serial

        try:
            self.serial_port = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=1.0
            )
            logger.info(f"Serial connection initialized on {self.config.port}")
        except Exception as e:
            logger.warning(f"Failed to initialize serial connection: {e}")
            self.serial_port = None

    def _init_ros2_bridge(self):
        """Initialize ROS2 bridge for communication."""
        try:
            # Import ROS2 dependencies (will be implemented in ROS2 adapter)
            from ros2_adapter.parol6_adapter import Parol6ROS2Bridge

            self.ros2_bridge = Parol6ROS2Bridge(
                namespace=self.config.ros2_namespace,
                control_frequency=self.config.control_frequency
            )
            logger.info(f"ROS2 bridge initialized with namespace {self.config.ros2_namespace}")
        except Exception as e:
            logger.warning(f"Failed to initialize ROS2 bridge: {e}")
            self.ros2_bridge = None

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Feature types for motor positions."""
        return {f"{motor}.pos": float for motor in self.joint_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Feature types for cameras."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Combined observation features."""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features (motor positions)."""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        """Check if robot and cameras are connected."""
        cameras_connected = all(cam.is_connected for cam in self.cameras.values())

        if self.config.use_ros2_bridge:
            return self._is_connected and cameras_connected
        else:
            return (self.serial_port is not None and
                    self.serial_port.is_open and
                    cameras_connected)

    @property
    def is_calibrated(self) -> bool:
        """Check if robot is calibrated."""
        # PAROL6 uses absolute encoders, so calibration is optional
        return True

    def connect(self, calibrate: bool = False) -> None:
        """Connect to PAROL6 and cameras."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # Connect based on communication method
        if self.config.use_ros2_bridge:
            if self.ros2_bridge:
                self.ros2_bridge.connect()
                self._is_connected = True
        else:
            if self.serial_port is None:
                self._init_serial_connection()

            if self.serial_port and not self.serial_port.is_open:
                self.serial_port.open()

            self._is_connected = True

        # Connect cameras
        for cam in self.cameras.values():
            cam.connect()

        # Move to home position
        self._move_to_home()

        logger.info(f"{self} connected.")

    def _move_to_home(self):
        """Move robot to home position."""
        home_action = {
            f"{joint}.pos": pos
            for joint, pos in zip(self.joint_names, self.config.home_position)
        }
        self.send_action(home_action)
        time.sleep(2.0)  # Wait for movement to complete

    def calibrate(self) -> None:
        """
        Calibrate PAROL6 (optional, as it uses absolute encoders).
        This can be used to verify home position and joint limits.
        """
        logger.info(f"Calibrating {self}")

        # Read current position
        current_pos = self._read_joint_positions()

        logger.info(f"Current position: {current_pos}")
        logger.info(f"Home position: {self.config.home_position}")

        # Verify joint limits
        for i, joint in enumerate(self.joint_names):
            min_limit, max_limit = self.config.joint_limits[joint]
            if not (min_limit <= current_pos[i] <= max_limit):
                logger.warning(
                    f"Joint {joint} position {current_pos[i]:.3f} "
                    f"outside limits [{min_limit:.3f}, {max_limit:.3f}]"
                )

    def _read_joint_positions(self) -> np.ndarray:
        """Read current joint positions from robot."""
        if self.config.use_ros2_bridge and self.ros2_bridge:
            return self.ros2_bridge.get_joint_positions()
        else:
            # Read from serial port
            # This is a placeholder - actual implementation depends on PAROL6 protocol
            return self._current_position.copy()

    def _read_joint_velocities(self) -> np.ndarray:
        """Read current joint velocities from robot."""
        if self.config.use_ros2_bridge and self.ros2_bridge:
            return self.ros2_bridge.get_joint_velocities()
        else:
            # Read from serial port
            return self._current_velocity.copy()

    def get_observation(self) -> dict[str, Any]:
        """Get current observation including joint positions and camera images."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read joint positions
        start = time.perf_counter()
        positions = self._read_joint_positions()
        self._current_position = positions

        # Create observation dictionary
        obs_dict = {
            f"{joint}.pos": pos
            for joint, pos in zip(self.joint_names, positions)
        }

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read joint state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Send action to robot.

        Args:
            action: Dictionary with joint position targets

        Returns:
            The actual action sent (potentially clipped for safety)
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Extract goal positions
        goal_pos = {}
        for key, val in action.items():
            if key.endswith(".pos"):
                joint_name = key.removesuffix(".pos")
                goal_pos[joint_name] = val

        # Convert to numpy array
        goal_array = np.array([goal_pos[joint] for joint in self.joint_names])

        # Apply safety limits
        if self.config.max_relative_target is not None:
            current_pos = self._current_position
            diff = goal_array - current_pos
            diff_norm = np.linalg.norm(diff)

            if diff_norm > self.config.max_relative_target:
                # Scale down to max_relative_target
                goal_array = current_pos + diff * (self.config.max_relative_target / diff_norm)
                logger.debug(f"Clipped action from {diff_norm:.3f} to {self.config.max_relative_target:.3f}")

        # Apply joint limits
        for i, joint in enumerate(self.joint_names):
            min_limit, max_limit = self.config.joint_limits[joint]
            goal_array[i] = np.clip(goal_array[i], min_limit, max_limit)

        # Apply position filter if enabled
        if self.config.enable_position_filter:
            alpha = self.config.position_filter_coeff
            self._filtered_position = alpha * self._filtered_position + (1 - alpha) * goal_array
            goal_array = self._filtered_position.copy()

        # Send to robot
        self._target_position = goal_array
        self._send_joint_positions(goal_array)

        # Return actual action sent
        return {f"{joint}.pos": pos for joint, pos in zip(self.joint_names, goal_array)}

    def _send_joint_positions(self, positions: np.ndarray):
        """Send joint positions to robot."""
        if self.config.use_ros2_bridge and self.ros2_bridge:
            self.ros2_bridge.send_joint_positions(positions)
        else:
            # Send via serial port
            # This is a placeholder - actual implementation depends on PAROL6 protocol
            # Example format: "MOVE,{j0},{j1},{j2},{j3},{j4},{j5}\n"
            if self.serial_port and self.serial_port.is_open:
                cmd = f"MOVE,{','.join(f'{p:.4f}' for p in positions)}\n"
                self.serial_port.write(cmd.encode())

    def disconnect(self):
        """Disconnect from robot and cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        # Disconnect robot
        if self.config.use_ros2_bridge and self.ros2_bridge:
            self.ros2_bridge.disconnect()
        else:
            if self.serial_port and self.serial_port.is_open:
                # Send stop command
                if self.config.disable_torque_on_disconnect:
                    self.serial_port.write(b"STOP\n")
                self.serial_port.close()

        self._is_connected = False
        logger.info(f"{self} disconnected.")
