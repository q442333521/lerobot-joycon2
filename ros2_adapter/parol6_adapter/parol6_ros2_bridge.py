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

"""
ROS2 Bridge for PAROL6 Robot

This module provides a bridge between LeRobot's Parol6 class and ROS2.
It handles communication with ROS2 topics and services.
"""

import logging
import numpy as np
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)


class Parol6ROS2Bridge:
    """
    Bridge between LeRobot and ROS2 for PAROL6 robot.

    This class manages:
    - Publishing joint commands to ROS2
    - Subscribing to joint states from ROS2
    - Service calls for robot control
    """

    def __init__(self, namespace: str = "/parol6", control_frequency: int = 100):
        """
        Initialize ROS2 bridge.

        Args:
            namespace: ROS2 namespace for topics
            control_frequency: Control loop frequency in Hz
        """
        self.namespace = namespace
        self.control_frequency = control_frequency
        self.dt = 1.0 / control_frequency

        # State variables
        self._current_positions = np.zeros(6)
        self._current_velocities = np.zeros(6)
        self._target_positions = np.zeros(6)
        self._is_connected = False

        # Thread safety
        self._lock = threading.Lock()

        # ROS2 node (will be initialized in connect())
        self.node = None
        self._ros2_thread = None
        self._running = False

        logger.info(f"Parol6ROS2Bridge initialized with namespace: {namespace}")

    def connect(self):
        """Initialize ROS2 node and start communication."""
        try:
            import rclpy
            from .parol6_ros2_node import Parol6ROS2Node

            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()

            # Create node
            self.node = Parol6ROS2Node(
                namespace=self.namespace,
                callback=self._joint_state_callback
            )

            # Start ROS2 spinning in separate thread
            self._running = True
            self._ros2_thread = threading.Thread(target=self._spin_ros2, daemon=True)
            self._ros2_thread.start()

            self._is_connected = True
            logger.info("ROS2 bridge connected")

        except ImportError as e:
            logger.error(f"Failed to import ROS2 dependencies: {e}")
            logger.error("Make sure ROS2 is installed and sourced")
            raise
        except Exception as e:
            logger.error(f"Failed to connect ROS2 bridge: {e}")
            raise

    def _spin_ros2(self):
        """Spin ROS2 node in separate thread."""
        import rclpy

        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _joint_state_callback(self, positions: np.ndarray, velocities: np.ndarray):
        """
        Callback for joint state updates from ROS2.

        Args:
            positions: Joint positions
            velocities: Joint velocities
        """
        with self._lock:
            self._current_positions = positions
            self._current_velocities = velocities

    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        with self._lock:
            return self._current_positions.copy()

    def get_joint_velocities(self) -> np.ndarray:
        """Get current joint velocities."""
        with self._lock:
            return self._current_velocities.copy()

    def send_joint_positions(self, positions: np.ndarray):
        """
        Send joint position commands to ROS2.

        Args:
            positions: Target joint positions (6 values)
        """
        if not self._is_connected or self.node is None:
            logger.warning("ROS2 bridge not connected, cannot send positions")
            return

        with self._lock:
            self._target_positions = positions.copy()

        # Publish to ROS2
        self.node.publish_joint_command(positions)

    def disconnect(self):
        """Disconnect from ROS2."""
        self._running = False

        if self._ros2_thread is not None:
            self._ros2_thread.join(timeout=2.0)

        if self.node is not None:
            self.node.destroy_node()

        self._is_connected = False
        logger.info("ROS2 bridge disconnected")

    @property
    def is_connected(self) -> bool:
        """Check if bridge is connected."""
        return self._is_connected
