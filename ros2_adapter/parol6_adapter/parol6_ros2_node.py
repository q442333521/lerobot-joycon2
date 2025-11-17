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
ROS2 Node for PAROL6 Robot Integration

This node provides ROS2 interface for PAROL6 robot, compatible with
standard ROS2 control interfaces.
"""

import numpy as np
from typing import Callable, Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.msg import JointTrajectoryControllerState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available. Install ROS2 to use this module.")


class Parol6ROS2Node(Node):
    """
    ROS2 Node for PAROL6 robot.

    Provides:
    - Joint state publisher
    - Joint command subscriber
    - Trajectory controller interface
    """

    def __init__(self, namespace: str = "/parol6", callback: Optional[Callable] = None):
        """
        Initialize ROS2 node.

        Args:
            namespace: ROS2 namespace
            callback: Callback function for joint state updates
        """
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 is not available")

        super().__init__('parol6_ros2_node')

        self.namespace = namespace
        self.callback = callback

        # Joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'{namespace}/joint_states',
            10
        )

        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            f'{namespace}/joint_trajectory_controller/state',
            10
        )

        # Subscribers
        self.joint_command_sub = self.create_subscription(
            JointTrajectory,
            f'{namespace}/joint_trajectory_controller/joint_trajectory',
            self.joint_command_callback,
            10
        )

        # Alternative: direct position command
        self.position_command_sub = self.create_subscription(
            JointState,
            f'{namespace}/joint_commands',
            self.position_command_callback,
            10
        )

        # State
        self.current_positions = np.zeros(6)
        self.current_velocities = np.zeros(6)
        self.current_efforts = np.zeros(6)

        # Create timer for publishing joint states
        self.timer = self.create_timer(0.01, self.publish_joint_state)  # 100 Hz

        self.get_logger().info(f"Parol6ROS2Node initialized with namespace: {namespace}")

    def joint_command_callback(self, msg: JointTrajectory):
        """
        Handle joint trajectory commands.

        Args:
            msg: JointTrajectory message
        """
        if len(msg.points) == 0:
            self.get_logger().warning("Received empty trajectory")
            return

        # Use the last point in trajectory
        point = msg.points[-1]

        if len(point.positions) != 6:
            self.get_logger().warning(
                f"Expected 6 joint positions, got {len(point.positions)}"
            )
            return

        positions = np.array(point.positions)

        # Update state
        self.current_positions = positions

        # Call callback if provided
        if self.callback is not None:
            velocities = (
                np.array(point.velocities)
                if len(point.velocities) == 6
                else self.current_velocities
            )
            self.callback(positions, velocities)

    def position_command_callback(self, msg: JointState):
        """
        Handle direct position commands.

        Args:
            msg: JointState message with position commands
        """
        if len(msg.position) != 6:
            self.get_logger().warning(
                f"Expected 6 joint positions, got {len(msg.position)}"
            )
            return

        positions = np.array(msg.position)

        # Update state
        self.current_positions = positions

        # Call callback if provided
        if self.callback is not None:
            velocities = (
                np.array(msg.velocity)
                if len(msg.velocity) == 6
                else self.current_velocities
            )
            self.callback(positions, velocities)

    def publish_joint_state(self):
        """Publish current joint state."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions.tolist()
        msg.velocity = self.current_velocities.tolist()
        msg.effort = self.current_efforts.tolist()

        self.joint_state_pub.publish(msg)

        # Also publish controller state
        self.publish_controller_state()

    def publish_controller_state(self):
        """Publish joint trajectory controller state."""
        msg = JointTrajectoryControllerState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        # Create trajectory point for current state
        point = JointTrajectoryPoint()
        point.positions = self.current_positions.tolist()
        point.velocities = self.current_velocities.tolist()

        msg.actual = point
        msg.desired = point  # Assuming we're at desired position
        msg.error = point  # Zero error for now

        self.controller_state_pub.publish(msg)

    def publish_joint_command(self, positions: np.ndarray):
        """
        Publish joint command (for internal use).

        This updates the internal state which will be published
        by the timer callback.

        Args:
            positions: Target joint positions
        """
        self.current_positions = positions.copy()

    def update_joint_state(self, positions: np.ndarray, velocities: np.ndarray):
        """
        Update internal joint state (called from robot feedback).

        Args:
            positions: Current joint positions
            velocities: Current joint velocities
        """
        self.current_positions = positions
        self.current_velocities = velocities


def main(args=None):
    """Main function for standalone ROS2 node."""
    if not ROS2_AVAILABLE:
        print("ROS2 is not available. Cannot run node.")
        return

    rclpy.init(args=args)

    node = Parol6ROS2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
