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
Simple example of using PAROL6 with LeRobot.

This example demonstrates:
1. Connecting to PAROL6
2. Reading observations
3. Sending actions
4. Smooth motion control
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add lerobot to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from lerobot.common.robots.parol6 import Parol6, Parol6Config


def example_basic_control():
    """Basic control example."""
    print("\n" + "=" * 60)
    print("PAROL6 Basic Control Example")
    print("=" * 60)

    # Create configuration
    config = Parol6Config(
        id="parol6_example",
        port="/dev/ttyUSB0",  # Change to your port
        use_ros2_bridge=False,  # Set to True to use ROS2
        max_relative_target=0.2,  # Limit movement for safety
        enable_position_filter=True,
    )

    # Connect to robot
    print("\nConnecting to PAROL6...")
    robot = Parol6(config)
    robot.connect()
    print("✓ Connected!")

    try:
        # Read initial state
        print("\nReading initial state...")
        obs = robot.get_observation()
        print("Initial joint positions:")
        for joint in robot.joint_names:
            pos = obs[f"{joint}.pos"]
            print(f"  {joint}: {pos:.4f} rad ({np.degrees(pos):.2f}°)")

        # Move to a specific position
        print("\nMoving to target position...")
        target_action = {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": -1.0,
            "elbow.pos": 1.5,
            "wrist_1.pos": -0.5,
            "wrist_2.pos": 0.0,
            "wrist_3.pos": 0.0,
        }

        # Send action
        robot.send_action(target_action)
        print("✓ Command sent!")

        # Wait for movement
        print("Waiting for movement to complete...")
        time.sleep(3.0)

        # Read new state
        obs = robot.get_observation()
        print("\nNew joint positions:")
        for joint in robot.joint_names:
            pos = obs[f"{joint}.pos"]
            target = target_action[f"{joint}.pos"]
            print(f"  {joint}: {pos:.4f} rad (target: {target:.4f})")

        # Return to home
        print("\nReturning to home position...")
        home_action = {
            f"{joint}.pos": pos
            for joint, pos in zip(robot.joint_names, config.home_position)
        }
        robot.send_action(home_action)
        time.sleep(3.0)

        print("✓ Example completed successfully!")

    finally:
        # Always disconnect
        robot.disconnect()
        print("\n✓ Disconnected")


def example_smooth_motion():
    """Smooth sinusoidal motion example."""
    print("\n" + "=" * 60)
    print("PAROL6 Smooth Motion Example")
    print("=" * 60)
    print("Press Ctrl+C to stop")

    # Create configuration
    config = Parol6Config(
        id="parol6_example",
        port="/dev/ttyUSB0",
        use_ros2_bridge=False,
        enable_position_filter=True,
        position_filter_coeff=0.9,  # High smoothing
    )

    # Connect
    print("\nConnecting to PAROL6...")
    robot = Parol6(config)
    robot.connect()
    print("✓ Connected!")

    try:
        print("\nExecuting smooth motion (10 seconds)...")
        start_time = time.time()
        dt = 0.01  # 100 Hz

        while time.time() - start_time < 10.0:
            # Calculate sinusoidal motion
            t = time.time() - start_time
            omega = 0.5  # Angular frequency

            # Create smooth motion
            pos = list(config.home_position)
            pos[0] += 0.3 * np.sin(omega * t)  # shoulder_pan
            pos[1] += 0.2 * np.cos(omega * t)  # shoulder_lift
            pos[2] += 0.1 * np.sin(2 * omega * t)  # elbow

            # Send action
            action = {
                f"{joint}.pos": p
                for joint, p in zip(robot.joint_names, pos)
            }
            robot.send_action(action)

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\nMotion stopped by user")

    finally:
        # Return to home
        print("\nReturning to home position...")
        home_action = {
            f"{joint}.pos": pos
            for joint, pos in zip(robot.joint_names, config.home_position)
        }
        robot.send_action(home_action)
        time.sleep(2.0)

        # Disconnect
        robot.disconnect()
        print("✓ Disconnected")


def example_with_cameras():
    """Example with camera integration."""
    print("\n" + "=" * 60)
    print("PAROL6 with Cameras Example")
    print("=" * 60)

    from lerobot.common.cameras import CameraConfig

    # Create configuration with cameras
    config = Parol6Config(
        id="parol6_example",
        port="/dev/ttyUSB0",
        cameras={
            "camera_front": CameraConfig(
                type="opencv",
                index_or_path=0,
                width=640,
                height=480,
                fps=30,
            ),
        },
    )

    # Connect
    print("\nConnecting to PAROL6 with cameras...")
    robot = Parol6(config)
    robot.connect()
    print("✓ Connected!")

    try:
        # Get observation with images
        print("\nCapturing observation with images...")
        obs = robot.get_observation()

        # Print joint positions
        print("\nJoint positions:")
        for joint in robot.joint_names:
            print(f"  {joint}: {obs[f'{joint}.pos']:.4f} rad")

        # Check camera images
        if "camera_front" in obs:
            image = obs["camera_front"]
            print(f"\nCamera image shape: {image.shape}")
            print("✓ Image captured successfully!")

    finally:
        robot.disconnect()
        print("\n✓ Disconnected")


def main():
    """Run examples."""
    import argparse

    parser = argparse.ArgumentParser(description="PAROL6 Examples")
    parser.add_argument(
        "--example",
        type=str,
        choices=["basic", "smooth", "cameras"],
        default="basic",
        help="Which example to run"
    )

    args = parser.parse_args()

    if args.example == "basic":
        example_basic_control()
    elif args.example == "smooth":
        example_smooth_motion()
    elif args.example == "cameras":
        example_with_cameras()


if __name__ == "__main__":
    main()
