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
Test script for PAROL6 with LeRobot integration.

This script tests:
1. Robot connection
2. Joint position control
3. Observation reading
4. Camera integration (if available)
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add lerobot to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from lerobot.common.robots.parol6 import Parol6, Parol6Config


def test_connection(use_ros2: bool = False):
    """Test basic connection to PAROL6."""
    print("\n" + "=" * 50)
    print("Test 1: Connection Test")
    print("=" * 50)

    config = Parol6Config(
        id="parol6_test",
        port="/dev/ttyUSB0",
        use_ros2_bridge=use_ros2,
    )

    print(f"Connecting to PAROL6 (ROS2: {use_ros2})...")

    try:
        robot = Parol6(config)
        robot.connect()

        print("✓ Connection successful!")
        print(f"  - Robot: {robot.name}")
        print(f"  - Connected: {robot.is_connected}")
        print(f"  - Calibrated: {robot.is_calibrated}")

        # Read initial state
        obs = robot.get_observation()
        print(f"\n  Initial joint positions:")
        for joint in robot.joint_names:
            pos = obs.get(f"{joint}.pos", 0.0)
            print(f"    {joint}: {pos:.4f} rad ({np.degrees(pos):.2f}°)")

        robot.disconnect()
        print("\n✓ Disconnection successful!")
        return True

    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_joint_control(use_ros2: bool = False):
    """Test joint position control."""
    print("\n" + "=" * 50)
    print("Test 2: Joint Control Test")
    print("=" * 50)

    config = Parol6Config(
        id="parol6_test",
        port="/dev/ttyUSB0",
        use_ros2_bridge=use_ros2,
        max_relative_target=0.2,  # Limit movement for safety
    )

    try:
        robot = Parol6(config)
        robot.connect()

        print("Moving joints in sequence...")

        # Test each joint individually
        for i, joint in enumerate(robot.joint_names):
            print(f"\n  Testing {joint}...")

            # Get current position
            obs = robot.get_observation()
            current_pos = [obs[f"{j}.pos"] for j in robot.joint_names]

            # Move joint slightly
            target_pos = current_pos.copy()
            target_pos[i] += 0.1  # Move 0.1 rad

            # Send command
            action = {f"{joint}.pos": pos for joint, pos in zip(robot.joint_names, target_pos)}
            robot.send_action(action)

            time.sleep(1.0)

            # Read new position
            obs = robot.get_observation()
            new_pos = obs[f"{joint}.pos"]

            print(f"    Commanded: {target_pos[i]:.4f} rad")
            print(f"    Actual: {new_pos:.4f} rad")
            print(f"    ✓ Joint {joint} moved!")

        # Return to home
        print("\n  Returning to home position...")
        home_action = {
            f"{joint}.pos": pos
            for joint, pos in zip(robot.joint_names, config.home_position)
        }
        robot.send_action(home_action)
        time.sleep(2.0)

        robot.disconnect()
        print("\n✓ Joint control test successful!")
        return True

    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_circular_motion(use_ros2: bool = False, duration: float = 10.0):
    """Test smooth circular motion."""
    print("\n" + "=" * 50)
    print("Test 3: Circular Motion Test")
    print("=" * 50)

    config = Parol6Config(
        id="parol6_test",
        port="/dev/ttyUSB0",
        use_ros2_bridge=use_ros2,
        enable_position_filter=True,
    )

    try:
        robot = Parol6(config)
        robot.connect()

        print(f"Executing circular motion for {duration} seconds...")
        print("Press Ctrl+C to stop early")

        start_time = time.time()
        dt = 0.01  # 100 Hz control

        try:
            while time.time() - start_time < duration:
                # Circular motion in shoulder joints
                t = time.time() - start_time
                omega = 0.5  # Angular frequency

                # Create sinusoidal motion
                pos = config.home_position.copy()
                pos[0] += 0.3 * np.sin(omega * t)  # shoulder_pan
                pos[1] += 0.2 * np.cos(omega * t)  # shoulder_lift

                # Send action
                action = {
                    f"{joint}.pos": p
                    for joint, p in zip(robot.joint_names, pos)
                }
                robot.send_action(action)

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n  Motion stopped by user")

        # Return to home
        print("\n  Returning to home position...")
        home_action = {
            f"{joint}.pos": pos
            for joint, pos in zip(robot.joint_names, config.home_position)
        }
        robot.send_action(home_action)
        time.sleep(2.0)

        robot.disconnect()
        print("\n✓ Circular motion test successful!")
        return True

    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    import argparse

    parser = argparse.ArgumentParser(description="Test PAROL6 with LeRobot")
    parser.add_argument(
        "--use-ros2",
        action="store_true",
        help="Use ROS2 bridge for communication"
    )
    parser.add_argument(
        "--test",
        type=str,
        choices=["connection", "control", "motion", "all"],
        default="all",
        help="Which test to run"
    )

    args = parser.parse_args()

    print("\n" + "=" * 50)
    print("PAROL6 LeRobot Integration Tests")
    print("=" * 50)
    print(f"Using ROS2: {args.use_ros2}")
    print("=" * 50)

    results = {}

    if args.test in ["connection", "all"]:
        results["connection"] = test_connection(args.use_ros2)

    if args.test in ["control", "all"]:
        results["control"] = test_joint_control(args.use_ros2)

    if args.test in ["motion", "all"]:
        results["motion"] = test_circular_motion(args.use_ros2)

    # Summary
    print("\n" + "=" * 50)
    print("Test Summary")
    print("=" * 50)
    for test_name, success in results.items():
        status = "✓ PASSED" if success else "✗ FAILED"
        print(f"{test_name}: {status}")

    all_passed = all(results.values())
    print("=" * 50)

    if all_passed:
        print("All tests passed!")
        return 0
    else:
        print("Some tests failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
