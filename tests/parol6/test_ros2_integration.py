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
Test ROS2 integration for PAROL6.

This script tests the ROS2 adapter functionality.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add paths
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


def test_ros2_node():
    """Test ROS2 node initialization."""
    print("\n" + "=" * 60)
    print("Test: ROS2 Node Initialization")
    print("=" * 60)

    try:
        import rclpy
        from ros2_adapter.parol6_adapter import Parol6ROS2Node

        print("Initializing ROS2...")
        rclpy.init()

        print("Creating PAROL6 ROS2 node...")
        node = Parol6ROS2Node(namespace="/parol6")

        print(f"✓ Node created: {node.get_name()}")
        print(f"  - Namespace: {node.namespace}")
        print(f"  - Joint names: {node.joint_names}")

        # Spin for a bit
        print("\nSpinning node for 2 seconds...")
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

        print("\n✓ ROS2 node test successful!")
        return True

    except ImportError as e:
        print(f"✗ ROS2 not available: {e}")
        print("Make sure ROS2 is installed and sourced")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_ros2_bridge():
    """Test ROS2 bridge functionality."""
    print("\n" + "=" * 60)
    print("Test: ROS2 Bridge")
    print("=" * 60)

    try:
        from ros2_adapter.parol6_adapter import Parol6ROS2Bridge

        print("Creating ROS2 bridge...")
        bridge = Parol6ROS2Bridge(namespace="/parol6", control_frequency=100)

        print("✓ Bridge created")
        print(f"  - Namespace: {bridge.namespace}")
        print(f"  - Control frequency: {bridge.control_frequency} Hz")

        print("\nConnecting bridge...")
        bridge.connect()

        print(f"✓ Bridge connected: {bridge.is_connected}")

        # Test position reading
        print("\nReading joint positions...")
        positions = bridge.get_joint_positions()
        print(f"  Positions: {positions}")

        # Test position sending
        print("\nSending test positions...")
        test_positions = np.array([0.1, -0.2, 0.3, -0.1, 0.2, 0.0])
        bridge.send_joint_positions(test_positions)
        print(f"  Sent: {test_positions}")

        time.sleep(1.0)

        # Read back
        new_positions = bridge.get_joint_positions()
        print(f"  Read back: {new_positions}")

        # Cleanup
        bridge.disconnect()
        print("\n✓ ROS2 bridge test successful!")
        return True

    except ImportError as e:
        print(f"✗ ROS2 not available: {e}")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_ros2_topics():
    """Test ROS2 topic communication."""
    print("\n" + "=" * 60)
    print("Test: ROS2 Topic Communication")
    print("=" * 60)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        rclpy.init()

        # Create test node
        node = Node('test_parol6_topics')

        print("✓ Test node created")

        # Subscribe to joint states
        joint_states_received = []

        def joint_state_callback(msg):
            joint_states_received.append(msg)
            print(f"  Received joint state: {len(msg.position)} joints")

        sub = node.create_subscription(
            JointState,
            '/parol6/joint_states',
            joint_state_callback,
            10
        )

        # Publish test joint state
        pub = node.create_publisher(
            JointState,
            '/parol6/joint_commands',
            10
        )

        print("\nPublishing test joint commands...")
        for i in range(5):
            msg = JointState()
            msg.name = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
            msg.position = [0.1 * i] * 6
            pub.publish(msg)
            print(f"  Published command {i+1}/5")

            # Spin to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.2)

        print(f"\nReceived {len(joint_states_received)} joint state messages")

        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

        print("\n✓ ROS2 topic test successful!")
        return True

    except ImportError as e:
        print(f"✗ ROS2 not available: {e}")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all ROS2 tests."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Test PAROL6 ROS2 integration"
    )
    parser.add_argument(
        "--test",
        type=str,
        choices=["node", "bridge", "topics", "all"],
        default="all",
        help="Which test to run"
    )

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("PAROL6 ROS2 Integration Tests")
    print("=" * 60)
    print("NOTE: These tests require ROS2 to be installed and sourced")
    print("=" * 60)

    results = {}

    if args.test in ["node", "all"]:
        results["node"] = test_ros2_node()

    if args.test in ["bridge", "all"]:
        results["bridge"] = test_ros2_bridge()

    if args.test in ["topics", "all"]:
        results["topics"] = test_ros2_topics()

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    for test_name, success in results.items():
        status = "✓ PASSED" if success else "✗ FAILED"
        print(f"{test_name}: {status}")
    print("=" * 60)

    all_passed = all(results.values())
    if all_passed:
        print("All tests passed!")
        return 0
    else:
        print("Some tests failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
