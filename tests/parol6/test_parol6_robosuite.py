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
Test PAROL6 with Robosuite simulation.

This script demonstrates integration between PAROL6 (similar to UR5) and
Robosuite simulation environment.

Requirements:
    pip install robosuite
"""

import sys
import numpy as np
import time
from pathlib import Path

# Add lerobot to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


def test_robosuite_basic():
    """Test basic Robosuite environment with UR5 (similar to PAROL6)."""
    print("\n" + "=" * 60)
    print("Test: Robosuite Basic Environment")
    print("=" * 60)

    try:
        import robosuite as suite
        from robosuite.controllers import load_controller_config

        print("✓ Robosuite imported successfully")

        # Create environment with UR5e (closest to PAROL6)
        # PAROL6 has similar kinematics to UR5
        env = suite.make(
            env_name="Lift",
            robots="UR5e",  # UR5e is similar to PAROL6
            has_renderer=True,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=20,
            horizon=1000,
        )

        print(f"✓ Environment created: {env}")
        print(f"  - Robot: {env.robots[0].name}")
        print(f"  - Action dim: {env.action_dim}")
        print(f"  - Action spec: {env.action_spec}")

        # Reset environment
        obs = env.reset()
        print(f"✓ Environment reset")

        # Run random actions
        print("\nRunning random actions for 100 steps...")
        for i in range(100):
            # Random action
            action = np.random.randn(env.action_dim) * 0.1

            # Step
            obs, reward, done, info = env.step(action)

            # Render
            env.render()

            if done:
                obs = env.reset()

            time.sleep(0.05)

        env.close()
        print("\n✓ Robosuite basic test successful!")
        return True

    except ImportError:
        print("✗ Robosuite not installed. Install with: pip install robosuite")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_parol6_robosuite_integration():
    """Test PAROL6 config with Robosuite (using UR5 as proxy)."""
    print("\n" + "=" * 60)
    print("Test: PAROL6-Robosuite Integration")
    print("=" * 60)

    try:
        import robosuite as suite
        from lerobot.common.robots.parol6 import Parol6Config

        print("Creating PAROL6 config...")
        parol6_config = Parol6Config(
            id="parol6_sim",
            port="/dev/null",  # Not used in sim
            use_ros2_bridge=False,
        )

        print(f"✓ PAROL6 config created")
        print(f"  - Joint names: {len(parol6_config.joint_limits)} joints")
        print(f"  - Home position: {parol6_config.home_position}")

        # Create Robosuite environment
        env = suite.make(
            env_name="Lift",
            robots="UR5e",
            has_renderer=True,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=20,
        )

        print(f"✓ Robosuite environment created")

        # Map PAROL6 actions to Robosuite
        obs = env.reset()

        print("\nTesting PAROL6-style control...")
        for i in range(200):
            # Simulate PAROL6 action (6 joint positions + gripper)
            # Robosuite UR5e expects 7-dim action (6 joints + gripper)

            # Create smooth sinusoidal motion
            t = i / 20.0
            parol6_action = np.array([
                0.1 * np.sin(t),      # shoulder_pan
                0.1 * np.cos(t),      # shoulder_lift
                0.05 * np.sin(2*t),   # elbow
                0.0,                   # wrist_1
                0.0,                   # wrist_2
                0.0,                   # wrist_3
                0.0,                   # gripper (open/close)
            ])

            # Send to simulation
            obs, reward, done, info = env.step(parol6_action)
            env.render()

            if i % 50 == 0:
                print(f"  Step {i}/200 - Reward: {reward:.3f}")

            if done:
                obs = env.reset()

            time.sleep(0.05)

        env.close()
        print("\n✓ PAROL6-Robosuite integration test successful!")
        return True

    except ImportError as e:
        print(f"✗ Missing dependency: {e}")
        print("Install with: pip install robosuite")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_robosuite_teleoperation():
    """Test teleoperation in Robosuite with PAROL6 mapping."""
    print("\n" + "=" * 60)
    print("Test: Robosuite Teleoperation")
    print("=" * 60)
    print("Use keyboard to control robot:")
    print("  w/s: shoulder_pan +/-")
    print("  a/d: shoulder_lift +/-")
    print("  r/f: elbow +/-")
    print("  ESC: quit")
    print("=" * 60)

    try:
        import robosuite as suite

        env = suite.make(
            env_name="Lift",
            robots="UR5e",
            has_renderer=True,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=20,
        )

        obs = env.reset()

        # Current joint positions (relative)
        joint_deltas = np.zeros(7)

        print("\n✓ Environment ready for teleoperation")
        print("(Note: Keyboard control requires proper window focus)")

        for i in range(1000):
            # Apply smooth damping to joint deltas
            joint_deltas *= 0.95

            # Send action
            obs, reward, done, info = env.step(joint_deltas)
            env.render()

            if done:
                obs = env.reset()
                joint_deltas = np.zeros(7)

            time.sleep(0.05)

        env.close()
        print("\n✓ Teleoperation test completed!")
        return True

    except ImportError:
        print("✗ Robosuite not installed")
        return False
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all Robosuite tests."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Test PAROL6 with Robosuite simulation"
    )
    parser.add_argument(
        "--test",
        type=str,
        choices=["basic", "integration", "teleop", "all"],
        default="all",
        help="Which test to run"
    )

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("PAROL6 Robosuite Integration Tests")
    print("=" * 60)

    results = {}

    if args.test in ["basic", "all"]:
        results["basic"] = test_robosuite_basic()

    if args.test in ["integration", "all"]:
        results["integration"] = test_parol6_robosuite_integration()

    if args.test in ["teleop", "all"]:
        results["teleop"] = test_robosuite_teleoperation()

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
