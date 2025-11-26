#!/usr/bin/env python3

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
ROS2 Launch file for PAROL6 robot with LeRobot integration.

This launch file starts:
1. PAROL6 ROS2 node
2. Robot state publisher
3. Joint state publisher (optional, for visualization)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (no real robot connection)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/parol6',
        description='ROS2 namespace for PAROL6 topics'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='100',
        description='Joint state publish frequency (Hz)'
    )

    # PAROL6 ROS2 Node
    parol6_node = Node(
        package='parol6_ros2_adapter',
        executable='parol6_ros2_node',
        name='parol6_ros2_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim': LaunchConfiguration('use_sim'),
            'publish_frequency': LaunchConfiguration('publish_frequency'),
        }],
    )

    # Robot State Publisher (for TF and visualization)
    # Note: Requires URDF file for PAROL6
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            # 'robot_description': # Load from URDF file
            'publish_frequency': LaunchConfiguration('publish_frequency'),
        }],
        # Uncomment when URDF is available:
        # arguments=[PathJoinSubstitution([
        #     FindPackageShare('parol6_description'),
        #     'urdf',
        #     'parol6.urdf'
        # ])]
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('parol6_ros2_adapter'),
                'config',
                'parol6.rviz'
            ])
        ] if False else [],  # Set to True when config file exists
    )

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        namespace_arg,
        use_rviz_arg,
        publish_frequency_arg,

        # Nodes
        parol6_node,
        # robot_state_publisher,  # Uncomment when URDF is ready
        # rviz_node,  # Uncomment for visualization
    ])
