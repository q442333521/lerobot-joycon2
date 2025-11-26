#!/usr/bin/env python

from setuptools import setup, find_packages

setup(
    name='parol6_ros2_adapter',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
    ],
    extras_require={
        'ros2': [
            # ROS2 dependencies (installed via apt/rosdep, not pip)
            # 'rclpy',
            # 'sensor_msgs',
            # 'trajectory_msgs',
            # 'control_msgs',
        ],
    },
    entry_points={
        'console_scripts': [
            'parol6_ros2_node = parol6_adapter.parol6_ros2_node:main',
        ],
    },
    author='LeRobot Team',
    description='ROS2 adapter for PAROL6 robot with LeRobot',
    license='Apache-2.0',
    keywords='robotics ros2 parol6 lerobot',
    python_requires='>=3.8',
)
