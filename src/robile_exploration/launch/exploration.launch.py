#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robile_exploration'),
        'config',
        'exploration_params.yaml'
    )

    explorer_node = Node(
        package='robile_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        explorer_node,
    ])
