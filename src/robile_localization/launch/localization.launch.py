#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robile_localization'),
        'config',
        'localization_params.yaml'
    )

    particle_filter_node = Node(
        package='robile_localization',
        executable='particle_filter',
        name='particle_filter',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        particle_filter_node,
    ])
