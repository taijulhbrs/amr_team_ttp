#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robile_planning'),
        'config',
        'planning_params.yaml'
    )

    global_planner_node = Node(
        package='robile_planning',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[config],
    )

    potential_field_node = Node(
        package='robile_planning',
        executable='potential_field_planner',
        name='potential_field_planner',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        global_planner_node,
        potential_field_node,
    ])
