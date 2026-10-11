#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('robile_localization'),
        'config', 'amcl_params.yaml')

    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_arg = DeclareLaunchArgument(
        'map_yaml', description='Full path to the map yaml file')
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[default_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        declare_map_arg,
        declare_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager,
    ])
