"""Launch li_init for Livox Horizon (config/horizon.yaml)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'lidar_imu_init'
    share = get_package_share_directory(pkg)

    with open(os.path.join(share, 'config', 'horizon.yaml')) as f:
        params = yaml.safe_load(f)
    params.update({'point_filter_num': 3, 'max_iteration': 5, 'cube_side_length': 2000.0})

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    laser_mapping = Node(
        package=pkg,
        executable='li_init',
        name='laserMapping',
        output='screen',
        parameters=[params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz_cfg', 'fast_lo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, laser_mapping, rviz])
