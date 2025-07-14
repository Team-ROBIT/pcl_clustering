#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    pkg_path = get_package_share_directory('pcl_clustering')
    param_file = os.path.join(pkg_path, 'config', 'cluster.yaml')

    pcl_clustering_node = Node(
        package='pcl_clustering',            
        executable='pcl_clustering',         
        name='pcl_clustering',               
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        pcl_clustering_node,
    ])