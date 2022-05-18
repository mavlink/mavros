#!/usr/bin/env python3
# Authors: Ryuji Yasukochi

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
	config_dir = LaunchConfiguration('config_dir',
		default=os.path.join(
			get_package_share_directory('mavros'),
			'param',
			"config.yaml"
			))
	plugin_config_dir = LaunchConfiguration('config_dir',
		default=os.path.join(
			get_package_share_directory('mavros'),
			'param',
			"plugin_lists.yaml"
			))

	return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
			parameters=[config_dir,plugin_config_dir],
			),
    ])
