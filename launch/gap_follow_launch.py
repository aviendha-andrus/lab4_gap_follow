#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
import argparse

# parser = argparse.ArgumentParser(description='bubble:=#   the size of the safety bubble in meters\nmaxvel:=#   the maximum velocity of the vehicle in meters/sec')
# args = parser.parse_args()

def generate_launch_description():
	bubble_arg = DeclareLaunchArgument('b', default_value='0.0')
	maxvel_arg = DeclareLaunchArgument('v', default_value='0.0')

	bubble = LaunchConfiguration('b')
	maxvel = LaunchConfiguration('v')

	gap_node = Node(
		package='gap_follow',
		executable='reactive_node.py',
		name='reactive_node',
		output='screen',
		parameters=[
			{'b': bubble},
			{'v': maxvel},
		]
	)

	return LaunchDescription([
        gap_node,
		bubble_arg,
		maxvel_arg,
	])
