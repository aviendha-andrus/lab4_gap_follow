#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	# bubble_arg = DeclareLaunchArgument('bubble', default_value='0.0')
	# maxvel_arg = DeclareLaunchArgument('maxvel', default_value='0.0')

	# bubble = LaunchConfiguration('bubble')
	# maxvel = LaunchConfiguration('maxvel')

	gap_node = Node(
        package='gap_follow',
        executable='reactive_node.py',
        name='wall_follow_node',
		# name=LaunchConfiguration('student'),
		output='screen',
		# parameters=[
		# 	{'bubble': bubble},
		# 	{'maxvel': maxvel},
		# ]
    )

	return LaunchDescription([
        gap_node,
		# bubble_arg, 
		# maxvel_arg, 
	])
