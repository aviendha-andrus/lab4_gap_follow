#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	# kp_arg = DeclareLaunchArgument('kp', default_value='0.0')
	# kd_arg = DeclareLaunchArgument('kd', default_value='0.0')
	# ki_arg = DeclareLaunchArgument('ki', default_value='0.0')
	# speed_arg = DeclareLaunchArgument('speed', default_value='0.0')
	# dist_arg = DeclareLaunchArgument('dist', default_value='0.0')
	# # theta_arg = DeclareLaunchArgument('theta', default_value='0.0')
	# # look_arg = DeclareLaunchArgument('look', default_value='0.0')

	# kp = LaunchConfiguration('kp')
	# kd = LaunchConfiguration('kd')
	# ki = LaunchConfiguration('ki')
	# speed = LaunchConfiguration('speed')
	# dist = LaunchConfiguration('dist')
	# theta = LaunchConfiguration('speed')
	# look = LaunchConfiguration('dist')

	gap_node = Node(
        package='gap_follow',
        executable='reactive_node.py',
        name='wall_follow_node'
		# name=LaunchConfiguration('student'),
		# output='screen',
		# parameters=[
		# 	{'kp': kp},
		# 	{'kd': kd},
		# 	{'ki': ki},
		# 	{'speed': speed},
		# 	{'dist': dist},
		# 	# {'theta': theta},
		# 	# {'look': look},
		# ]
    )

	return LaunchDescription([
        gap_node,
		# kp_arg,
		# kd_arg,
		# ki_arg,
		# speed_arg, 
		# dist_arg,
		# theta_arg, 
		# look_arg,
	])




# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='wall_follow',
#             executable='wall_follow_node.py',
#             name='wall_follow_node'
#         ),
#     ])
# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='gap_follow',
#             executable='reactive_node.py',
#             name='wall_follow_node'
#         ),
#     ])

# #!/usr/bin/env python3
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
# import argparse

# parser = argparse.ArgumentParser(description='bubble:=#   the size of the safety bubble in meters\nmaxvel:=#   the maximum velocity of the vehicle in meters/sec')
# args = parser.parse_args()

# def generate_launch_description():
# 	bubble_arg = DeclareLaunchArgument('bubble', default_value='0.0')
# 	maxvel_arg = DeclareLaunchArgument('maxvel', default_value='0.0')

# 	bubble = LaunchConfiguration('bubble')
# 	maxvel = LaunchConfiguration('maxvel')

# 	gap_node = Node(
# 		package='gap_follow',
# 		executable='reactive_node.py',
# 		name='reactive_node',
# 		# name=LaunchConfiguration('student'),
# 		output='screen',
# 		parameters=[
# 			{'bubble': bubble},
# 			{'maxvel': maxvel},
# 		]
# 	)

# 	return LaunchDescription([
#         gap_node,
# 		bubble_arg,
# 		maxvel_arg,
# 	])

