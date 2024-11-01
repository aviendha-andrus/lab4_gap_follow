from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
	wall_node = Node(
		package='gap_follow',
		executable='reactive_node.py',
		name='reactive_node',
		output='screen',
		parameters=[{'bubble':LaunchConfiguration('bubble', default='0.0'),
			'maxvel':LaunchConfiguration('maxvel', default='0.0')}
		]
	)

	return LaunchDescription([
        wall_node
	])
