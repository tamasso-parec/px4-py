
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_py')

    gui = Node(
		package='px4_py',
		executable='gui',
		name='gui_node',
		output='screen'
	)

    px4_py = Node(
		package='px4_py',
		executable='px4_py',
		name='px4_py_node',
		output='screen'
	)
    return LaunchDescription([
        gui, 
        px4_py
    ])
