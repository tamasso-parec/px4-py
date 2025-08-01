import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    
    set_robot_id_env = SetEnvironmentVariable('TRACKED_ROBOT_ID', '44')


    dds_cmd = ExecuteProcess(
            # cmd=['gnome-terminal', '--', "MicroXRCEAgent", "serial", "--dev", "/dev/ttyS2", "-b", "921600"],
            cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/ttyS2", "-b", "921600"],
            cwd=os.getcwd(),
            output='screen'
        )
    # opti_command = ExecuteProcess(
    #         # cmd=['gnome-terminal', '--', "TRACKED_ROBOT_ID=44", "ros2", "run", "optitrack_interface", "optitrack"],
    #         cmd=["TRACKED_ROBOT_ID=44", "ros2", "run", "optitrack_interface", "optitrack"],
    #         cwd=os.getcwd(),
    #         output='screen'
    #     )
    opti_node = Node(
            package='optitrack_interface',
            namespace='optitrack_interface',
            executable='optitrack',
            name='optitrack'
            # prefix='gnome-terminal --tab --',
        )
    
    # opti_command= ExecuteProcess(
    #         # cmd=['gnome-terminal', '--', "TRACKED_ROBOT_ID=44", "ros2", "run", "optitrack_interface", "optitrack"],
    #         cmd=["TRACKED_ROBOT_ID=44", "ros2", "run", "optitrack_interface", "optitrack"],
    #         cwd=os.getcwd(),
    #         output='screen'
    #     )

    opti2px4_node = Node(
            package='px4_py',
            namespace='px4_py',
            executable='opti_to_px4',
            name='opti_to_px4'
            # prefix='gnome-terminal --tab --',
        )

    # Define the launch description
    return LaunchDescription([
        dds_cmd,
        opti_node,
        opti2px4_node
    ])