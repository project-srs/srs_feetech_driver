from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():
    default_model_path = get_package_share_path('srs_feetech_driver') / 'urdf/turret.urdf'
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=str(default_model_path), description='Absolute path to robot urdf file'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='srs_feetech_driver',
            executable='pan_tilt_node',
        ),
        Node(
            package='srs_feetech_driver',
            executable='joy_to_cmd_rate',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(get_package_share_path('srs_feetech_driver') / 'rviz/turret.rviz')],
        ),
    ])
