from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_number_one', default_value='True'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('donatello'), 'launch', '01-single.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_number_one')),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('donatello'), 'launch', '02-param.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('use_number_one')),
        ),
    ])
