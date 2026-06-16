from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamic_param_path = PathJoinSubstitution([
        FindPackageShare('donatello'),
        'config',
        [LaunchConfiguration('config'), '.yaml']
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='params'),
        Node(package='donatello',
             executable='donatello_node',
             name='does_machines',
             parameters=[dynamic_param_path]),
    ])
