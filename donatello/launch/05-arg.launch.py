from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[{'pizza': LaunchConfiguration('pizza_type')}]),
    ])
