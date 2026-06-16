from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import AnonName, Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch.substitutions import PythonExpression, ThisLaunchFile, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    object_parameters = {
        'pizza': LaunchConfiguration('pizza_type'),
        'anonymous_name': AnonName('leo'),
        'favorite_brother': EnvironmentVariable('BROTHER_NAME', default_value='mikey'),
        'filename': ThisLaunchFile(),
        'directory': ThisLaunchFileDir(),
        'list_exec': FindExecutable(name='ls'),
        'list_output': Command('ls'),
        'version': ['ROS ', EnvironmentVariable('ROS_VERSION')],
        'circumference': PythonExpression(['2.*3.1415*', LaunchConfiguration('radius')]),
    }

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[object_parameters]),
    ])
