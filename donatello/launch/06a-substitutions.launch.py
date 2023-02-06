import launch
from launch.actions import DeclareLaunchArgument
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    object_parameters = {
        'pizza': launch.substitutions.LaunchConfiguration('pizza_type'),
        'anonymous_name': launch.substitutions.AnonName('leo'),
        'favorite_brother': launch.substitutions.EnvironmentVariable('BROTHER_NAME', default_value='mikey'),
        'filename': launch.substitutions.ThisLaunchFile(),
        'directory': launch.substitutions.ThisLaunchFileDir(),
        'list_exec': launch.substitutions.FindExecutable(name='ls'),
        'list_output': launch.substitutions.Command('ls'),
        'version': ['ROS ', launch.substitutions.EnvironmentVariable('ROS_VERSION')],
        'circumference': launch.substitutions.PythonExpression([
            '2.*3.1415*',
            launch.substitutions.LaunchConfiguration('radius')
        ]),
    }

    return launch.LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[object_parameters]),
    ])
