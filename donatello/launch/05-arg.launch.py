import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[{'pizza': LaunchConfiguration('pizza_type')}]),
    ])
