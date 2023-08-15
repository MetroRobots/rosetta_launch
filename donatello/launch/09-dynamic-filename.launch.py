import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamic_param_path = [FindPackageShare('donatello'), '/config/', LaunchConfiguration('config'), '.yaml']

    return launch.LaunchDescription([
        DeclareLaunchArgument('config', default_value='params'),
        launch_ros.actions.Node(package='donatello', executable='donatello_node', name='does_machines',
                                parameters=[dynamic_param_path]),
    ])
