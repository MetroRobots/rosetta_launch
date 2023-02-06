from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    dynamic_param_path = [get_package_share_directory('donatello'), '/config/', LaunchConfiguration('config'), '.yaml']

    return launch.LaunchDescription([
        DeclareLaunchArgument('config', default_value='params'),
        launch_ros.actions.Node(package='donatello', executable='donatello_node', name='does_machines',
                                parameters=[dynamic_param_path]),
    ])
