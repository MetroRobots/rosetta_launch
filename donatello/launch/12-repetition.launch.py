import launch
from launch.actions import DeclareLaunchArgument, ForLoop
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def for_i(i: int):
    return [
        # i will be [0, N), so use i+1 to get [1, N]
        launch_ros.actions.Node(name=['donatello_node', str(i + 1)],
                                package='donatello', executable='donatello_node'),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('N', default_value='10'),
        ForLoop(LaunchConfiguration('N'), function=for_i),
    ])
