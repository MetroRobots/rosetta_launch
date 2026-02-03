from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('donatello'), 'launch', '05-arg.launch.py']),
                                 launch_arguments={'pizza_type': 'peppers'}.items()),
    ])
