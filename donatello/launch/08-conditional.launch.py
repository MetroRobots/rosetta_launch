from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_number_one', default_value='True'),
        IncludeLaunchDescription(
            str(get_package_share_path('donatello') / 'launch/01-single.launch.py'),
            condition=IfCondition(LaunchConfiguration('use_number_one')),
        ),
        IncludeLaunchDescription(
            str(get_package_share_path('donatello') / 'launch/02-param.launch.py'),
            condition=UnlessCondition(LaunchConfiguration('use_number_one')),
        ),
    ])
