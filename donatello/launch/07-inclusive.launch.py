from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(str(get_package_share_path('donatello') / 'launch/05-arg.launch.py'),
                                 launch_arguments={'pizza_type': 'peppers'}.items()),
    ])
