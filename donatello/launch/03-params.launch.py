import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[
                                    [FindPackageShare('donatello'), '/config/params.yaml']]),
    ])
