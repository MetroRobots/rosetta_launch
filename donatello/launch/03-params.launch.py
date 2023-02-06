from ament_index_python.packages import get_package_share_path
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[
                                    str(get_package_share_path('donatello') / 'config/params.yaml')]),
    ])
