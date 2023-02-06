from ament_index_python.packages import get_package_share_path
import launch
from launch.substitutions import Command
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('urdf_tutorial') / 'urdf/01-myfirst.urdf')]),
        value_type=str)

    return launch.LaunchDescription([
        launch_ros.actions.Node(package='robot_state_publisher', executable='robot_state_publisher',
                                parameters=[{'robot_description': robot_description}])
    ])
