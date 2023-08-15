import launch
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    file_parameters = ParameterFile(
        param_file=[FindPackageShare('donatello'), '/config/sub_params.yaml'],
        allow_substs=True
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[file_parameters]),
    ])
