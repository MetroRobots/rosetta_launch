from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('N', default_value='10'),
        launch_ros.actions.Node(name=['donatello_node', LaunchConfiguration('N')],
                                package='donatello', executable='donatello_node'),
        IncludeLaunchDescription(str(get_package_share_path('donatello') / 'launch/11-recursion.launch.py'),
                                 condition=IfCondition(PythonExpression([LaunchConfiguration('N'), '>1'])),
                                 launch_arguments={'N': PythonExpression([LaunchConfiguration('N'), '-1'])}.items()),
    ])
