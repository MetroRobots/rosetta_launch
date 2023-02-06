import launch
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.frontend.parse_substitution import parse_substitution


def generate_launch_description():
    text_params = {
        'pizza': parse_substitution('$(var pizza_type)'),
        'anonymous_name': parse_substitution('$(anon leo)'),
        'favorite_brother': parse_substitution('$(env BROTHER_NAME mikey)'),
        'filename': parse_substitution('$(filename)'),
        'directory': parse_substitution('$(dirname)'),
        'list_exec': parse_substitution('$(find-exec ls)'),
        'list_output': parse_substitution('$(command ls)'),
        'version': parse_substitution('ROS $(env ROS_VERSION)'),
        'circumference': parse_substitution('$(eval 2.*3.1415*$(var radius))'),
    }

    return launch.LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.5'),
        DeclareLaunchArgument('pizza_type', default_value='mushrooms'),
        launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node',
                                parameters=[text_params]),
    ])
