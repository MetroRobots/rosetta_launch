from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[PathJoinSubstitution([FindPackageShare('donatello'), 'config', 'params.yaml'])]),
    ])
