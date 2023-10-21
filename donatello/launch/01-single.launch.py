from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(name='does_machines', package='donatello', executable='donatello_node'),
    ])
