from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    don_node = Node(name='does_machines', package='donatello', executable='donatello_node')
    five_node = Node(name='five_seconds', package='donatello', executable='five_seconds', on_exit=Shutdown())
    return LaunchDescription([don_node, five_node])
