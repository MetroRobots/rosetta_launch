from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value='true'),
        Node(name='does_machines',
             package='donatello',
             executable='donatello_node',
             parameters=[{'pizza': 'mushrooms',
                          'brothers': ['leo', 'mike', 'raph']}]),
    ])
