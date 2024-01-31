from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    don_node = Node(name='does_machines', package='donatello', executable='donatello_node')
    five_node = Node(name='five_seconds', package='donatello', executable='five_seconds')
    handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=five_node,
            on_exit=[
                LogInfo(
                    msg='Five node exited; tearing down entire system.'),
                EmitEvent(
                    event=Shutdown())]))

    return LaunchDescription([don_node, five_node, handler])
