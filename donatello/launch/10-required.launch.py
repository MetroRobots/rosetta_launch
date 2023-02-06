import launch
import launch_ros.actions


def generate_launch_description():
    don_node = launch_ros.actions.Node(name='does_machines', package='donatello', executable='donatello_node')
    five_node = launch_ros.actions.Node(name='five_seconds', package='donatello', executable='five_seconds')
    handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=five_node,
            on_exit=[
                launch.actions.LogInfo(
                    msg='Five node exited; tearing down entire system.'),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown())]))

    return launch.LaunchDescription([don_node, five_node, handler])
