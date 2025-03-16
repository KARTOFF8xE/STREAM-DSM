import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='talker',
            name='saturn_talker',
            namespace='hallo/saturn'
        ),
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='listener',
            name='saturn_listener',
            namespace='hallo/saturn'
        ),
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='talker',
            name='erde_talker',
            namespace='hallo/erde'
        ),
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='listener',
            name='erde_listener',
            namespace='hallo/erde'
        )
    ])