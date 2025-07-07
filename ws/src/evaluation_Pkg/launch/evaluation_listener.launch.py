import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os

def launch_setup(context, *args, **kwargs):
    num_nodes = int(LaunchConfiguration('num_nodes').perform(context))
    frequency = float(LaunchConfiguration('frequency').perform(context))
    test_type = LaunchConfiguration('test_type').perform(context)

    nodes = []


    for i in range(num_nodes):
        base_path = f"latencies/{test_type}/{frequency}/{num_nodes}/{i}"
        topic_name = f"/topic_{i}"

        params = [
            {"frequency": frequency},
            {"topic": topic_name},
            {"log_base_path": base_path}
        ]

        nodes.append(
            launch_ros.actions.Node(
                package='evaluationpkg',
                executable='listener',
                name=f'listener_{i}',
                namespace='listeners',
                parameters=params
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_nodes', default_value='1', description='Anzahl Talker+Listener Paare'),
        DeclareLaunchArgument('frequency', default_value='2', description='Frequenz in Hz'),
        OpaqueFunction(function=launch_setup)
    ])
