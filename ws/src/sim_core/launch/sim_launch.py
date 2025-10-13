from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_core',
            executable='mimic_perception',
            name='mimic_perception',
            output='screen'
        ),
        Node(
            package='sim_core',
            executable='vehicle_dynamics',
            name='vehicle_dynamics',
            output='screen'
        )
    ])
