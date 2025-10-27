from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Identity static transform: world -> map
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_map",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            output="screen",
        ),
    ])