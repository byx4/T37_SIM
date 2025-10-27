from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    world = LaunchConfiguration('world_frame')
    mapf = LaunchConfiguration('map_frame')
    base = LaunchConfiguration('base_frame')
    cone_scale = LaunchConfiguration('cone_scale')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='Path to FSG map YAML'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('cone_scale', default_value='0.30'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0','0','0','0','0','0', world, mapf],
            output='screen'
        ),
        Node(
            package='sim_viz',
            executable='publish_cone_map',
            name='global_cone_markers',
            parameters=[{'map_yaml': map_yaml, 'cone_scale': cone_scale}],
            output='screen'
        ),
        Node(
            package='sim_viz',
            executable='pose_to_tf',
            name='pose_to_tf',
            parameters=[{'pose_topic': '/vehicle_pose', 'base_frame': base}],
            output='screen'
        ),
    ])
