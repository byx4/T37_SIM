# ws/src/sim_viz/launch/viz.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch args
    map_yaml   = LaunchConfiguration('map_yaml')
    world      = LaunchConfiguration('world_frame')
    mapf       = LaunchConfiguration('map_frame')
    base       = LaunchConfiguration('base_frame')
    cone_scale = LaunchConfiguration('cone_scale')

    return LaunchDescription([
        # --------- Arguments ----------
        DeclareLaunchArgument('map_yaml', description='Path to FSG map YAML'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('map_frame',   default_value='map'),
        DeclareLaunchArgument('base_frame',  default_value='base_link'),
        DeclareLaunchArgument('cone_scale',  default_value='0.35'),  # meters

        # --------- Static TF: world -> map (identity) ----------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            # x y z qx qy qz qw parent child
            arguments=['0', '0', '0', '0', '0', '0', world, mapf],
            output='screen',
        ),

        # --------- Global cones in MAP frame ----------
        Node(
            package='sim_viz',
            executable='publish_cone_map',
            name='global_cone_markers',
            output='screen',
            parameters=[
                {'map_yaml': map_yaml},
                {'viz_frame': mapf},          # force markers to map
                {'point_size': cone_scale},   # publisher expects 'point_size'
                {'cone_scale': cone_scale},   # harmless: in case you read this key in code
            ],
        ),

        # --------- Pose -> TF (vehicle_pose -> base_link) ----------
        Node(
            package='sim_viz',
            executable='pose_to_tf',
            name='pose_to_tf',
            output='screen',
            parameters=[
                {'pose_topic': '/vehicle_pose'},
                {'base_frame': base},
            ],
        ),

        # --------- Foxglove Bridge ----------
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765}],
        ),
    ])
