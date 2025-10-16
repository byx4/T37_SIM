from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument('map_path', default_value='')
    map_path = LaunchConfiguration('map_path')

    sim_node = Node(
        package='sim_core',
        executable='sim_loop',
        name='sim_loop',
        output='screen',
        parameters=[{
            'map_path': map_path,
            'map_frame_id': 'map',
            'dt': 0.02,
            'L': 1.6,
            'v_max': 8.0,
            'accel_gain': 2.5,
            'delta_max_deg': 28.0,
            'cmd_timeout': 0.25,
            'yaw_rate_limit': 3.0,
            'cones_rate_hz': 10.0,
            'range_x_max': 25.0,
            'range_y_abs_max': 10.0,
            'max_cones_per_msg': 50,
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '' ],  # (optional) provide a .rviz config path if you create one
        output='screen'
    )

    return LaunchDescription([map_path_arg, sim_node, rviz])
