from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('parse_paths'),
        'config',
        'test_path_generator.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_type', default_value='line'),
        DeclareLaunchArgument('path_topic', default_value='/test_path'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='parse_paths',
            executable='test_path_generator',
            name='test_path_generator',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'path_type': LaunchConfiguration('path_type'),
                    'path_topic': LaunchConfiguration('path_topic'),
                    'frame_id': LaunchConfiguration('frame_id'),
                },
            ],
        ),
    ])
