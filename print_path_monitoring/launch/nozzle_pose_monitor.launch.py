from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('print_path_monitoring'),
        'config',
        'nozzle_pose_monitor.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('tcp_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('reference_path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        Node(
            package='print_path_monitoring',
            executable='nozzle_pose_monitor',
            name='nozzle_pose_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'tcp_pose_topic': LaunchConfiguration('tcp_pose_topic'),
                    'reference_path_topic': LaunchConfiguration('reference_path_topic'),
                    'path_index_topic': LaunchConfiguration('path_index_topic'),
                },
            ],
        ),
    ])
