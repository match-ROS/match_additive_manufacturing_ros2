from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('am_bringup'),
        'config',
        'rbvogui_path_following_demo.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_type', default_value='line'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('path_topic', default_value='/base_path'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/robot/robotnik_base_control/cmd_vel_unstamped'),
        DeclareLaunchArgument('output_stamped', default_value='false'),
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
                    'frame_id': LaunchConfiguration('path_frame'),
                    'path_topic': LaunchConfiguration('path_topic'),
                },
            ],
        ),
        Node(
            package='base_trajectory_follower',
            executable='simple_base_follower',
            name='simple_base_follower',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'path_topic': LaunchConfiguration('path_topic'),
                    'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'output_stamped': LaunchConfiguration('output_stamped'),
                },
            ],
        ),
    ])
