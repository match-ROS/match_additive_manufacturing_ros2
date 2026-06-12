from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('base_trajectory_follower'),
        'config',
        'rbvogui_simple_base_follower.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_topic', default_value='/base_path'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/robot/robotnik_base_control/cmd_vel_unstamped'),
        DeclareLaunchArgument('output_stamped', default_value='false'),
        DeclareLaunchArgument('command_frame_id', default_value='base_link'),
        DeclareLaunchArgument('follower_type', default_value='pid'),
        DeclareLaunchArgument('diff_drive_mode', default_value='false'),
        DeclareLaunchArgument('use_external_path_index', default_value='false'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='false'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_condition'),
        DeclareLaunchArgument('velocity_override_topic', default_value='/velocity_override'),
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
                    'command_frame_id': LaunchConfiguration('command_frame_id'),
                    'follower_type': LaunchConfiguration('follower_type'),
                    'diff_drive_mode': LaunchConfiguration('diff_drive_mode'),
                    'use_external_path_index': LaunchConfiguration('use_external_path_index'),
                    'path_index_topic': LaunchConfiguration('path_index_topic'),
                    'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                    'start_condition_topic': LaunchConfiguration('start_condition_topic'),
                    'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
                },
            ],
        ),
    ])
