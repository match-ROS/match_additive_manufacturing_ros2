from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_topic', default_value='/mobile_base_path'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('robot_pose_type', default_value='pose_stamped'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/diff_drive_controller/cmd_vel'),
        DeclareLaunchArgument('output_stamped', default_value='false'),
        DeclareLaunchArgument('command_frame_id', default_value='base_link'),
        DeclareLaunchArgument('diff_drive_mode', default_value='true'),
        DeclareLaunchArgument('path_index', default_value='0'),
        DeclareLaunchArgument('target_yaw_mode', default_value='auto'),
        DeclareLaunchArgument('target_yaw_lookahead_points', default_value='10'),
        DeclareLaunchArgument('publish_start_condition', default_value='false'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_condition'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='move_to_path_idx',
            executable='move_to_path_idx',
            name='move_to_path_idx',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': LaunchConfiguration('path_topic'),
                'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
                'robot_pose_type': LaunchConfiguration('robot_pose_type'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'output_stamped': LaunchConfiguration('output_stamped'),
                'command_frame_id': LaunchConfiguration('command_frame_id'),
                'diff_drive_mode': LaunchConfiguration('diff_drive_mode'),
                'path_index': LaunchConfiguration('path_index'),
                'target_yaw_mode': LaunchConfiguration('target_yaw_mode'),
                'target_yaw_lookahead_points': LaunchConfiguration('target_yaw_lookahead_points'),
                'publish_start_condition': LaunchConfiguration('publish_start_condition'),
                'start_condition_topic': LaunchConfiguration('start_condition_topic'),
            }],
        ),
    ])
