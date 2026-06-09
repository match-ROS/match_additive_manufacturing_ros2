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
        DeclareLaunchArgument('path_index', default_value='0'),
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
                'path_index': LaunchConfiguration('path_index'),
            }],
        ),
    ])
