"""Publish the calibrated nozzle and deposition pose used by arm motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('tip_link', default_value='robot_arm_tool0'),
        DeclareLaunchArgument('publish_tcp_pose_from_tf', default_value='false'),
        DeclareLaunchArgument('tcp_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('derive_nozzle_pose_from_tcp', default_value='false'),
        DeclareLaunchArgument('nozzle_pose_topic', default_value='/current_nozzle_tip_pose'),
        DeclareLaunchArgument('deposition_pose_topic', default_value='/current_deposition_pose'),
        DeclareLaunchArgument('fixed_tool_offset_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument(
            'fixed_tool_offset_quaternion_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'
        ),
        DeclareLaunchArgument('spray_distance_initial', default_value='0.1'),
        DeclareLaunchArgument('spray_distance_max_rate', default_value='0.02'),
        Node(
            package='ur_trajectory_follower', executable='current_pose_from_tf',
            name='current_tcp_pose', output='screen',
            condition=IfCondition(LaunchConfiguration('publish_tcp_pose_from_tf')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': LaunchConfiguration('path_frame'),
                'source_frame': LaunchConfiguration('tip_link'),
                'pose_topic': LaunchConfiguration('tcp_pose_topic'),
            }],
        ),
        Node(
            package='ur_trajectory_follower', executable='fixed_tool_pose',
            name='fixed_tool_pose', output='screen',
            condition=IfCondition(LaunchConfiguration('derive_nozzle_pose_from_tcp')),
            parameters=[{
                'input_pose_topic': LaunchConfiguration('tcp_pose_topic'),
                'output_pose_topic': LaunchConfiguration('nozzle_pose_topic'),
                'fixed_tool_offset_xyz': LaunchConfiguration('fixed_tool_offset_xyz'),
                'fixed_tool_offset_quaternion_xyzw': LaunchConfiguration(
                    'fixed_tool_offset_quaternion_xyzw'
                ),
            }],
        ),
        Node(
            package='ur_trajectory_follower', executable='deposition_pose',
            name='deposition_pose', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'nozzle_pose_topic': LaunchConfiguration('nozzle_pose_topic'),
                'deposition_pose_topic': LaunchConfiguration('deposition_pose_topic'),
                'spray_distance_topic': '/spray_distance',
                'smoothed_spray_distance_topic': '/spray_distance_smoothed',
                'spray_distance_initial': LaunchConfiguration('spray_distance_initial'),
                'spray_distance_max_rate': LaunchConfiguration('spray_distance_max_rate'),
            }],
        ),
    ])
