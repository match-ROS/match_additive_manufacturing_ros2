from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_TRAJECTORY_DIRECTORY = str(
    Path(__file__).resolve().parents[2] / 'components' / 'robotnik_paired_demo'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('base_path_topic', default_value='/base_path'),
        DeclareLaunchArgument('base_original_path_topic', default_value='/base_path_original'),
        DeclareLaunchArgument('arm_path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('arm_original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('current_arm_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('use_current_poses', default_value='true'),
        DeclareLaunchArgument('base_start_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('base_start_offset', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
        DeclareLaunchArgument('arm_start_xyz', default_value='[0.6, 0.0, 0.8]'),
        DeclareLaunchArgument('sideways_distance', default_value='0.8'),
        DeclareLaunchArgument('diagonal_distance', default_value='0.8'),
        DeclareLaunchArgument('arm_xy_offset', default_value='[0.15, 0.0, 0.0]'),
        DeclareLaunchArgument('ramp_arm_xy_offset', default_value='true'),
        DeclareLaunchArgument('arm_height_delta', default_value='0.2'),
        DeclareLaunchArgument('min_reachable_radius', default_value='0.25'),
        DeclareLaunchArgument('max_reachable_radius', default_value='0.85'),
        DeclareLaunchArgument('nozzle_axis', default_value='[0.0, 1.0, 0.0]'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        DeclareLaunchArgument('publish_once', default_value='true'),
        DeclareLaunchArgument('wait_for_trigger', default_value='false'),
        DeclareLaunchArgument('trigger_topic', default_value='/start_pose_reached'),
        DeclareLaunchArgument('export_trajectories', default_value='false'),
        DeclareLaunchArgument('load_exported_trajectories', default_value='false'),
        DeclareLaunchArgument('trajectory_directory', default_value=DEFAULT_TRAJECTORY_DIRECTORY),
        DeclareLaunchArgument('base_trajectory_filename', default_value='base_path.json'),
        DeclareLaunchArgument('arm_trajectory_filename', default_value='arm_path.json'),
        DeclareLaunchArgument('normal_filename', default_value='normal_vector.json'),
        DeclareLaunchArgument('path_transform_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('path_transform_yaw_deg', default_value='0.0'),
        Node(
            package='parse_paths',
            executable='publish_robotnik_base_arm_paths',
            name='robotnik_base_arm_path_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('frame_id'),
                'base_path_topic': LaunchConfiguration('base_path_topic'),
                'base_original_path_topic': LaunchConfiguration('base_original_path_topic'),
                'arm_path_topic': LaunchConfiguration('arm_path_topic'),
                'arm_original_path_topic': LaunchConfiguration('arm_original_path_topic'),
                'normal_topic': LaunchConfiguration('normal_topic'),
                'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
                'current_arm_pose_topic': LaunchConfiguration('current_arm_pose_topic'),
                'use_current_poses': LaunchConfiguration('use_current_poses'),
                'base_start_xyz': LaunchConfiguration('base_start_xyz'),
                'base_start_offset': LaunchConfiguration('base_start_offset'),
                'base_yaw': LaunchConfiguration('base_yaw'),
                'arm_start_xyz': LaunchConfiguration('arm_start_xyz'),
                'sideways_distance': LaunchConfiguration('sideways_distance'),
                'diagonal_distance': LaunchConfiguration('diagonal_distance'),
                'arm_xy_offset': LaunchConfiguration('arm_xy_offset'),
                'ramp_arm_xy_offset': LaunchConfiguration('ramp_arm_xy_offset'),
                'arm_height_delta': LaunchConfiguration('arm_height_delta'),
                'min_reachable_radius': LaunchConfiguration('min_reachable_radius'),
                'max_reachable_radius': LaunchConfiguration('max_reachable_radius'),
                'nozzle_axis': LaunchConfiguration('nozzle_axis'),
                'num_points': LaunchConfiguration('num_points'),
                'time_step': LaunchConfiguration('time_step'),
                'publish_once': LaunchConfiguration('publish_once'),
                'wait_for_trigger': LaunchConfiguration('wait_for_trigger'),
                'trigger_topic': LaunchConfiguration('trigger_topic'),
                'export_trajectories': LaunchConfiguration('export_trajectories'),
                'load_exported_trajectories': LaunchConfiguration('load_exported_trajectories'),
                'trajectory_directory': LaunchConfiguration('trajectory_directory'),
                'base_trajectory_filename': LaunchConfiguration('base_trajectory_filename'),
                'arm_trajectory_filename': LaunchConfiguration('arm_trajectory_filename'),
                'normal_filename': LaunchConfiguration('normal_filename'),
                'path_transform_xyz': LaunchConfiguration('path_transform_xyz'),
                'path_transform_yaw_deg': LaunchConfiguration('path_transform_yaw_deg'),
            }],
        ),
    ])
