from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('arm_path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('arm_original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('base_path_topic', default_value='/mir_path_transformed'),
        DeclareLaunchArgument('base_original_path_topic', default_value='/mir_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('use_current_arm_pose', default_value='true'),
        DeclareLaunchArgument('current_arm_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('robot_pose_type', default_value='pose_stamped'),
        DeclareLaunchArgument('wait_for_home_pose', default_value='false'),
        DeclareLaunchArgument('home_pose_ready_topic', default_value='/home_pose_ready'),
        DeclareLaunchArgument('arm_start_offset', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('arm_path_delta', default_value='[3.0, 3.0, 0.0]'),
        DeclareLaunchArgument('nozzle_axis', default_value='[0.0, 1.0, 0.0]'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        Node(
            package='parse_paths',
            executable='publish_front_side_arm_base_paths',
            name='front_side_arm_base_path_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('frame_id'),
                'arm_path_topic': LaunchConfiguration('arm_path_topic'),
                'arm_original_path_topic': LaunchConfiguration('arm_original_path_topic'),
                'base_path_topic': LaunchConfiguration('base_path_topic'),
                'base_original_path_topic': LaunchConfiguration('base_original_path_topic'),
                'normal_topic': LaunchConfiguration('normal_topic'),
                'use_current_arm_pose': LaunchConfiguration('use_current_arm_pose'),
                'current_arm_pose_topic': LaunchConfiguration('current_arm_pose_topic'),
                'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
                'robot_pose_type': LaunchConfiguration('robot_pose_type'),
                'wait_for_home_pose': LaunchConfiguration('wait_for_home_pose'),
                'home_pose_ready_topic': LaunchConfiguration('home_pose_ready_topic'),
                'arm_start_offset': LaunchConfiguration('arm_start_offset'),
                'arm_path_delta': LaunchConfiguration('arm_path_delta'),
                'nozzle_axis': LaunchConfiguration('nozzle_axis'),
                'num_points': LaunchConfiguration('num_points'),
                'time_step': LaunchConfiguration('time_step'),
            }],
        ),
    ])
