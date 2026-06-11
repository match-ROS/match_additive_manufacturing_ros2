from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _optional_sim_launch(context, *args, **kwargs):
    if LaunchConfiguration('launch_sim').perform(context).strip().lower() not in {'1', 'true', 'yes', 'on'}:
        return []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rbvogui_ur_sim_setup'),
                    'launch',
                    'rbvogui_ur_standard_control.launch.py',
                ])
            ),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'robot_id': LaunchConfiguration('robot_id'),
            }.items(),
        )
    ]


def generate_launch_description():
    path_publisher = Node(
        package='parse_paths',
        executable='publish_robotnik_base_arm_paths',
        name='robotnik_base_arm_path_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('generate_test_paths')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': LaunchConfiguration('path_frame'),
            'base_path_topic': LaunchConfiguration('base_path_topic'),
            'base_original_path_topic': LaunchConfiguration('base_original_path_topic'),
            'arm_path_topic': LaunchConfiguration('arm_path_topic'),
            'arm_original_path_topic': LaunchConfiguration('arm_original_path_topic'),
            'normal_topic': LaunchConfiguration('normal_topic'),
            'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
            'current_arm_pose_topic': LaunchConfiguration('current_arm_pose_topic'),
            'use_current_poses': LaunchConfiguration('use_current_poses'),
            'base_start_offset': LaunchConfiguration('base_start_offset'),
            'sideways_distance': LaunchConfiguration('sideways_distance'),
            'diagonal_distance': LaunchConfiguration('diagonal_distance'),
            'arm_xy_offset': LaunchConfiguration('arm_xy_offset'),
            'ramp_arm_xy_offset': LaunchConfiguration('ramp_arm_xy_offset'),
            'arm_height_delta': LaunchConfiguration('arm_height_delta'),
            'min_reachable_radius': LaunchConfiguration('min_reachable_radius'),
            'max_reachable_radius': LaunchConfiguration('max_reachable_radius'),
            'num_points': LaunchConfiguration('num_points'),
            'time_step': LaunchConfiguration('time_step'),
            'publish_once': LaunchConfiguration('publish_once'),
            'wait_for_trigger': False,
        }],
    )

    move_to_start = Node(
        package='move_to_path_idx',
        executable='move_to_path_idx',
        name='move_to_base_path_start',
        output='screen',
        condition=IfCondition(LaunchConfiguration('move_to_start_pose')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_topic': LaunchConfiguration('base_path_topic'),
            'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
            'robot_pose_type': 'pose_stamped',
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'path_index': 0,
            'publish_start_condition': LaunchConfiguration('publish_start_pose_reached'),
            'start_condition_topic': LaunchConfiguration('start_pose_reached_topic'),
            'distance_tolerance': LaunchConfiguration('start_distance_tolerance'),
            'yaw_tolerance': LaunchConfiguration('start_yaw_tolerance'),
            'max_linear_velocity': LaunchConfiguration('start_max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('start_max_angular_velocity'),
        }],
    )

    path_index = Node(
        package='ur_trajectory_follower',
        executable='increment_path_index',
        name='shared_path_index',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_index_topic': LaunchConfiguration('path_index_topic'),
            'next_goal_topic': LaunchConfiguration('base_next_goal_topic'),
            'normal_topic': LaunchConfiguration('normal_topic'),
            'initial_path_index': LaunchConfiguration('initial_path_index'),
            'path_topic': LaunchConfiguration('base_path_topic'),
            'publish_rate': LaunchConfiguration('path_index_rate'),
            'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            'start_condition_topic': LaunchConfiguration('start_condition_topic'),
        }],
    )

    base_follower = Node(
        package='base_trajectory_follower',
        executable='simple_base_follower',
        name='simple_base_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_topic': LaunchConfiguration('base_path_topic'),
            'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
            'robot_pose_type': 'pose_stamped',
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'output_stamped': LaunchConfiguration('output_stamped'),
            'use_external_path_index': True,
            'path_index_topic': LaunchConfiguration('path_index_topic'),
            'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            'start_condition_topic': LaunchConfiguration('start_condition_topic'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_wz': LaunchConfiguration('max_wz'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_sim', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('path_frame', default_value='robotnik_simple'),
        DeclareLaunchArgument('base_path_topic', default_value='/base_path'),
        DeclareLaunchArgument('base_original_path_topic', default_value='/base_path_original'),
        DeclareLaunchArgument('arm_path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('arm_original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('current_arm_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('use_current_poses', default_value='true'),
        DeclareLaunchArgument('base_start_offset', default_value='[0.35, 0.0, 0.0]'),
        DeclareLaunchArgument('sideways_distance', default_value='0.8'),
        DeclareLaunchArgument('diagonal_distance', default_value='0.8'),
        DeclareLaunchArgument('arm_xy_offset', default_value='[0.15, 0.0, 0.0]'),
        DeclareLaunchArgument('ramp_arm_xy_offset', default_value='true'),
        DeclareLaunchArgument('arm_height_delta', default_value='0.2'),
        DeclareLaunchArgument('min_reachable_radius', default_value='0.25'),
        DeclareLaunchArgument('max_reachable_radius', default_value='0.85'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        DeclareLaunchArgument('generate_test_paths', default_value='true'),
        DeclareLaunchArgument('publish_once', default_value='true'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        DeclareLaunchArgument('base_next_goal_topic', default_value='/base_next_goal'),
        DeclareLaunchArgument('initial_path_index', default_value='0'),
        DeclareLaunchArgument('path_index_rate', default_value='5.0'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='true'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_condition'),
        DeclareLaunchArgument('start_pose_reached_topic', default_value='/start_pose_reached'),
        DeclareLaunchArgument('publish_start_pose_reached', default_value='true'),
        DeclareLaunchArgument('move_to_start_pose', default_value='true'),
        DeclareLaunchArgument('start_distance_tolerance', default_value='0.06'),
        DeclareLaunchArgument('start_yaw_tolerance', default_value='0.08'),
        DeclareLaunchArgument('start_max_linear_velocity', default_value='0.2'),
        DeclareLaunchArgument('start_max_angular_velocity', default_value='0.5'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/robot/robotnik_base_control/cmd_vel_unstamped'),
        DeclareLaunchArgument('output_stamped', default_value='false'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.3'),
        DeclareLaunchArgument('max_vx', default_value='0.25'),
        DeclareLaunchArgument('max_vy', default_value='0.25'),
        DeclareLaunchArgument('max_wz', default_value='0.5'),
        OpaqueFunction(function=_optional_sim_launch),
        path_publisher,
        move_to_start,
        path_index,
        base_follower,
    ])
