from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
                    FindPackageShare('bunker_description'),
                    'launch',
                    'spawn_with_controllers.launch.py',
                ])
            ),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        )
    ]


def generate_launch_description():
    tcp_pose_publisher = Node(
        package='ur_trajectory_follower',
        executable='current_pose_from_tf',
        name='bunker_current_tcp_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': LaunchConfiguration('tcp_target_frame'),
            'source_frame': LaunchConfiguration('tcp_source_frame'),
            'pose_topic': LaunchConfiguration('tcp_pose_topic'),
            'publish_rate': LaunchConfiguration('tcp_pose_publish_rate'),
        }],
    )

    path_publisher = Node(
        package='parse_paths',
        executable='publish_front_side_arm_base_paths',
        name='front_side_arm_base_path_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': LaunchConfiguration('path_frame'),
            'arm_path_topic': LaunchConfiguration('arm_path_topic'),
            'arm_original_path_topic': LaunchConfiguration('arm_original_path_topic'),
            'base_path_topic': LaunchConfiguration('base_path_topic'),
            'base_original_path_topic': LaunchConfiguration('base_original_path_topic'),
            'normal_topic': LaunchConfiguration('normal_topic'),
            'use_current_arm_pose': LaunchConfiguration('use_current_arm_pose'),
            'current_arm_pose_topic': LaunchConfiguration('tcp_pose_topic'),
            'robot_pose_topic': LaunchConfiguration('robot_pose_topic'),
            'robot_pose_type': 'pose_stamped',
            'wait_for_home_pose': False,
            'arm_path_delta': LaunchConfiguration('arm_path_delta'),
            'num_points': LaunchConfiguration('num_points'),
            'time_step': LaunchConfiguration('time_step'),
        }],
    )

    nozzle_monitor = Node(
        package='print_path_monitoring',
        executable='nozzle_pose_monitor',
        name='nozzle_pose_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'tcp_pose_topic': LaunchConfiguration('tcp_pose_topic'),
            'reference_path_topic': LaunchConfiguration('arm_path_topic'),
            'path_index_topic': LaunchConfiguration('path_index_topic'),
            'fixed_path_index': LaunchConfiguration('fixed_path_index'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_sim', default_value='false'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('tcp_target_frame', default_value='map'),
        DeclareLaunchArgument('tcp_source_frame', default_value='ur_tool0'),
        DeclareLaunchArgument('tcp_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('tcp_pose_publish_rate', default_value='50.0'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('arm_path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('arm_original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('base_path_topic', default_value='/bunker_base_path'),
        DeclareLaunchArgument('base_original_path_topic', default_value='/bunker_base_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('robot_pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('use_current_arm_pose', default_value='true'),
        DeclareLaunchArgument('arm_path_delta', default_value='[3.0, 3.0, 0.0]'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        DeclareLaunchArgument('fixed_path_index', default_value='0'),
        OpaqueFunction(function=_optional_sim_launch),
        tcp_pose_publisher,
        path_publisher,
        nozzle_monitor,
    ])
