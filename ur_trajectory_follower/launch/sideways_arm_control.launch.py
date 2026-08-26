from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    joint_prefix = LaunchConfiguration('joint_prefix').perform(context)
    joint_names = [
        f'{joint_prefix}shoulder_pan_joint',
        f'{joint_prefix}shoulder_lift_joint',
        f'{joint_prefix}elbow_joint',
        f'{joint_prefix}wrist_1_joint',
        f'{joint_prefix}wrist_2_joint',
        f'{joint_prefix}wrist_3_joint',
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    path_topic = LaunchConfiguration('path_topic')
    current_pose_topic = LaunchConfiguration('current_pose_topic')
    path_index_topic = LaunchConfiguration('path_index_topic')
    normal_topic = LaunchConfiguration('normal_topic')
    start_condition_topic = LaunchConfiguration('start_condition_topic')
    combined_twist_source_topic = LaunchConfiguration('combined_twist_source_topic')
    twist_topics = (
        '['
        + LaunchConfiguration('ur_twist_world_topic').perform(context)
        + ', '
        + LaunchConfiguration('orientation_twist_topic').perform(context)
        + ']'
    )

    nodes = [
        Node(
            package='ur_trajectory_follower',
            executable='deposition_pose',
            name='deposition_pose',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'nozzle_pose_topic': LaunchConfiguration('nozzle_pose_topic'),
                'deposition_pose_topic': current_pose_topic,
                'spray_distance_topic': LaunchConfiguration('spray_distance_topic'),
                'smoothed_spray_distance_topic': LaunchConfiguration('smoothed_spray_distance_topic'),
                'spray_distance_initial': LaunchConfiguration('spray_distance_initial'),
                'spray_distance_max_rate': LaunchConfiguration('spray_distance_max_rate'),
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='current_pose_from_tf',
            name='current_pose_from_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_current_pose_from_tf')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': LaunchConfiguration('path_frame'),
                'source_frame': LaunchConfiguration('tip_link'),
                'pose_topic': LaunchConfiguration('nozzle_pose_topic'),
                'publish_rate': LaunchConfiguration('pose_publish_rate'),
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='publish_joint_pose_once',
            name='sideways_start_joint_pose',
            output='screen',
            condition=IfCondition(LaunchConfiguration('move_to_start_pose')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'trajectory_topic': LaunchConfiguration('start_pose_trajectory_topic'),
                'joint_names': joint_names,
                'positions': LaunchConfiguration('start_joint_positions'),
                'time_from_start': LaunchConfiguration('start_pose_time_from_start'),
                'publish_delay': LaunchConfiguration('start_pose_publish_delay'),
            }],
        ),
        Node(
            package='parse_paths',
            executable='publish_sideways_arm_test_path',
            name='sideways_arm_test_path',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_path')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': LaunchConfiguration('path_frame'),
                'path_topic': path_topic,
                'original_path_topic': LaunchConfiguration('original_path_topic'),
                'normal_topic': normal_topic,
                'use_current_pose': LaunchConfiguration('use_current_pose'),
                'current_pose_topic': current_pose_topic,
                'wait_for_home_pose': LaunchConfiguration('wait_for_home_pose'),
                'home_pose_ready_topic': LaunchConfiguration('home_pose_ready_topic'),
                'start_offset': LaunchConfiguration('start_offset'),
                'direction': LaunchConfiguration('path_direction'),
                'nozzle_axis': LaunchConfiguration('nozzle_axis'),
                'path_length': LaunchConfiguration('path_length'),
                'num_points': LaunchConfiguration('num_points'),
                'time_step': LaunchConfiguration('time_step'),
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='increment_path_index',
            name='increment_path_index',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_path_index')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'path_index_topic': path_index_topic,
                'next_goal_topic': LaunchConfiguration('next_goal_topic'),
                'normal_topic': normal_topic,
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'path_topic': path_topic,
                'progress_mode': LaunchConfiguration('progress_mode'),
                'base_path_topic': LaunchConfiguration('base_path_topic'),
                'arm_reference_topic': LaunchConfiguration('arm_reference_topic'),
                'base_reference_topic': LaunchConfiguration('base_reference_topic'),
                'processed_path_topic': LaunchConfiguration('tracking_path_topic'),
                'processed_base_path_topic': LaunchConfiguration('tracking_base_path_topic'),
                'phase_topic': LaunchConfiguration('phase_topic'),
                'desired_speed_topic': LaunchConfiguration('desired_speed_topic'),
                'desired_arm_speed': LaunchConfiguration('default_velocity'),
                'enable_path_resampling': LaunchConfiguration('enable_path_resampling'),
                'resample_spacing': LaunchConfiguration('resample_spacing'),
                'publish_rate': LaunchConfiguration('path_index_rate'),
                'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
                'start_condition_topic': start_condition_topic,
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='ur_direction_controller',
            name='ur_direction_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kp_z': LaunchConfiguration('kp_z'),
                'orthogonal_kp': LaunchConfiguration('orthogonal_kp'),
                'orthogonal_max_velocity': LaunchConfiguration('orthogonal_max_velocity'),
                'along_track_kp': LaunchConfiguration('along_track_kp'),
                'max_along_track_correction': LaunchConfiguration('max_along_track_correction'),
                'max_spray_axis_correction': LaunchConfiguration('max_spray_axis_correction'),
                'max_tracking_linear_velocity': LaunchConfiguration('max_tracking_linear_velocity'),
                'final_position_tolerance': LaunchConfiguration('final_position_tolerance'),
                'spray_axis_source': LaunchConfiguration('spray_axis_source'),
                'spray_axis_sign': LaunchConfiguration('spray_axis_sign'),
                'start_condition_topic': start_condition_topic,
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'path_index_topic': path_index_topic,
                'path_topic': LaunchConfiguration('tracking_path_topic'),
                'default_velocity': LaunchConfiguration('default_velocity'),
                'reference_pose_topic': LaunchConfiguration('arm_reference_topic'),
                'current_pose_topic': current_pose_topic,
                'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
                'desired_speed_topic': LaunchConfiguration('desired_speed_topic'),
            }],
            remappings=[
                ('ur_twist_world', LaunchConfiguration('ur_twist_world_topic')),
            ],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='ur_orientation_controller',
            name='ur_orientation_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_orientation_controller')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'kp_orientation': LaunchConfiguration('kp_orientation'),
                'ki_orientation': LaunchConfiguration('ki_orientation'),
                'kd_orientation': LaunchConfiguration('kd_orientation'),
                'path_topic': LaunchConfiguration('tracking_path_topic'),
                'reference_pose_topic': LaunchConfiguration('arm_reference_topic'),
                'current_pose_topic': current_pose_topic,
                'path_index_topic': path_index_topic,
                'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
                'twist_topic': LaunchConfiguration('orientation_twist_topic'),
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'start_condition_topic': start_condition_topic,
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='combine_twists',
            name='twist_combiner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'twist_topics': twist_topics,
                'combined_twist_topic': combined_twist_source_topic,
                'output_stamped': True,
                'frame_id': LaunchConfiguration('path_frame'),
                'publish_rate_hz': LaunchConfiguration('combined_twist_rate'),
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                'start_condition_topic': start_condition_topic,
            }],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='transform_twist_stamped',
            name='transform_twist_to_command_frame',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_command_transform')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': combined_twist_source_topic,
                'output_topic': LaunchConfiguration('combined_twist_topic'),
                'target_frame': LaunchConfiguration('base_link'),
                'fallback_source_frame': LaunchConfiguration('path_frame'),
            }],
        ),
    ]

    if LaunchConfiguration('start_jparse_controller').perform(context).lower() == 'true':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('am_jparse_controller'),
                        'launch',
                        'am_jparse_velocity_controller.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'arm': LaunchConfiguration('arm'),
                    'base_link': LaunchConfiguration('base_link'),
                    'tip_link': LaunchConfiguration('tip_link'),
                    'fixed_tool_offset_xyz': LaunchConfiguration('fixed_tool_offset_xyz'),
                    'fixed_tool_offset_quaternion_xyzw': LaunchConfiguration('fixed_tool_offset_quaternion_xyzw'),
                    'spray_distance_topic': LaunchConfiguration('smoothed_spray_distance_topic'),
                    'robot_description_topic': LaunchConfiguration('robot_description_topic'),
                    'twist_topic': LaunchConfiguration('combined_twist_topic'),
                    'command_topic': LaunchConfiguration('velocity_command_topic'),
                    'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                    'readiness_topic': LaunchConfiguration('jparse_readiness_topic'),
                    'command_joint_names_csv': LaunchConfiguration('command_joint_names_csv'),
                    'max_cartesian_linear_velocity': LaunchConfiguration(
                        'jparse_max_cartesian_linear_velocity'),
                }.items(),
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='bunkur'),
        DeclareLaunchArgument('arm', default_value='ur'),
        DeclareLaunchArgument('joint_prefix', default_value='ur_'),
        DeclareLaunchArgument('base_link', default_value='ur_base_link'),
        DeclareLaunchArgument('tip_link', default_value='ur_tool0'),
        DeclareLaunchArgument('fixed_tool_offset_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('fixed_tool_offset_quaternion_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('velocity_command_topic', default_value='/ur_forward_velocity_controller/commands'),
        DeclareLaunchArgument('start_jparse_controller', default_value='true'),
        DeclareLaunchArgument(
            'jparse_max_cartesian_linear_velocity',
            default_value='0.8',
            description=(
                'Cartesian input limit for J-PARSE. Must exceed the maximum '
                'base-induced TCP compensation plus the tracking command.'
            ),
        ),
        DeclareLaunchArgument(
            'start_orientation_controller',
            default_value='true',
            description='Enable the orientation channel of the Cartesian arm controller.',
        ),
        DeclareLaunchArgument('start_command_transform', default_value='true'),
        DeclareLaunchArgument('publish_current_pose_from_tf', default_value='true'),
        DeclareLaunchArgument('nozzle_pose_topic', default_value='/current_nozzle_tip_pose'),
        DeclareLaunchArgument('current_pose_topic', default_value='/current_deposition_pose'),
        DeclareLaunchArgument('spray_distance_topic', default_value='/spray_distance'),
        DeclareLaunchArgument('smoothed_spray_distance_topic', default_value='/spray_distance_smoothed'),
        DeclareLaunchArgument('spray_distance_initial', default_value='0.0'),
        DeclareLaunchArgument('spray_distance_max_rate', default_value='0.02'),
        DeclareLaunchArgument('pose_publish_rate', default_value='50.0'),
        DeclareLaunchArgument('publish_path', default_value='true'),
        DeclareLaunchArgument('path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('base_path_topic', default_value=''),
        DeclareLaunchArgument('progress_mode', default_value='timestamp'),
        DeclareLaunchArgument('arm_reference_topic', default_value='/arm_trajectory_reference'),
        DeclareLaunchArgument('base_reference_topic', default_value='/base_trajectory_reference'),
        DeclareLaunchArgument('tracking_path_topic', default_value='/ur_path_tracking'),
        DeclareLaunchArgument('tracking_base_path_topic', default_value='/base_path_tracking'),
        DeclareLaunchArgument('phase_topic', default_value='/trajectory_phase'),
        DeclareLaunchArgument('desired_speed_topic', default_value='/desired_arm_speed'),
        DeclareLaunchArgument('enable_path_resampling', default_value='true'),
        DeclareLaunchArgument('resample_spacing', default_value='0.005'),
        DeclareLaunchArgument('original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('next_goal_topic', default_value='/next_goal'),
        DeclareLaunchArgument('publish_path_index', default_value='true'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        DeclareLaunchArgument('path_index_rate', default_value='10.0'),
        DeclareLaunchArgument('initial_path_index', default_value='0'),
        DeclareLaunchArgument('default_velocity', default_value='-1.0'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_condition'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='false'),
        DeclareLaunchArgument('use_current_pose', default_value='true'),
        DeclareLaunchArgument('wait_for_home_pose', default_value='false'),
        DeclareLaunchArgument('home_pose_ready_topic', default_value='/home_pose_ready'),
        DeclareLaunchArgument('start_offset', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('path_direction', default_value='[-1.0, 0.0, 0.0]'),
        DeclareLaunchArgument('nozzle_axis', default_value='[0.0, 1.0, 0.0]'),
        DeclareLaunchArgument('path_length', default_value='1.2'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        DeclareLaunchArgument('spray_axis_source', default_value='tool_z'),
        DeclareLaunchArgument('spray_axis_sign', default_value='1.0'),
        DeclareLaunchArgument('nozzle_height_default', default_value='0.0'),
        DeclareLaunchArgument('kp_z', default_value='0.7'),
        DeclareLaunchArgument('ki_z', default_value='0.0'),
        DeclareLaunchArgument('kd_z', default_value='0.0'),
        DeclareLaunchArgument('along_track_kp', default_value='2.0'),
        DeclareLaunchArgument('max_along_track_correction', default_value='0.03'),
        DeclareLaunchArgument('max_spray_axis_correction', default_value='0.03'),
        DeclareLaunchArgument('max_tracking_linear_velocity', default_value='0.12'),
        DeclareLaunchArgument('final_position_tolerance', default_value='0.005'),
        DeclareLaunchArgument('from_index_offset', default_value='-1'),
        DeclareLaunchArgument('goal_index_offset', default_value='0'),
        DeclareLaunchArgument('direction_control_mode', default_value='speed_orthogonal'),
        DeclareLaunchArgument('orthogonal_kp', default_value='1.0'),
        DeclareLaunchArgument('orthogonal_max_velocity', default_value='0.02'),
        DeclareLaunchArgument('lift_joint_name', default_value='unused_lift_joint'),
        DeclareLaunchArgument('velocity_override_topic', default_value='/velocity_override'),
        DeclareLaunchArgument('nozzle_height_override_topic', default_value='/nozzle_height_override'),
        DeclareLaunchArgument('ur_error_topic', default_value='/ur_error_world'),
        DeclareLaunchArgument('ur_twist_world_topic', default_value='/ur_twist_world'),
        DeclareLaunchArgument('orientation_twist_topic', default_value='/ur_orientation_twist'),
        DeclareLaunchArgument('combined_twist_source_topic', default_value='/jparse_velocity_controller_ur/twist_cmd_world'),
        DeclareLaunchArgument('combined_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd'),
        DeclareLaunchArgument('combined_twist_rate', default_value='100.0'),
        DeclareLaunchArgument('jparse_readiness_topic', default_value='/am/jparse_ready'),
        DeclareLaunchArgument('command_joint_names_csv', default_value=''),
        DeclareLaunchArgument('kp_orientation', default_value='1.0'),
        DeclareLaunchArgument('ki_orientation', default_value='0.0'),
        DeclareLaunchArgument('kd_orientation', default_value='0.0'),
        DeclareLaunchArgument('move_to_start_pose', default_value='false'),
        DeclareLaunchArgument('start_pose_trajectory_topic', default_value='/ur_joint_trajectory_controller/joint_trajectory'),
        DeclareLaunchArgument('start_joint_positions', default_value='[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]'),
        DeclareLaunchArgument('start_pose_time_from_start', default_value='4.0'),
        DeclareLaunchArgument('start_pose_publish_delay', default_value='3.0'),
        OpaqueFunction(function=_launch_setup),
    ])
