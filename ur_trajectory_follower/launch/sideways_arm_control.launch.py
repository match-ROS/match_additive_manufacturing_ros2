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
            executable='current_pose_from_tf',
            name='current_pose_from_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_current_pose_from_tf')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': LaunchConfiguration('base_link'),
                'source_frame': LaunchConfiguration('tip_link'),
                'pose_topic': current_pose_topic,
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
            package='ur_trajectory_follower',
            executable='publish_sideways_test_path',
            name='sideways_ur_test_path',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': LaunchConfiguration('base_link'),
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
            parameters=[{
                'use_sim_time': use_sim_time,
                'path_index_topic': path_index_topic,
                'next_goal_topic': LaunchConfiguration('next_goal_topic'),
                'normal_topic': normal_topic,
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'path_topic': path_topic,
                'publish_rate': LaunchConfiguration('path_index_rate'),
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
                'nozzle_height_default': LaunchConfiguration('nozzle_height_default'),
                'kp_z': LaunchConfiguration('kp_z'),
                'ki_z': LaunchConfiguration('ki_z'),
                'kd_z': LaunchConfiguration('kd_z'),
                'from_index_offset': LaunchConfiguration('from_index_offset'),
                'goal_index_offset': LaunchConfiguration('goal_index_offset'),
                'spray_axis_source': LaunchConfiguration('spray_axis_source'),
                'spray_axis_sign': LaunchConfiguration('spray_axis_sign'),
                'start_condition_topic': start_condition_topic,
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'joint_state_topic': LaunchConfiguration('joint_states_topic'),
                'path_index_topic': path_index_topic,
                'lift_joint_name': LaunchConfiguration('lift_joint_name'),
            }],
            remappings=[
                ('path', path_topic),
                ('current_pose', current_pose_topic),
                ('velocity_override', LaunchConfiguration('velocity_override_topic')),
                ('nozzle_height_override', LaunchConfiguration('nozzle_height_override_topic')),
                ('ur_twist_world', LaunchConfiguration('ur_error_topic')),
            ],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='pid_twist_controller',
            name='pid_twist_controller_direction',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('ur_trajectory_follower'), 'config', 'pid_twist_controller.yaml']),
                {
                    'use_sim_time': use_sim_time,
                    'input_twist_topic': LaunchConfiguration('ur_error_topic'),
                    'output_twist_topic': LaunchConfiguration('ur_twist_world_topic'),
                },
            ],
        ),
        Node(
            package='ur_trajectory_follower',
            executable='ur_orientation_controller',
            name='ur_orientation_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kp_orientation': LaunchConfiguration('kp_orientation'),
                'ki_orientation': LaunchConfiguration('ki_orientation'),
                'kd_orientation': LaunchConfiguration('kd_orientation'),
                'path_topic': path_topic,
                'current_pose_topic': current_pose_topic,
                'path_index_topic': path_index_topic,
                'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
                'twist_topic': LaunchConfiguration('orientation_twist_topic'),
                'initial_path_index': LaunchConfiguration('initial_path_index'),
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
                'combined_twist_topic': LaunchConfiguration('combined_twist_topic'),
                'output_stamped': True,
                'frame_id': LaunchConfiguration('base_link'),
                'publish_rate_hz': LaunchConfiguration('combined_twist_rate'),
            }],
        ),
    ]

    if LaunchConfiguration('start_jparse_controller').perform(context).lower() == 'true':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('controllers_ros2'),
                        'launch',
                        'bunkur_ur_velocity_controller.launch.py',
                    ])
                ),
                launch_arguments={
                    'sim': LaunchConfiguration('use_sim_time'),
                    'robot_name': LaunchConfiguration('robot_name'),
                    'arm': LaunchConfiguration('arm'),
                    'base_link': LaunchConfiguration('base_link'),
                    'tip_link': LaunchConfiguration('tip_link'),
                    'robot_description_topic': LaunchConfiguration('robot_description_topic'),
                    'twist_topic': LaunchConfiguration('combined_twist_topic'),
                    'command_topic': LaunchConfiguration('velocity_command_topic'),
                    'joint_states_topic': LaunchConfiguration('joint_states_topic'),
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
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('velocity_command_topic', default_value='/ur_forward_velocity_controller/commands'),
        DeclareLaunchArgument('start_jparse_controller', default_value='true'),
        DeclareLaunchArgument('publish_current_pose_from_tf', default_value='true'),
        DeclareLaunchArgument('current_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('pose_publish_rate', default_value='50.0'),
        DeclareLaunchArgument('path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('original_path_topic', default_value='/ur_path_original'),
        DeclareLaunchArgument('normal_topic', default_value='/normal_vector'),
        DeclareLaunchArgument('next_goal_topic', default_value='/next_goal'),
        DeclareLaunchArgument('path_index_topic', default_value='/path_index'),
        DeclareLaunchArgument('path_index_rate', default_value='10.0'),
        DeclareLaunchArgument('initial_path_index', default_value='0'),
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
        DeclareLaunchArgument('from_index_offset', default_value='-1'),
        DeclareLaunchArgument('goal_index_offset', default_value='0'),
        DeclareLaunchArgument('lift_joint_name', default_value='unused_lift_joint'),
        DeclareLaunchArgument('velocity_override_topic', default_value='/velocity_override'),
        DeclareLaunchArgument('nozzle_height_override_topic', default_value='/nozzle_height_override'),
        DeclareLaunchArgument('ur_error_topic', default_value='/ur_error_world'),
        DeclareLaunchArgument('ur_twist_world_topic', default_value='/ur_twist_world'),
        DeclareLaunchArgument('orientation_twist_topic', default_value='/ur_orientation_twist'),
        DeclareLaunchArgument('combined_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd'),
        DeclareLaunchArgument('combined_twist_rate', default_value='100.0'),
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
