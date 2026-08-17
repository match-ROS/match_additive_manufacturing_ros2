from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compute_path_topics(context):
    path_ns = LaunchConfiguration('path_ns').perform(context).strip('/')
    prefix = f'/{path_ns}' if path_ns else ''

    path_topic = LaunchConfiguration('path_topic').perform(context)
    path_index_topic = LaunchConfiguration('path_index_topic').perform(context)

    if not path_topic:
        path_topic = f'{prefix}/ur_path_transformed'
    if not path_index_topic:
        path_index_topic = f'{prefix}/path_index'

    return path_topic, path_index_topic


def _launch_setup(context, *args, **kwargs):
    path_topic, path_index_topic = _compute_path_topics(context)

    node = Node(
        package='ur_trajectory_follower',
        executable='ur_direction_controller',
        name='ur_direction_controller',
        output='screen',
        parameters=[
            {
                'nozzle_height_default': LaunchConfiguration('nozzle_height_default'),
                'kp_z': LaunchConfiguration('kp_z'),
                'ki_z': LaunchConfiguration('ki_z'),
                'kd_z': LaunchConfiguration('kd_z'),
                'from_index_offset': LaunchConfiguration('from_index_offset'),
                'goal_index_offset': LaunchConfiguration('goal_index_offset'),
                'control_mode': LaunchConfiguration('control_mode'),
                'orthogonal_kp': LaunchConfiguration('orthogonal_kp'),
                'orthogonal_max_velocity': LaunchConfiguration('orthogonal_max_velocity'),
                'spray_axis_source': LaunchConfiguration('spray_axis_source'),
                'spray_axis_sign': LaunchConfiguration('spray_axis_sign'),
                'start_condition_topic': LaunchConfiguration('start_condition_topic'),
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                'initial_path_index': LaunchConfiguration('initial_path_index'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'path_topic': path_topic,
                'path_index_topic': path_index_topic,
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
                'velocity_override_topic': LaunchConfiguration('velocity_override_topic'),
            }
        ],
        remappings=[
            ('nozzle_height_override', LaunchConfiguration('nozzle_height_override_topic')),
            ('ur_twist_world', LaunchConfiguration('twist_topic')),
            ('ur_twist_world_feedforward', LaunchConfiguration('feedforward_twist_topic')),
            ('ur_twist_world_control', LaunchConfiguration('control_twist_topic')),
        ],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('nozzle_height_default', default_value='0.1'),
            DeclareLaunchArgument('kp_z', default_value='0.7'),
            DeclareLaunchArgument('ki_z', default_value='0.0'),
            DeclareLaunchArgument('kd_z', default_value='0.0'),
            DeclareLaunchArgument(
                'path_ns',
                default_value='',
                description='Namespace prefix for UR path topics (with or without leading slash).',
            ),
            DeclareLaunchArgument('path_topic', default_value=''),
            DeclareLaunchArgument('path_index_topic', default_value=''),
            DeclareLaunchArgument('current_pose_topic', default_value='/global_nozzle_pose'),
            DeclareLaunchArgument('velocity_override_topic', default_value='/velocity_override'),
            DeclareLaunchArgument('nozzle_height_override_topic', default_value='/nozzle_height_override'),
            DeclareLaunchArgument('joint_state_topic', default_value='/mur620c/joint_states'),
            DeclareLaunchArgument('twist_topic', default_value='/ur_error_world'),
            DeclareLaunchArgument('feedforward_twist_topic', default_value='/ur_twist_world_feedforward'),
            DeclareLaunchArgument('control_twist_topic', default_value='/ur_twist_world_control'),
            DeclareLaunchArgument('from_index_offset', default_value='-1'),
            DeclareLaunchArgument('goal_index_offset', default_value='0'),
            DeclareLaunchArgument('control_mode', default_value='goal_direction'),
            DeclareLaunchArgument('orthogonal_kp', default_value='1.0'),
            DeclareLaunchArgument('orthogonal_max_velocity', default_value='0.1'),
            DeclareLaunchArgument('spray_axis_source', default_value='tool_z'),
            DeclareLaunchArgument('spray_axis_sign', default_value='1.0'),
            DeclareLaunchArgument('start_condition_topic', default_value='/start_condition'),
            DeclareLaunchArgument('wait_for_start_condition', default_value='true'),
            DeclareLaunchArgument('initial_path_index', default_value='-1'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
