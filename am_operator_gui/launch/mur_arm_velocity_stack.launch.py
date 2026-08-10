"""Bridge AM twists into the shared AM J-PARSE controller for a MuR arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='mur620a'),
        DeclareLaunchArgument('arm', default_value='l'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('arm_base_link', default_value='mur620a/UR10_l/base_link'),
        DeclareLaunchArgument('controller_frame', default_value='UR10_l/base_link'),
        DeclareLaunchArgument('source_twist_topic', default_value='/mur620a/arm_following/twist_world'),
        DeclareLaunchArgument('controller_twist_topic', default_value='/mur620a/jparse_velocity_controller_l/twist_cmd'),
        DeclareLaunchArgument('velocity_command_topic', default_value='/mur620a/forward_velocity_controller_l/commands'),
        DeclareLaunchArgument('tip_link', default_value='mur620a/UR10_l/tool0'),
        DeclareLaunchArgument('robot_description_topic', default_value='/mur620a/robot_description'),
        DeclareLaunchArgument('joint_states_topic', default_value='/mur620a/joint_states'),
        DeclareLaunchArgument('fixed_tool_offset_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument(
            'fixed_tool_offset_quaternion_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'),
        DeclareLaunchArgument('spray_distance_topic', default_value='/spray_distance_smoothed'),
        DeclareLaunchArgument('jparse_readiness_topic', default_value='/am/jparse_ready'),
        Node(
            package='ur_trajectory_follower', executable='transform_twist_stamped',
            name='mur_transform_twist_to_arm_base', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('source_twist_topic'),
                'output_topic': LaunchConfiguration('controller_twist_topic'),
                'target_frame': LaunchConfiguration('arm_base_link'),
                'output_frame': LaunchConfiguration('controller_frame'),
                'fallback_source_frame': LaunchConfiguration('path_frame'),
            }],
        ),
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
                'base_link': LaunchConfiguration('arm_base_link'),
                'tip_link': LaunchConfiguration('tip_link'),
                'fixed_tool_offset_xyz': LaunchConfiguration('fixed_tool_offset_xyz'),
                'fixed_tool_offset_quaternion_xyzw':
                    LaunchConfiguration('fixed_tool_offset_quaternion_xyzw'),
                'spray_distance_topic': LaunchConfiguration('spray_distance_topic'),
                'robot_description_topic': LaunchConfiguration('robot_description_topic'),
                'twist_topic': LaunchConfiguration('controller_twist_topic'),
                'command_topic': LaunchConfiguration('velocity_command_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'readiness_topic': LaunchConfiguration('jparse_readiness_topic'),
            }.items(),
        ),
        Node(
            package='am_operator_gui', executable='mur_arm_readiness',
            name='mur_arm_readiness', output='screen',
            parameters=[{
                'jparse_twist_topic': LaunchConfiguration('controller_twist_topic'),
                'velocity_command_topic': LaunchConfiguration('velocity_command_topic'),
                'jparse_ready_topic': LaunchConfiguration('jparse_readiness_topic'),
                'controller_ready_topic': '/am/arm_controller_ready',
            }],
        ),
    ])
