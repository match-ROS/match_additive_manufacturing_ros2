"""Bridge AM world-frame twists into the verified native MuR arm controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        Node(
            package='am_operator_gui', executable='mur_arm_readiness',
            name='mur_arm_readiness', output='screen',
            parameters=[{
                'jparse_twist_topic': LaunchConfiguration('controller_twist_topic'),
                'velocity_command_topic': LaunchConfiguration('velocity_command_topic'),
                'jparse_ready_topic': '/am/jparse_ready',
                'controller_ready_topic': '/am/arm_controller_ready',
            }],
        ),
    ])
