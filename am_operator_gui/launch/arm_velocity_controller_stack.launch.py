from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('arm', default_value='arm'),
        DeclareLaunchArgument('base_link', default_value='robot_arm_base_link'),
        DeclareLaunchArgument('tip_link', default_value='robot_arm_tool0'),
        DeclareLaunchArgument('path_frame', default_value='robotnik_simple'),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot/robot_description'),
        DeclareLaunchArgument('joint_states_topic', default_value='/robot/joint_states'),
        DeclareLaunchArgument('source_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd_world'),
        DeclareLaunchArgument('controller_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd'),
        DeclareLaunchArgument('velocity_command_topic', default_value='/robot/arm_forward_velocity_controller/commands'),
        DeclareLaunchArgument('controller_manager', default_value='/robot/controller_manager'),
        DeclareLaunchArgument('deactivate_controller', default_value='joint_trajectory_controller'),
        DeclareLaunchArgument('activate_controller', default_value='arm_forward_velocity_controller'),
        DeclareLaunchArgument('switch_delay', default_value='13.0'),
        Node(
            package='ur_trajectory_follower',
            executable='transform_twist_stamped',
            name='operator_transform_twist_to_command_frame',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('source_twist_topic'),
                'output_topic': LaunchConfiguration('controller_twist_topic'),
                'target_frame': LaunchConfiguration('base_link'),
                'fallback_source_frame': LaunchConfiguration('path_frame'),
            }],
        ),
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
                'twist_topic': LaunchConfiguration('controller_twist_topic'),
                'command_topic': LaunchConfiguration('velocity_command_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            }.items(),
        ),
        TimerAction(
            period=LaunchConfiguration('switch_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'control',
                        'switch_controllers',
                        '--controller-manager',
                        LaunchConfiguration('controller_manager'),
                        '--deactivate',
                        LaunchConfiguration('deactivate_controller'),
                        '--activate',
                        LaunchConfiguration('activate_controller'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
