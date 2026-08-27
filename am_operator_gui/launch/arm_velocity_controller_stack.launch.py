from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('arm', default_value='arm'),
        DeclareLaunchArgument('base_link', default_value='robot_arm_base_link'),
        DeclareLaunchArgument('tip_link', default_value='robot_arm_tool0'),
        DeclareLaunchArgument('fixed_tool_offset_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('fixed_tool_offset_quaternion_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'),
        DeclareLaunchArgument('spray_distance_topic', default_value='/spray_distance_smoothed'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot/robot_description'),
        DeclareLaunchArgument('joint_states_topic', default_value='/robot/joint_states'),
        DeclareLaunchArgument('source_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd_world'),
        DeclareLaunchArgument('controller_twist_topic', default_value='/jparse_velocity_controller_ur/twist_cmd'),
        DeclareLaunchArgument(
            'velocity_command_topic',
            default_value='/robot/arm/forward_velocity_controller/commands',
        ),
        DeclareLaunchArgument('controller_manager', default_value='/robot/arm/controller_manager'),
        DeclareLaunchArgument('deactivate_controller', default_value='joint_trajectory_controller'),
        DeclareLaunchArgument('activate_controller', default_value='forward_velocity_controller'),
        DeclareLaunchArgument('jparse_readiness_topic', default_value='/am/jparse_ready'),
        DeclareLaunchArgument('controller_readiness_topic', default_value='/am/arm_controller_ready'),
        DeclareLaunchArgument('jparse_max_joint_velocity', default_value='1.5'),
        DeclareLaunchArgument('jparse_max_cartesian_linear_velocity', default_value='0.25'),
        DeclareLaunchArgument('jparse_max_cartesian_angular_velocity', default_value='0.8'),
        DeclareLaunchArgument(
            'command_joint_names_csv',
            default_value='robot_arm_shoulder_pan_joint,robot_arm_shoulder_lift_joint,'
                          'robot_arm_elbow_joint,robot_arm_wrist_1_joint,'
                          'robot_arm_wrist_2_joint,robot_arm_wrist_3_joint',
        ),
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
                'spray_distance_topic': LaunchConfiguration('spray_distance_topic'),
                'robot_description_topic': LaunchConfiguration('robot_description_topic'),
                'twist_topic': LaunchConfiguration('controller_twist_topic'),
                'command_topic': LaunchConfiguration('velocity_command_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'readiness_topic': LaunchConfiguration('jparse_readiness_topic'),
                'command_joint_names_csv': LaunchConfiguration('command_joint_names_csv'),
                'max_joint_velocity': LaunchConfiguration('jparse_max_joint_velocity'),
                'max_cartesian_linear_velocity': LaunchConfiguration(
                    'jparse_max_cartesian_linear_velocity'),
                'max_cartesian_angular_velocity': LaunchConfiguration(
                    'jparse_max_cartesian_angular_velocity'),
            }.items(),
        ),
        Node(
            package='am_operator_gui',
            executable='controller_switch_guard',
            name='operator_arm_controller_guard',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'controller_manager': LaunchConfiguration('controller_manager'),
                'activate_controller': LaunchConfiguration('activate_controller'),
                'deactivate_controller': LaunchConfiguration('deactivate_controller'),
                'ready_topic': LaunchConfiguration('controller_readiness_topic'),
                'switch_on_start': True,
            }],
        ),
    ])
