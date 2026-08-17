"""IK-first, deposition-pose-corrected start move for a MuR UR arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('current_pose_topic', default_value='/current_deposition_pose'),
        DeclareLaunchArgument('path_index', default_value='0'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='true'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_pose_reached'),
        DeclareLaunchArgument('ready_topic', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/mur620a/arm_following/twist_world'),
        DeclareLaunchArgument('arm', default_value='r'),
        DeclareLaunchArgument('tf_arm_base_frame', default_value='mur620a/base_footprint'),
        DeclareLaunchArgument('ik_pose_frame', default_value='base_footprint'),
        DeclareLaunchArgument('joint_states_topic', default_value='/mur620a/joint_states'),
        DeclareLaunchArgument('trajectory_topic', default_value='/mur620a/joint_trajectory_controller_r/joint_trajectory'),
        DeclareLaunchArgument('controller_manager', default_value='/mur620a/controller_manager'),
        DeclareLaunchArgument('joint_trajectory_controller', default_value='joint_trajectory_controller_r'),
        DeclareLaunchArgument('velocity_controller', default_value='forward_velocity_controller_r'),
        DeclareLaunchArgument('fixed_tool_offset_xyz', default_value='[0.0, 0.0, 0.25]'),
        DeclareLaunchArgument('fixed_tool_offset_quaternion_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'),
        DeclareLaunchArgument('spray_distance', default_value='0.1'),
        DeclareLaunchArgument('ik_configuration', default_value='shoulder_right_elbow_up_wrist_unflip'),
        DeclareLaunchArgument('ik_seed_positions', default_value='[-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]'),
        DeclareLaunchArgument('trajectory_duration', default_value='8.0'),
        DeclareLaunchArgument('settle_time', default_value='1.0'),
        DeclareLaunchArgument('distance_tolerance', default_value='0.03'),
        DeclareLaunchArgument('orientation_tolerance', default_value='0.06'),
        DeclareLaunchArgument('kp_linear', default_value='0.8'),
        DeclareLaunchArgument('kp_angular', default_value='1.0'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.12'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.5'),
    ]
    correction_ready = '/am/ik_correction_ready'
    ik_node = Node(
        package='move_to_path_idx', executable='move_ur_ik_to_path_idx',
        name='move_ur_ik_to_path_idx', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_topic': LaunchConfiguration('path_topic'), 'path_index': LaunchConfiguration('path_index'),
            'path_frame': LaunchConfiguration('path_frame'),
            'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            'start_condition_topic': LaunchConfiguration('start_condition_topic'),
            'correction_ready_topic': correction_ready,
            'ik_service': '/mur620a/compute_ik', 'ik_group_name': 'UR_arm_r', 'ik_link_name': 'UR10_r/tool0',
            'tf_arm_base_frame': LaunchConfiguration('tf_arm_base_frame'),
            'ik_pose_frame': LaunchConfiguration('ik_pose_frame'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'trajectory_topic': LaunchConfiguration('trajectory_topic'),
            'controller_manager': LaunchConfiguration('controller_manager'),
            'joint_trajectory_controller': LaunchConfiguration('joint_trajectory_controller'),
            'velocity_controller': LaunchConfiguration('velocity_controller'),
            'fixed_tool_offset_xyz': LaunchConfiguration('fixed_tool_offset_xyz'),
            'fixed_tool_offset_quaternion_xyzw': LaunchConfiguration('fixed_tool_offset_quaternion_xyzw'),
            'spray_distance': LaunchConfiguration('spray_distance'),
            'ik_configuration': LaunchConfiguration('ik_configuration'),
            'ik_seed_positions': LaunchConfiguration('ik_seed_positions'),
            'trajectory_duration': LaunchConfiguration('trajectory_duration'),
            'settle_time': LaunchConfiguration('settle_time'),
        }],
    )
    correction_node = Node(
        package='move_to_path_idx', executable='move_ur_to_path_idx',
        name='move_ur_to_path_idx_correction', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_topic': LaunchConfiguration('path_topic'), 'current_pose_topic': LaunchConfiguration('current_pose_topic'),
            'path_index': LaunchConfiguration('path_index'), 'path_frame': LaunchConfiguration('path_frame'),
            'wait_for_start_condition': True, 'start_condition_topic': correction_ready,
            'ready_topic': LaunchConfiguration('ready_topic'), 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'distance_tolerance': LaunchConfiguration('distance_tolerance'),
            'orientation_tolerance': LaunchConfiguration('orientation_tolerance'),
            'kp_linear': LaunchConfiguration('kp_linear'), 'kp_angular': LaunchConfiguration('kp_angular'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }],
    )
    return LaunchDescription(args + [
        # MoveIt provides the UR IK service.  It is used for solving only;
        # the trajectory below goes to the existing robot controller.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mur_moveit_config'), 'launch', 'ur_moveit.launch.py',
            ])),
            launch_arguments={
                'ur_type': 'ur10', 'controller_namespace': 'mur620a',
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'), 'launch_rviz': 'false',
            }.items(),
        ),
        ik_node,
        correction_node,
        RegisterEventHandler(OnProcessExit(
            target_action=correction_node,
            on_exit=[EmitEvent(event=Shutdown(reason='IK start-pose correction complete'))],
        )),
    ])
