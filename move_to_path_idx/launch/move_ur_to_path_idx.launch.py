from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('path_topic', default_value='/ur_path_transformed'),
        DeclareLaunchArgument('current_pose_topic', default_value='/current_tcp_pose'),
        DeclareLaunchArgument('path_index', default_value='0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('distance_tolerance', default_value='0.03'),
        DeclareLaunchArgument('orientation_tolerance', default_value='0.06'),
        DeclareLaunchArgument('kp_linear', default_value='0.8'),
        DeclareLaunchArgument('kp_angular', default_value='1.0'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.12'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.5'),
        DeclareLaunchArgument('publish_stop_count', default_value='3'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='true'),
        DeclareLaunchArgument('start_condition_topic', default_value='/start_pose_reached'),
        DeclareLaunchArgument('ready_topic', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/jparse_velocity_controller_ur/twist_cmd_world'),
        DeclareLaunchArgument('path_frame', default_value='map'),
        Node(
            package='move_to_path_idx',
            executable='move_ur_to_path_idx',
            name='move_ur_to_path_idx',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': LaunchConfiguration('path_topic'),
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
                'path_index': LaunchConfiguration('path_index'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'distance_tolerance': LaunchConfiguration('distance_tolerance'),
                'orientation_tolerance': LaunchConfiguration('orientation_tolerance'),
                'kp_linear': LaunchConfiguration('kp_linear'),
                'kp_angular': LaunchConfiguration('kp_angular'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'publish_stop_count': LaunchConfiguration('publish_stop_count'),
                'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
                'start_condition_topic': LaunchConfiguration('start_condition_topic'),
                'ready_topic': LaunchConfiguration('ready_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'path_frame': LaunchConfiguration('path_frame'),
            }],
        ),
    ])
