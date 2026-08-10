from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('component_name', default_value='rectangleRoundedCorners'),
        DeclareLaunchArgument('component_root', default_value=''),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('start_index', default_value='10'),
        DeclareLaunchArgument('end_trim', default_value='1'),
        DeclareLaunchArgument('arm_suffix', default_value=''),
        DeclareLaunchArgument('base_suffix', default_value=''),
        DeclareLaunchArgument('base_z', default_value='0.0'),
        DeclareLaunchArgument('arm_transform_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('arm_transform_rpy', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('base_transform_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('base_transform_rpy', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('normal_vector', default_value='[0.0, 1.0, 0.0]'),
        DeclareLaunchArgument('arm_filename', default_value='arm_path.json'),
        DeclareLaunchArgument('base_filename', default_value='base_path.json'),
        DeclareLaunchArgument('normal_filename', default_value='normal_vector.json'),
        Node(
            package='parse_paths',
            executable='export_component_paths',
            name='component_path_exporter',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'component_name': LaunchConfiguration('component_name'),
                'component_root': LaunchConfiguration('component_root'),
                'output_directory': LaunchConfiguration('output_directory'),
                'frame_id': LaunchConfiguration('frame_id'),
                'start_index': LaunchConfiguration('start_index'),
                'end_trim': LaunchConfiguration('end_trim'),
                'arm_suffix': LaunchConfiguration('arm_suffix'),
                'base_suffix': LaunchConfiguration('base_suffix'),
                'base_z': LaunchConfiguration('base_z'),
                'arm_transform_xyz': LaunchConfiguration('arm_transform_xyz'),
                'arm_transform_rpy': LaunchConfiguration('arm_transform_rpy'),
                'base_transform_xyz': LaunchConfiguration('base_transform_xyz'),
                'base_transform_rpy': LaunchConfiguration('base_transform_rpy'),
                'normal_vector': LaunchConfiguration('normal_vector'),
                'arm_filename': LaunchConfiguration('arm_filename'),
                'base_filename': LaunchConfiguration('base_filename'),
                'normal_filename': LaunchConfiguration('normal_filename'),
            }],
        ),
    ])
