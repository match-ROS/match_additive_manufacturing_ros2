"""Publish standalone arm tuning paths for existing GUI/RViz tracking controls."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _setup(context):
    test = LaunchConfiguration('test').perform(context)
    folder = Path(__file__).resolve().parent / 'arm_p_tuning' / test
    if folder.parent != Path(__file__).resolve().parent / 'arm_p_tuning' or not (folder / 'arm_path.json').is_file():
        raise ValueError(f'Unknown arm tuning test: {test}')
    publisher = Path(get_package_share_directory('parse_paths')) / 'launch' / 'robotnik_base_arm_paths.launch.py'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(publisher)),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'vicon_world',
                'load_exported_trajectories': 'true',
                'trajectory_directory': str(folder),
                'publish_once': 'false',
                'base_path_topic': '/base_path',
                'path_transform_xyz': LaunchConfiguration('path_transform_xyz'),
                'path_transform_yaw_deg': LaunchConfiguration('path_transform_yaw_deg'),
            }.items(),
        ),
        Node(
            package='ur_trajectory_follower', executable='increment_path_index',
            name='increment_path_index', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'path_topic': '/ur_path_transformed',
                'base_path_topic': '/base_path',
                'progress_mode': 'timestamp',
                'enable_path_resampling': False,
                'initial_path_index': 0,
                'processed_path_topic': '/ur_path_tracking',
                'processed_base_path_topic': '/base_path_tracking',
                'arm_reference_topic': '/arm_trajectory_reference',
                'base_reference_topic': '/base_trajectory_reference',
                'path_index_command_topic': '/path_index_command',
                'start_condition_topic': '/start_condition',
                'wait_for_start_condition': True,
            }],
        ),
        # The existing arm follower may have a positive GUI-configured speed.
        # Zero selects segment-timestamp feedforward rather than that override.
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--rate', '1', '--print', '100000',
                 '/desired_arm_speed', 'std_msgs/msg/Float32', '{data: 0.0}'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test', default_value='01_along_track'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('path_transform_xyz', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('path_transform_yaw_deg', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
