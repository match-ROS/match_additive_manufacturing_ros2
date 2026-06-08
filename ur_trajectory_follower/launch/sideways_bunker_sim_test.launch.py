from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bunker_description'),
                'launch',
                'spawn_with_controllers.launch.py',
            ])
        ),
        launch_arguments={
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    switch_to_velocity = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('switch_to_velocity_controller')),
        cmd=[
            'ros2',
            'control',
            'switch_controllers',
            '--controller-manager',
            LaunchConfiguration('controller_manager'),
            '--deactivate',
            'ur_joint_trajectory_controller',
            '--activate',
            'ur_forward_velocity_controller',
        ],
        output='screen',
    )

    start_pose = Node(
        package='ur_trajectory_follower',
        executable='publish_joint_pose_once',
        name='sideways_start_joint_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('move_to_start_pose')),
        parameters=[{
            'use_sim_time': True,
            'trajectory_topic': '/ur_joint_trajectory_controller/joint_trajectory',
            'joint_names': [
                'ur_shoulder_pan_joint',
                'ur_shoulder_lift_joint',
                'ur_elbow_joint',
                'ur_wrist_1_joint',
                'ur_wrist_2_joint',
                'ur_wrist_3_joint',
            ],
            'positions': LaunchConfiguration('start_joint_positions'),
            'time_from_start': '4.0',
            'publish_delay': '0.1',
        }],
    )

    sideways_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_trajectory_follower'),
                'launch',
                'sideways_arm_control.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'bunkur',
            'arm': 'ur',
            'joint_prefix': 'ur_',
            'base_link': 'ur_base_link',
            'tip_link': 'ur_tool0',
            'robot_description_topic': '/robot_description',
            'joint_states_topic': '/joint_states',
            'velocity_command_topic': '/ur_forward_velocity_controller/commands',
            'start_jparse_controller': 'true',
            'path_length': LaunchConfiguration('path_length'),
            'num_points': LaunchConfiguration('num_points'),
            'time_step': LaunchConfiguration('time_step'),
            'wait_for_start_condition': LaunchConfiguration('wait_for_start_condition'),
            'move_to_start_pose': 'false',
            'start_pose_trajectory_topic': '/ur_joint_trajectory_controller/joint_trajectory',
            'start_joint_positions': LaunchConfiguration('start_joint_positions'),
            'start_pose_publish_delay': '5.0',
            'path_direction': LaunchConfiguration('path_direction'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        DeclareLaunchArgument('switch_to_velocity_controller', default_value='true'),
        DeclareLaunchArgument('path_length', default_value='1.2'),
        DeclareLaunchArgument('num_points', default_value='50'),
        DeclareLaunchArgument('time_step', default_value='0.1'),
        DeclareLaunchArgument('path_direction', default_value='[-1.0, 0.0, 0.0]'),
        DeclareLaunchArgument('wait_for_start_condition', default_value='false'),
        DeclareLaunchArgument('move_to_start_pose', default_value='false'),
        DeclareLaunchArgument(
            'start_joint_positions',
            default_value='[0.7854, -1.57, 1.5707963268, -0.0, 0.7854, 0.0]',
        ),
        sim_launch,
        TimerAction(period=5.0, actions=[start_pose]),
        TimerAction(period=10.0, actions=[switch_to_velocity]),
        TimerAction(period=12.0, actions=[sideways_control]),
    ])
