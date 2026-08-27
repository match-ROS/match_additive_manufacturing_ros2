from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ros_domain_id', default_value='38'),
        DeclareLaunchArgument(
            'ui', default_value='qt',
            description='Operator interface: qt (reference GUI) or web (browser GUI).',
        ),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID',
            LaunchConfiguration('ros_domain_id'),
        ),
        Node(
            package='am_operator_gui',
            executable='am_operator_gui',
            name='am_operator_gui',
            output='screen',
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('ui'), "' == 'web'"])),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='am_operator_gui',
            executable='am_operator_web',
            name='am_operator_web',
            output='screen',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ui'), "' == 'web'"])),
        ),
    ])
