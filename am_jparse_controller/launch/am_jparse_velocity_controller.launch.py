from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    argument_names = (
        'robot_name', 'arm', 'base_link', 'tip_link',
        'fixed_tool_offset_xyz', 'fixed_tool_offset_quaternion_xyzw',
        'spray_distance_topic', 'robot_description_topic', 'twist_topic',
        'command_topic', 'joint_states_topic', 'singular_values_topic',
        'debug_twist_topic', 'readiness_topic', 'rate_hz', 'command_timeout',
        'joint_state_timeout', 'command_joint_names_csv', 'gamma',
        'singular_gain_position', 'singular_gain_angular', 'pinv_tolerance',
        'max_joint_velocity', 'max_cartesian_linear_velocity',
        'max_cartesian_angular_velocity',
    )
    defaults = {
        'use_sim_time': 'false',
        'robot_name': 'robot',
        'arm': 'arm',
        'base_link': 'robot_arm_base_link',
        'tip_link': 'robot_arm_tool0',
        'fixed_tool_offset_xyz': '[0.0, 0.0, 0.0]',
        'fixed_tool_offset_quaternion_xyzw': '[0.0, 0.0, 0.0, 1.0]',
        'spray_distance_topic': '/spray_distance_smoothed',
        'robot_description_topic': '/robot/robot_description',
        'twist_topic': '~/twist_cmd',
        'command_topic': '/robot/arm/forward_velocity_controller/commands',
        'joint_states_topic': '/robot/joint_states',
        'singular_values_topic': '/am/jparse/singular_values',
        'debug_twist_topic': '/am/jparse/debug_twist',
        'readiness_topic': '/am/jparse_ready',
        'rate_hz': '500.0',
        'command_timeout': '0.12',
        'joint_state_timeout': '0.5',
        'command_joint_names_csv': '',
        'gamma': '0.1',
        'singular_gain_position': '1.0',
        'singular_gain_angular': '1.0',
        'pinv_tolerance': '1.0e-6',
        'max_joint_velocity': '1.5',
        'max_cartesian_linear_velocity': '0.25',
        'max_cartesian_angular_velocity': '0.8',
    }
    parameters = {name: LaunchConfiguration(name) for name in argument_names}
    parameters['use_sim_time'] = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()] + [
            Node(
                package='am_jparse_controller',
                executable='jparse_velocity_controller',
                name='am_jparse_velocity_controller',
                output='screen',
                parameters=[parameters],
            )
        ]
    )
