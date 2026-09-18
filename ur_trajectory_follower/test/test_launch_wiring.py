from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_sideways_controllers_use_processed_path_for_resampled_index() -> None:
    source = (PACKAGE_ROOT / 'launch' / 'sideways_arm_control.launch.py').read_text(
        encoding='utf-8'
    )

    wiring = "'path_topic': LaunchConfiguration('tracking_path_topic')"
    assert source.count(wiring) == 2
    assert "('path', LaunchConfiguration('tracking_path_topic'))" not in source


def test_standalone_direction_launch_sets_path_parameter() -> None:
    source = (PACKAGE_ROOT / 'launch' / 'ur_direction_controller.launch.py').read_text(
        encoding='utf-8'
    )

    assert "'path_topic': path_topic" in source


def test_compensation_launch_parameters_and_single_combiner_input(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    spec = importlib.util.spec_from_file_location(
        'sideways_compensation_test', PACKAGE_ROOT / 'launch' / 'sideways_arm_control.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    assert context.launch_configurations['start_base_motion_compensation'] == 'false'
    assert context.launch_configurations['base_compensation_pose_source'] == 'tf'
    assert context.launch_configurations['base_compensation_translation_only'] == 'false'
    context.launch_configurations['base_compensation_pose_source'] = 'tcp_pose'
    context.launch_configurations['start_jparse_controller'] = 'false'
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    for enabled in ('false', 'true'):
        context.launch_configurations['start_base_motion_compensation'] = enabled
        context.launch_configurations['base_compensation_translation_only'] = enabled
        nodes = module._launch_setup(context)
        nodes = [n for n in nodes if isinstance(n, dict)]
        compensation = [n for n in nodes if n['executable'] == 'ur_vel_induced_by_base']
        assert len(compensation) == 1
        assert compensation[0]['condition'].evaluate(context) == (enabled == 'true')
        parameters = evaluate_parameters(context, normalize_parameters(compensation[0]['parameters']))[0]
        assert parameters['tcp_frame'] == context.launch_configurations['tip_link']
        assert parameters['pose_source'] == 'tcp_pose'
        assert parameters['translation_only'] == (enabled == 'true')
        assert parameters['tcp_pose_topic'] == '/robot/arm/tcp_pose_broadcaster/pose'
        assert parameters['base_pose_topic'] == '/robot_pose'
        assert parameters['controller_tcp_frame'] == 'robot_arm_tool0_controller_raw'
        assert parameters['pose_timeout'] == 1.5
        assert parameters['tf_timeout'] == 1.5
        assert parameters['startup_wait_timeout'] == 45.0
        assert parameters['spray_distance_topic'] == context.launch_configurations['smoothed_spray_distance_topic']
        assert tuple(parameters['fixed_tool_offset_xyz']) == (0., 0., 0.)
        combiner = next(n for n in nodes if n['executable'] == 'combine_twists')
        parameters = evaluate_parameters(context, normalize_parameters(combiner['parameters']))[0]
        assert isinstance(parameters['twist_topics'], str)
        assert parameters['twist_topics'].count('/ur_twist_base_compensation_world') == (enabled == 'true')
        assert parameters['input_timeout'] == .5


def test_udp_transport_is_scoped_to_follower_launch(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location(
        'sideways_udp_test', PACKAGE_ROOT / 'launch' / 'sideways_arm_control.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    for enabled in ('false', 'true'):
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
        context.launch_configurations['dds_udp_only'] = enabled
        context.launch_configurations['start_jparse_controller'] = 'false'
        context.environment['FASTDDS_BUILTIN_TRANSPORTS'] = 'DEFAULT'
        actions = module._launch_setup(context)
        assert isinstance(actions[0], SetEnvironmentVariable)
        if actions[0].condition.evaluate(context):
            actions[0].execute(context)
        assert context.environment['FASTDDS_BUILTIN_TRANSPORTS'] == ('UDPv4' if enabled == 'true' else 'DEFAULT')
