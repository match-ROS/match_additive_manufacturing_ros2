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
