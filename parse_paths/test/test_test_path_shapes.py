import math

from parse_paths.test_path_shapes import (
    generate_circle,
    generate_line,
    generate_rectangle,
    generate_waypoints,
    tangent_yaw,
)


def test_generate_line_includes_start_and_end():
    points = generate_line([0.0, 0.0, 0.0], [2.0, 0.0, 0.0], 3)

    assert len(points) == 3
    assert points[0].tolist() == [0.0, 0.0, 0.0]
    assert points[-1].tolist() == [2.0, 0.0, 0.0]


def test_generate_rectangle_samples_requested_count():
    points = generate_rectangle([0.0, 0.0, 0.0], 2.0, 1.0, 8)

    assert len(points) == 8
    assert points[0].tolist() == [-1.0, -0.5, 0.0]


def test_generate_circle_uses_requested_count():
    points = generate_circle([1.0, 2.0, 0.0], 0.5, 12)

    assert len(points) == 12
    assert points[0].tolist() == [1.5, 2.0, 0.0]


def test_generate_waypoints_interpolates_polyline():
    points = generate_waypoints([0.0, 0.0, 0.0, 0.0, 2.0, 0.0], 3)

    assert len(points) == 3
    assert points[1].tolist() == [0.0, 1.0, 0.0]


def test_tangent_yaw_follows_segment_direction():
    points = generate_line([0.0, 0.0, 0.0], [0.0, 2.0, 0.0], 3)

    assert math.isclose(tangent_yaw(points, 0), math.pi / 2.0)
