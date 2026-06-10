# parse_paths

ROS 2 path-publisher utilities for simple additive-manufacturing simulation tests.

The generic node `test_path_generator` publishes `nav_msgs/msg/Path` and can create:

- `line`
- `rectangle`
- `circle`
- `waypoints`

It uses standard ROS messages only. The node can also publish a matching original path
topic and a constant normal vector for consumers that expect those topics.

## Run

```bash
ros2 launch parse_paths generate_test_path.launch.py
```

Common overrides:

```bash
ros2 launch parse_paths generate_test_path.launch.py \
  path_type:=circle \
  path_topic:=/base_path \
  frame_id:=map
```

For detailed parameters, start from:

```bash
ros2 pkg prefix parse_paths
```

and inspect `share/parse_paths/config/test_path_generator.yaml`.

## Important Parameters

- `path_type`: `line`, `rectangle`, `circle`, or `waypoints`.
- `path_topic`: output `nav_msgs/msg/Path` topic.
- `frame_id`: frame used by the path and poses.
- `num_points`: number of generated poses.
- `time_step`: timestamp spacing between generated poses.
- `orientation_mode`: `tangent` or `fixed`.
- `fixed_yaw`: yaw used when `orientation_mode` is `fixed`.
- `publish_once`: publish once with transient-local QoS, or republish at `publish_rate`.

Shape-specific parameters:

- line: `line_start`, `line_end`
- rectangle: `rectangle_center`, `rectangle_width`, `rectangle_height`
- circle: `circle_center`, `circle_radius`, `closed_path`
- waypoints: flattened `waypoints` list `[x0, y0, z0, x1, y1, z1, ...]`

## Existing Specialized Publishers

- `publish_sideways_arm_test_path`: arm-only sideways path used by the current UR
  sideways test launch.
- `publish_front_side_arm_base_paths`: paired arm/base path publisher that preserves
  the startup XY offset between the arm path and mobile-base path.
- `publish_robotnik_base_arm_paths`: paired RB-VOGUI base/UR arm publisher. The
  base path keeps a fixed yaw while moving sideways and then in a 45 degree
  direction; the arm path has the same number of indices, a small XY offset, and a
  height change.

## Robotnik Paired Base/Arm Paths

Generate synchronized base and arm paths for the RB-VOGUI demo:

```bash
ros2 launch parse_paths robotnik_base_arm_paths.launch.py
```

Default topics:

- base path: `/base_path`
- base original path: `/base_path_original`
- arm path: `/ur_path_transformed`
- arm original path: `/ur_path_original`
- normal vector: `/normal_vector`
- base pose input: `/robot_pose`
- TCP pose input: `/current_tcp_pose`

By default, the publisher waits for the current base and TCP poses and then creates
the start poses offset from those current poses. For a deterministic path without
pose inputs:

```bash
ros2 launch parse_paths robotnik_base_arm_paths.launch.py \
  use_current_poses:=false \
  base_start_xyz:='[0.0, 0.0, 0.0]' \
  arm_start_xyz:='[0.6, 0.0, 0.8]'
```
