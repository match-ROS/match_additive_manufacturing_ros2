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
