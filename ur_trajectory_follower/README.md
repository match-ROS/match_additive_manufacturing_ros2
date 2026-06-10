# ur_trajectory_follower

ROS 2 nodes for following an arm path with linear and orientation twist commands.

## Direction Control Modes

`ur_direction_controller` supports two planar direction modes:

- `goal_direction`: points the planar velocity from the current TCP pose toward the
  selected goal waypoint. This preserves the original ROS 2 behavior.
- `speed_orthogonal`: follows the current path-segment tangent at the speed encoded
  by waypoint positions and timestamps, then adds bounded cross-track correction.

For `speed_orthogonal`, the planar command is:

```text
v = tangent * trajectory_speed * velocity_override
    + clamp(orthogonal_kp * cross_track_error, orthogonal_max_velocity)
```

The tangent and cross-track correction are projected onto the plane orthogonal to
the configured spray axis. Spray-axis/nozzle-height PID control remains separate.

Relevant parameters:

- `control_mode`: `goal_direction` or `speed_orthogonal`.
- `from_index_offset`: segment start relative to `/path_index`.
- `goal_index_offset`: segment goal relative to `/path_index`.
- `orthogonal_kp`: proportional cross-track correction gain.
- `orthogonal_max_velocity`: maximum cross-track correction speed in m/s.
- `velocity_override_topic`: scales the trajectory feed-forward speed.

Standalone launch:

```bash
ros2 launch ur_trajectory_follower ur_direction_controller.launch.py \
  control_mode:=speed_orthogonal \
  orthogonal_kp:=1.0 \
  orthogonal_max_velocity:=0.1
```

The paired Robotnik base+arm demo enables `speed_orthogonal` by default.
