# print_path_monitoring

Monitoring-only nodes for print-path simulation experiments.

`nozzle_pose_monitor` compares an externally supplied TCP/nozzle pose with either:

- a direct reference pose topic, or
- a `nav_msgs/msg/Path` plus current path index or fixed fallback index.

It publishes diagnostics only. It does not command the robot, estimate pose, or apply
nozzle/TCP correction.

## Run

```bash
ros2 launch print_path_monitoring nozzle_pose_monitor.launch.py
```

Default topics:

- TCP/nozzle pose: `/current_tcp_pose`
- reference path: `/ur_path_transformed`
- path index: `/path_index`
- fixed fallback path index: disabled by default with `fixed_path_index: -1`
- position error vector: `/nozzle_position_error`
- position error norm: `/nozzle_position_error_norm`
- yaw error: `/nozzle_yaw_error`

To monitor against a direct reference pose, set `reference_pose_topic` in the YAML or
pass it with `ros2 run` parameters.

For static smoke tests without a live path-index publisher, set
`fixed_path_index` to a non-negative index. A live `path_index_topic` message
still takes precedence when it is available.
