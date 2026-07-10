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

## Base and TCP accuracy recordings

`trajectory_accuracy_monitor` records a reference path comparison without sending
any command. It accepts the same pose/path contract in simulation and on hardware:

- Base: `/robot_pose` against `/base_path`
- TCP: `/current_tcp_pose` against `/ur_path_transformed`

The monitor writes one CSV with per-sample error vector (`dx`, `dy`, `dz`), absolute
and yaw errors, plus a JSON summary with RMSE, P95, maximum, bias and sample quality.
TCP recordings also contain planar tangential and cross-track errors. For example:

```bash
ros2 run print_path_monitoring trajectory_accuracy_monitor --ros-args \
  -p mode:=base -p actual_pose_topic:=/robot_pose \
  -p reference_path_topic:=/base_path -p run_name:=base_baseline
```

In `am_operator_gui`, use **Record Base Accuracy** for the base-only run and
**Record TCP Accuracy** for the coupled run. Recordings are saved to
`/tmp/am_trajectory_runs`; stopping a recording writes its JSON summary. Select
**Baseline** or **Tuned** before each run. After three runs of each phase, use
**Summarize Accuracy Runs** to create `accuracy_comparison.md` and JSON. A tuning
is accepted only when its median P95 position error is lower and its median maximum
position error is not higher.

When Default velocity is enabled in the GUI, its path-index rate is locked to
`velocity * (number_of_path_steps / total_path_length)`. It is recalculated only
when the requested default velocity or trajectory changes. The comparison report
also checks the paired base/TCP trajectory against the configured conservative
planar reach range and flags it as a possible TCP-error cause.

For static smoke tests without a live path-index publisher, set
`fixed_path_index` to a non-negative index. A live `path_index_topic` message
still takes precedence when it is available.
