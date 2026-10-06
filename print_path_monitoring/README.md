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

When Default velocity is enabled in the GUI, the coupled trajectory advances by
the requested arm speed. Otherwise, it advances according to the exported
per-segment timestamps. The comparison report also checks the paired base/TCP
trajectory against the configured conservative planar reach range and flags it
as a possible TCP-error cause.

For static smoke tests without a live path-index publisher, set
`fixed_path_index` to a non-negative index. A live `path_index_topic` message
still takes precedence when it is available.

## Paper datasets

The desktop and web operator GUIs provide **Record Paper Dataset** and
**Summarize Paper Datasets**. The default persistent output root is
`~/am_accuracy_runs`; set **Paper output directory** to a campaign directory.
Use one session per trial and set a condition label and trial notes before starting.

1. Launch the trajectory/pose stack and position the robot at the selected start index.
2. Set the phase, condition, output directory and trial notes. Include the sensor
   calibration identifier, payload and environment in the notes.
3. Press **Record Paper Dataset** and wait for the console to report that the bag
   is recording. The recorder waits for fresh paths, trajectory state and sensor poses.
4. Press **Start Following**. Both measurements use the same source start gate.
   A recording started during motion is a partial trial.
5. Allow the endpoint to settle; press **Stop Paper Recording** to finalize files.
   Stopping the recording does not change motion commands.
6. Repeat the trial and conditions, then press **Summarize Paper Datasets**.

Each `run_<UTC timestamp>/` directory contains:

- `manifest.json`: session/condition, units, configuration, effective platform
  settings, commands, git revision/dirty status, timestamps and finalization status.
- `base.csv` and `tcp.csv`, plus JSON summaries: sensor/receipt/reference timestamps,
  full actual/target poses, frames, index/segment phase, speed/override, Cartesian
  and angular errors, quality counters, and active/paused/settling window statistics.
- Hashed snapshots of the exact processed reference paths, and copies/hashes of
  exported source files when available.
- `runtime_parameters/`: live ROS parameter dumps and explicit errors for nodes
  that could not be queried. Parameters are captured at startup; keep
  gains/calibration fixed during a trial.
- `raw/`: the rosbag2 recording of poses, external measurement topics, odometry,
  paths, reference/state messages, TF, commands, joints, spray distance and events.
  `bag_qos.yaml` and the manifest preserve the recording configuration.

The existing reference headers contain **planned trajectory time**, not live
measurement time. `/trajectory_state` (`std_msgs/String`, JSON schema version 1)
therefore carries one atomic snapshot with live ROS `stamp_ns`, both references,
path index, segment phase, start gate and speeds. After receiving a state at/after
the sensor sample, the monitor selects the last source state at/before that sample.
It measures the discrete commanded target with a maximum reference age of 0.1 s;
it does not interpolate a future target or compensate away tracking lag. CSV
`reference_age` makes the timing resolution visible. All sources must use the same
ROS clock (and synchronized hardware clocks); simulation sessions use `/clock`.

TCP measurements use `/measured_deposition_pose`, emitted once per new nozzle
sample with its sensor timestamp and current smoothed stand-off distance.
The periodic `/current_deposition_pose` remains the control input. The deposition
point is computed from nozzle pose/calibration and stand-off; these measurements
do not directly measure deposited material or final surface geometry.
Estimated/odometry/fallback sources remain identifiable in the manifest and raw bag.
Planar reach classification is disabled in paper sessions because mount geometry
must be established separately rather than assumed from defaults.

Summaries retain sample-weighted metrics for compatibility and provide separate
time-weighted active/paused/settling metrics. Time weighting holds each accepted
error until the next accepted sample, excludes gaps over 0.25 s (`max_sample_gap`) and does not bridge
pause/resume episodes. A 100-bin progress diagnostic weights covered index/phase
bins equally; it is not an arc-length metric. Endpoint statistics use the final
one-second settling window, in addition to the legacy last-sample endpoint metric.
Check sample timestamps, gaps and coverage alongside errors.

The paper report groups conditions by platform, processed path hashes, frame,
sensor source, configured speed and initial index. It excludes unfinished bags,
uncompleted paths, changed paths, missing active duration, and recordings below
95% accepted samples. Sessions started after motion and bags missing required
measurement topics are also excluded. Exclusions remain in the JSON report. Each trial has equal
weight; 95% percentile bootstrap intervals resample independent trial metrics
(2000 resamples, fixed seed), never individual pose samples. A single trial has no
confidence interval. Keep tuning trials separate from final evaluation trials.

Reports can also be generated without the GUI:

```bash
ros2 run print_path_monitoring paper_accuracy_report \
  --input-directory ~/am_accuracy_runs
```

Outputs are `paper_accuracy_report.json` (including per-run values and exclusions)
and `paper_accuracy_report.md`. Raw bags can be replayed with
`ros2 bag play <run directory>/raw --clock`, using an isolated evaluation domain
and `use_sim_time:=true` on monitors. Do not replay command topics into a live robot.
