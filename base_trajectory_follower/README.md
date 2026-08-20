# base_trajectory_follower

Generic ROS 2 mobile-base path follower for early simulation tests.

The first node, `simple_base_follower`, is intentionally small and platform-light:

- subscribes to `nav_msgs/msg/Path`
- subscribes to an external base pose topic
- publishes `geometry_msgs/msg/Twist` by default
- can publish `geometry_msgs/msg/TwistStamped` when `output_stamped:=true`
- supports x, y, and yaw velocity for omnidirectional bases such as RB-VOGUI
- can optionally consume a shared external `std_msgs/msg/Int32` path index
- publishes independent sequential geometric base-progress diagnostics for coupled paths
- can wait for a shared `std_msgs/msg/Bool` start condition before publishing commands
- publishes zero velocity if no path, no pose, stale pose, or goal reached

It does not estimate pose, run localization, perform obstacle avoidance, or compensate
TCP/base motion. Those concerns belong in later branches.

## Run

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py
```

RB-VOGUI-style unstamped command topic:

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py \
  path_topic:=/base_path \
  robot_pose_topic:=/robot_pose \
  cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel_unstamped \
  output_stamped:=false
```

Stamped command fallback:

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py \
  cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel \
  output_stamped:=true
```

## Important Parameters

- `lookahead_distance`: distance used to pick a forward control target on the path.
  With `use_external_path_index:=true`, the external index remains the progress
  anchor and this distance selects a target ahead of that anchor.
- `use_external_path_index`: if true, use `path_index_topic` as the progress
  anchor instead of selecting progress from the current pose.
- `path_index_topic`: shared path-index input, default `/path_index`.
- `wait_for_start_condition`: if true, stay quiet until `start_condition_topic`
  publishes true.
- `start_condition_topic`: shared start gate, default `/start_condition`.
- `stale_pose_timeout`: max allowed age of the latest pose before commanding zero.
- `kp_x`, `kp_y`, `kp_yaw`: proportional gains in robot frame.
- `max_vx`, `max_vy`, `max_wz`: command limits.
- `xy_goal_tolerance`, `yaw_goal_tolerance`: final pose tolerances.
- `allow_reverse`: if false, negative x velocity is clamped to zero.
- `pure_pursuit_k_progress`: gain (m/s per m) for the signed base-path
  geometric progress error.  It affects only Pure Pursuit linear feedforward.
- `max_progress_speed_correction`: absolute bound on that extra linear speed.
- `base_progress_xy_tolerance`, `base_progress_yaw_tolerance`: deprecated
  compatibility parameters; translational progress no longer uses reach gates.

For an external index, the node maps `/path_index` through
`external_path_index_stride` to its `base_reference_index`.  Separately,
`/base_progress_index` is a forward-only, sequential estimate of translational
progress. It advances when the next densely sampled base waypoint is closer to
the measured base XY pose than the current waypoint, and it skips zero-XY/yaw-only entries because
they add no arc length.  `/base_progress_error_m` is the discrete arc-length
difference `arc_length(base_reference_index) - arc_length(base_progress_index)`.
`/base_reference_progress_m` and `/base_progress_arc_length_m` publish the two
coordinates.  This is intentionally a discrete approximation; consider a local
continuous segment projection if base paths become sparse or quantization causes
visible catch-up steps.

## Expected Topic Contract

Base pose:

- `geometry_msgs/msg/PoseStamped`, default `/robot_pose`
- `geometry_msgs/msg/Pose` is also supported with `robot_pose_type:=pose`

Reference path:

- `nav_msgs/msg/Path`, default `/base_path`

Command:

- `geometry_msgs/msg/Twist`, default `/robot/robotnik_base_control/cmd_vel_unstamped`
- or `geometry_msgs/msg/TwistStamped` with `output_stamped:=true`
