# ur_trajectory_follower

ROS 2 Cartesian arm tracking for coupled mobile-base and arm trajectories.

## Coupled trajectory progress

`increment_path_index` keeps the legacy shared `/path_index` contract and adds a
continuous segment phase `/trajectory_phase`.  It publishes interpolated,
transient-local references for both paths:

- `/arm_trajectory_reference`
- `/base_trajectory_reference`

For segment `i -> i + 1`, both references use the same phase `alpha`:

```text
p_ref(alpha) = (1 - alpha) * p_i + alpha * p_(i+1)
q_ref(alpha) = SLERP(q_i, q_(i+1), alpha)
```

`progress_mode:=timestamp` uses the exported segment timestamps.
`progress_mode:=desired_speed` derives each non-zero arm segment duration from
`desired_arm_speed`; zero-length arm segments retain their original timestamp
duration so base motion, orientation-only motion, and dwell segments remain
valid. Coupled paths must have equal lengths and matching, strictly increasing
timestamps.

`/velocity_override` scales phase advancement and arm feedforward. At zero it
freezes the reference phase; the arm and base controllers continue bounded
feedback to the frozen reference. This is a trajectory pause, not an emergency
stop.

## Cartesian arm control

`ur_direction_controller` commands a world-frame linear twist:

```text
v_cmd = v_feedforward + v_along + v_lateral + v_spray
```

For debugging, the controller publishes the already-limited and smoothed
components separately on `/ur_twist_world_feedforward` and
`/ur_twist_world_control`; the latter is the sum of the three feedback terms.
The twist combiner subscribes to both topics. The legacy `/ur_twist_world`
topic remains available as their combined value for monitoring.

Each combiner input is fresh only for `input_timeout` seconds after local receipt
(0.5 seconds by default). A missing or stale input contributes a zero twist;
the combiner warns once for each input that becomes stale and warns again only
after that input becomes fresh and later expires. `sideways_arm_control.launch.py` exposes this as
`combined_twist_input_timeout`.

The feedforward follows the active segment. Along-track, lateral, and spray-axis
terms use measured deposition-pose error and are independently bounded before a
global Cartesian velocity limit. The defaults are:

```text
along_track_kp: 2.0 s^-1
orthogonal_kp: 1.0 s^-1
max_along_track_correction: 0.03 m/s
orthogonal_max_velocity: 0.02 m/s
max_spray_axis_correction: 0.03 m/s
max_tracking_linear_velocity: 0.12 m/s
final_position_tolerance: 0.005 m
```

The legacy `pid_twist_controller` remains available for external users but is
not launched in the arm-following chain: it processed a velocity command rather
than a measured Cartesian tracking error.

J-PARSE remains responsible only for mapping the Cartesian twist to bounded
joint velocities.

## TCP and nozzle poses in simulation

With derive_nozzle_pose_from_tcp:=true, the arm stack treats
/current_tcp_pose as the raw tool0/TCP pose and publishes
/current_nozzle_tip_pose = current_tcp_pose * fixed_tool_offset. The
deposition-pose node uses this derived pose, so its stand-off is measured from
the configured nozzle rather than from tool0. This is used by Robotnik
simulation and by either selected MuR arm. The shared AM J-PARSE controller
uses the same offset and dynamic spray distance in its Jacobian, so Cartesian
velocity is solved at the deposition point as well.

## Moving-base compensation

When the arm and base follow a paired print trajectory, start
`sideways_arm_control.launch.py` with `start_base_motion_compensation:=true`.
`ur_vel_induced_by_base` reads the base velocity, resolves the current
base-to-tool TF offset, and publishes the negative TCP velocity induced by
base translation and angular velocity. The correction is combined with the
world-frame arm command before it is transformed into the arm controller
frame. Missing or stale velocity/TF input produces zero compensation.

The paired RB-VOGUI demo enables this by default. Its base velocity source is
`/robot/robotnik_base_control/odom`; override `base_velocity_topic` and
`base_velocity_type` (`odometry`, `twist_stamped`, or `twist`) for another
platform.
