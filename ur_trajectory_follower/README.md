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

`/velocity_override` scales phase advancement. At zero it freezes the reference
phase; by default the arm and base controllers continue bounded feedback to the
frozen reference. This is a trajectory pause, not an emergency stop.

## Cartesian arm control

`ur_direction_controller` commands a world-frame linear twist:

```text
v_cmd = v_feedforward + v_along + v_lateral + v_spray
```

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
