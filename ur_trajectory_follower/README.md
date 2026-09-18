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
freezes the reference phase; by default the arm and base controllers continue
bounded feedback to the frozen reference. This is a trajectory pause, not an
emergency stop.

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


## Moving-base compensation

Optional backup mode: `base_compensation_translation_only:=true` (default false)
sets `v_comp = -R_world_from_velocity * v_measured`, `omega_comp = 0`.
It drops both angular counter-motion and `omega × r`; linear velocity is used
at the odometry child-frame origin, without shifting that origin. Consequently,
different measurement origins need not give equivalent results during rotation.
With `base_compensation_pose_source:=tcp_pose`, this mode requires only fresh
base pose and odometry, not TCP pose, spray distance, or tool/mounting geometry.
If odometry and configured base frames differ, their fixed relative rotation
is captured from TF. In TF mode only world-to-velocity-frame rotation is needed.
Velocity and base-pose freshness checks still apply; missing inputs yield zero.

The GUI exposes a saved, live checkbox for this mode. Its reliable transient-local
Bool topic is `/am/base_compensation/translation_only`. Switching modes clears
the previous smoothed output and geometry cache; returning to full compensation
reacquires geometry and requires fresh TCP data again. The regular tracking
controllers remain active. This is a TCP-dependency fallback, not a DDS repair;
uncompensated base rotation can still cause tracking errors.

The Web and desktop GUIs automatically start `ur_vel_induced_by_base` with the
arm follower. Standalone `sideways_arm_control.launch.py` opts in with
`start_base_motion_compensation:=true`. The node uses measured odometry, not
commanded base velocity: Robotnik `/robot/robotnik_base_control/odom`, Bunker
`/odom`. Pose and odometry subscriptions use Best-Effort/Volatile QoS with depth
one, so only the latest sample is queued after a busy executor recovers.

The correction is `-R * (v + omega x r)` and `-R * omega`, expressed in
`path_frame`. The lever arm starts at the odometry `child_frame_id` origin and
ends at the deposition point, including the configured nozzle offset and local
+Z spray distance. This matches the J-PARSE reference point. Distance starts at
`spray_distance_initial` and follows `smoothed_spray_distance_topic`.
For `TwistStamped`, the header frame also specifies the velocity origin;
plain `Twist` uses `compensation_base_frame`.

Hardware GUI launches select `base_compensation_pose_source:=tcp_pose`:

- `/robot/arm/tcp_pose_broadcaster/pose` supplies the controller TCP pose in its
  `header.frame_id` (normally `robot_arm_base`, not `robot_arm_base_link`).
- `/robot_pose` supplies the pose of `compensation_base_frame` in `path_frame`.
- Once fresh inputs and TF are available, the node captures base-to-UR-base and
  base-to-velocity-origin geometry. It also captures `tip_link` to
  `base_compensation_controller_tcp_frame` (the raw controller TCP) and converts
  the configured flange-to-nozzle calibration into controller-TCP-to-nozzle.
  Thus an already configured nozzle TCP does not receive the tool offset twice.
- After capture, this node performs no TF lookups. It composes the captured
  mount, live TCP pose, TCP-to-nozzle offset and live spray distance. No additional
  static TF publisher is created, avoiding competing parents in the robot tree.
- Keep the lift height and active UR TCP configuration fixed during a run.
  Restart the arm follower after changing either or the nozzle calibration.
  The startup log confirms when geometry has been captured. Changed pose or
  velocity frame names require a restart as well.
- A missing or expired TCP/base pose produces zero compensation and a diagnostic
  naming the input; there is no automatic fallback to delayed TF. Invalid,
  duplicate and out-of-order pose messages do not replace the last valid sample
  or refresh its receipt time.

Simulation GUI launches and standalone launches retain `base_compensation_pose_source:=tf`:
the node resolves the world-to-velocity and velocity-to-`tip_link` TFs on each
cycle and applies the configured nozzle offset and spray distance.

Launch arguments:

| Argument | Standalone default |
| --- | --- |
| `start_base_motion_compensation` | `false` |
| `base_compensation_pose_source` | `tf` (`tcp_pose` on GUI hardware) |
| `base_compensation_tcp_pose_topic` | `/robot/arm/tcp_pose_broadcaster/pose` |
| `base_compensation_base_pose_topic` | `/robot_pose` |
| `base_compensation_controller_tcp_frame` | `robot_arm_tool0_controller_raw` |
| `base_velocity_topic` | `/odom` |
| `base_velocity_type` | `odometry` (also `twist_stamped`, `twist`) |
| `compensation_base_frame` | `base_link` (fallback for empty frames) |
| `base_compensation_topic` | `/ur_twist_base_compensation_world` |
| `base_compensation_rate` | `100.0` Hz |
| `base_compensation_stale_timeout` | `0.5` s |
| `base_compensation_pose_timeout` | `1.5` s (TCP and base pose) |
| `base_compensation_tf_timeout` | `1.5` s (startup capture / TF mode) |
| `base_compensation_startup_wait_timeout` | `45.0` s |
| `base_compensation_smoothing_coeff` | `0.0` |
| `combined_twist_input_timeout` | `0.5` s |
| `dds_udp_only` | `false` (GUI passes `true`) |

The GUI arm-follower launch sets `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for its child
processes. With the installed Fast DDS builtin transports, this avoids the
`RTPS_TRANSPORT_SHM ... open_and_lock_file failed` path without deleting shared
memory files or changing the robot's configuration. Other GUI processes and the
calling shell are unaffected. Standalone users can opt in with
`dds_udp_only:=true`; `false` preserves the inherited DDS configuration. Custom
XML profiles that disable builtin transports must configure UDP in that profile;
the environment variable does not override such profiles. See the
[Fast DDS transport setting](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html#fastdds-builtin-transports).

During initial discovery, the node reports `connecting` once and publishes zero
until all required inputs and mounting geometry are available. The 45-second
startup grace only controls diagnostics: it neither starts motion nor forces a
45-second delay. After the grace period, missing inputs produce throttled warnings;
reception is retried indefinitely. `Base compensation ready` is logged on first
success and on recovery from an outage.

Pose gaps up to 1.5 seconds reuse the last valid geometry, covering the observed
~0.92-second TCP gaps. Each correction is still recomputed using fresh measured
base velocity; the previous velocity command is not held. The odometry/source
velocity limit remains 0.5 seconds. Source age and local receipt age must both
remain within their respective limits. Longer outages immediately clear output
and smoothing state, and fresh inputs resume compensation automatically. Using
older geometry can reduce compensation accuracy during a gap; the timeout is
configurable and does not remove the need for timely pose delivery.

Only enabled compensation is included in the twist combiner, once, before
rotation into the arm command frame. It is independent of velocity override:
while the reference is paused, actual base motion still requires compensation.
The existing start-condition gate controls combined output.

Invalid/expired velocity, expired poses or unavailable TF yields zero correction.
Static TF is timeless. The combiner expires cached inputs individually, so a
crashed compensation publisher cannot leave its last command active forever.
This fallback does not stop the base or guarantee world-position holding.

For verification, compare `/arm_trajectory_reference` with
`/current_deposition_pose`, and inspect `/ur_twist_world`,
`/ur_twist_base_compensation_world`, and
`/jparse_velocity_controller_ur/twist_cmd_world`. Equal 0.075 m/s base and desired
world translation should cancel in the relative arm translation; rotating the
base should produce the opposite angular velocity and the corresponding lever-arm
correction. At rest the compensation should be zero.

Tracking limits apply before compensation; J-PARSE Cartesian and joint limits
apply to the resulting relative-arm command. Saturation or acceleration limits
can prevent complete cancellation. These limits are intentionally unchanged.
