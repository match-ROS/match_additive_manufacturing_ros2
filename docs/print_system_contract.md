# ROS 2 Print-System Contract

This document is the hardware-independent contract for the ROS 2 additive
manufacturing system.  It defines the inputs consumed by application packages,
the outputs they are allowed to command, and the operating profiles used during
offline, simulation, and later hardware validation.

## Scope and ownership

`match_additive_manufacturing_ros2` owns print paths, progress, coordination,
tracking, monitoring, and operator workflow.  It does not own vendor drivers,
robot descriptions, or safety hardware.

`match_mobile_robotics_jazzy` owns MiR/MuR platform descriptions, base-driver
integration, platform transforms, and platform controller configuration.

`bunker_manipulator` owns Bunker-specific simulation and controller wiring.
Generic Cartesian-to-joint controller logic currently supplied by
`controllers_ros2` should be extracted to a dedicated ROS 2 controller package
before it is used by more than one physical platform.

Future ROS 2 `match_mocap` packages own the Qualisys/QTM transport and generic
mocap-to-map calibration.  Future ROS 2 `match_hardware_utilities` packages own
Keyence transport and raw profile decoding.  The AM repository owns
profile-to-print-error estimation and contour correction.

## Canonical topics

| Topic | Type | Producer | Consumer | Contract |
| --- | --- | --- | --- | --- |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | exactly one selected pose adapter | base follower, start mover, GUI | Fresh pose in `path_frame`; no competing publishers. |
| `/current_tcp_pose` | `geometry_msgs/msg/PoseStamped` | TF adapter or platform pose source | arm start mover, monitor | Fresh tool-centre pose in `path_frame`. |
| `/current_deposition_pose` | `geometry_msgs/msg/PoseStamped` | deposition/TCP adapter | direction controller, GUI | Calibrated nozzle/deposition point in `path_frame`. |
| `/base_path` | `nav_msgs/msg/Path` | path publisher | base follower, start mover | Static or transient-local paired path with valid timestamps. |
| `/ur_path_transformed` | `nav_msgs/msg/Path` | path publisher | arm follower, monitor | Same number of poses and timestamps as `/base_path`. |
| `/path_index` | `std_msgs/msg/Int32` | shared progress node | base and arm followers | Index is within both paired paths. |
| `/trajectory_phase` | `std_msgs/msg/Float64` | shared progress node | reference consumers | Segment phase is in `[0, 1]`. |
| `/start_condition` | `std_msgs/msg/Bool` | GUI/operator | motion followers | `false` must inhibit non-zero following commands. |
| base command | `Twist` or `TwistStamped` | base follower/start mover | platform driver | Platform profile fixes type, topic, frame, and limits. |
| arm Cartesian command | `TwistStamped` | AM arm controller | J-PARSE/controller adapter | Bounded command in the configured world frame. |

Path, robot, TCP, and deposition poses must all resolve to the configured
`path_frame`.  A node must reject or safely stop on an empty frame, missing TF,
non-finite values, stale input, or a timestamp outside the configured tolerance.

## Operating profiles

### `localization_only`

The selected platform localization or odometry adapter is the sole publisher of
`/robot_pose`.  The print system uses path and deposition-pose feedback only.
No mocap or Keyence input is required.

### `mocap_localization`

A Qualisys/Vicon adapter is the sole publisher of `/robot_pose` after its
calibration is valid.  The platform localization remains available only as a
diagnostic comparison source.  A missing, stale, or invalid mocap pose prevents
motion.

### `localization_contour_monitor`

`localization_only` or `mocap_localization` is active.  Keyence profiles are
decoded and converted to lateral/height diagnostic errors, but those errors do
not affect arm or base commands.

### `localization_contour_control`

Contour diagnostics are fresh and calibrated.  The AM controller may add a
bounded contour correction to the arm command.  A stale, invalid, or
out-of-range profile removes that correction and reports an interlock fault;
it must never preserve the last non-zero correction.

The current implementation adds the correction as a third `Twist` input to the
existing arm twist combiner (`/contour/twist_world`). It is launched with
`contour_control_enabled:=false` by default. The operator GUI exposes the
setting as **Enable bounded contour correction** and persists the explicit
choice; a new arm-follower launch uses that live choice. Enabling it also
requires fresh lateral and height errors and `/start_condition=true`; otherwise
it repeatedly contributes zero. The correction is bounded in world Y/Z by its
configured velocity limits.

## Safety invariants

1. Only one node publishes `/robot_pose` in each profile.
2. Followers publish zero command when their gate is closed, input is stale,
   input is invalid, a path is unavailable, or their goal is reached.
3. `Stop Following` repeatedly commands zero to both base and arm endpoints;
   it is not a replacement for the safety-rated stop.
4. Contour control is disabled by default and cannot be enabled by a valid
   profile alone; the operator must select the control profile explicitly.
5. A platform launch owns platform remappings and controller endpoints.  Generic
   AM nodes remain free of robot-specific topic names.
6. A platform whose arm-controller integration is not explicitly verified must
   reject arm-controller, arm-follower, and arm-start requests.  It must not
   reuse another platform's controller names, frames, or command endpoint.

## Offline verification matrix

| Requirement | Evidence before hardware use |
| --- | --- |
| Path pairing and timing | unit tests with valid, malformed, and unequal paths |
| Pose-source exclusivity | launch test asserts one `/robot_pose` publisher |
| Stop behavior | synthetic stale/missing input tests observe zero commands |
| MuR profile | headless simulation launch and command-topic type check |
| Mocap adapter | recorded QTM/replay fixtures and timeout tests |
| Keyence driver | fake TCP server plus recorded profile fixtures |
| Contour correction | synthetic profiles verify sign, limit, and stale reset |
| Process control | pseudo-terminal serial tests and mocked Dynamixel adapter |

## Reproducible offline build

Use the pinned manifest in `dependencies/print_system.jazzy.repos`. The QTM and
Keyence entries must be added with their exact Jazzy-branch commits after those
local package additions have been published. Build and test in a clean overlay
before adding hardware packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths \
  src/match_additive_manufacturing_ros2 \
  src/match_mocap/mocap_toolbox_ros2 \
  src/match_hardware_utilities/keyence_profile_ros2
colcon test --base-paths \
  src/match_additive_manufacturing_ros2 \
  src/match_mocap/mocap_toolbox_ros2 \
  src/match_hardware_utilities/keyence_profile_ros2
colcon test-result --all --verbose
```
