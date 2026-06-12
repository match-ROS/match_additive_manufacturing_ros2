# ROS 1 MiR Trajectory Follower Analysis

This note reviews the ROS Noetic `match_additive_manufacturing/mir_trajectory_follower`
package as input for the ROS 2 print-path-following foundation. It is an analysis,
not a porting plan for the full ROS 1 implementation.

## Scope

The ROS 1 package was built for a MiR/MUR-style mobile manipulator following a print
trajectory together with a UR arm. The current ROS 2 target is different:

- first simulation platform: RB-VOGUI + UR, because lateral base motion is useful
  for early tests;
- later platform: Bunker + UR, likely differential-drive only;
- pose inputs come from external simulation topics;
- early control should stay simple, observable, and testable.

Because of that, only the reusable control concepts should influence the ROS 2
foundation. The old package should not be copied into ROS 2 as-is.

## ROS 1 Package Summary

The active launch file starts:

- `mir_trajectory_follower.py`
- `mir_index_offset_applier.py`

`path_index_modifier.py` exists but is commented out in the launch file.
`rl_pure_pursuit_offset.py` is a placeholder correction node and is not part of the
normal launch.

The main follower consumes:

- `nav_msgs/Path` for the MiR path;
- `geometry_msgs/Pose` for the current base pose;
- `std_msgs/Int32` for a UR/trajectory index;
- `Float32MultiArray` timestamps;
- a path-velocity topic;
- velocity override and start-condition topics.

It publishes:

- `geometry_msgs/Twist` on the base command topic;
- optional target/actual poses;
- layer progress;
- completion status;
- debug TF frames for current, target, and lookahead points.

## Reusable Ideas

The following concepts are worth keeping, but should be implemented with small ROS 2
components and unit-tested math helpers:

- Standard topic contract: reference path, external base pose, and `Twist` command.
- Configurable topics, gains, limits, tolerances, and control rate.
- Lookahead or target-index selection from a `nav_msgs/Path`.
- Explicit start gating when a print process needs synchronized startup.
- Zero velocity on missing/stale inputs or when path following is inactive.
- Linear and angular velocity saturation.
- Completion reporting when the path is finished.
- Optional diagnostic publications for current target, actual pose, and path errors.
- Error thresholds that can stop motion when tracking becomes unsafe.

Most of these concepts already fit the ROS 2 `base_trajectory_follower` package. The
remaining ones should be added only when simulation tests show they are needed.

## Complexity To Avoid For Now

The ROS 1 implementation has several features that are tightly coupled to the old
MiR/MUR print stack:

- Timestamp-vector based feedforward velocity generation.
- `mir_path_velocity` and smoothed per-point path velocity.
- Coupling between UR path index and MiR path index.
- Time-warping of base index based on path timestamps and local speed.
- Offset-vector based index modification from `mir_index_offset`.
- Layer-progress publishing based on global `/points_per_layer`.
- Velocity override coupling across multiple nodes.
- Debug TF broadcasting from inside the controller.
- ROS 1 topic names and non-stamped `geometry_msgs/Pose` pose input.
- A placeholder RL correction node with no production inference implementation.

These are not good defaults for the clean ROS 2 foundation. They add behavior that is
hard to validate before the simple simulation loop is working.

## Migration Risks Found

If parts of the old package are ever ported, these issues should be addressed first:

- `mir_trajectory_follower.py` blocks during initialization with `wait_for_message`
  calls, then enters its control loop from the constructor. In ROS 2 this should be
  replaced with subscriptions, timers, and explicit readiness state.
- The main follower waits for timestamp data even though simple simulation paths can
  be followed without timestamps.
- `geometry_msgs/Pose` input loses frame and timestamp information. ROS 2 nodes
  should prefer `PoseStamped`.
- The controller mixes control, path indexing, diagnostics, layer progress, and TF
  debug output in one node.
- `mir_index_offset_applier.py` subscribes to a `Bool` start signal but annotates the
  callback as `Int32`.
- `mir_index_offset_applier.py` indexes `offset_vec[ur_idx + int(offset_middle)]`
  without proving the shifted index is in range.
- The main follower is differential-drive oriented: it primarily publishes
  `linear.x` and `angular.z`. RB-VOGUI tests may need `linear.y`; Bunker tests should
  keep `linear.y` disabled.
- Error thresholds, index monotonicity, lookahead selection, and velocity saturation
  currently have no focused unit tests.

## Recommended ROS 2 Direction

Do not port `mir_trajectory_follower.py` wholesale.

Use the existing ROS 2 `base_trajectory_follower/simple_base_follower` as the first
mobile-base controller:

- It already consumes a path and external pose.
- It already publishes configurable `Twist` or `TwistStamped` commands.
- It keeps x/y/yaw behavior parameterized for RB-VOGUI.
- It can be constrained later for Bunker by setting lateral velocity to zero.
- It has isolated math helpers and tests.

Only reintroduce ROS 1 features when a specific simulation test proves they are
needed. The likely order is:

1. Add richer diagnostics for target pose, actual pose, and tracking error.
2. Add explicit completion/status publication.
3. Add optional start-condition gating.
4. Add optional path-index synchronization with the UR arm.
5. Add feedforward/timestamp behavior only if proportional path following is not
   sufficient for the real print timing requirement.

This keeps the ROS 2 foundation simple while leaving a clear path to recover useful
MiR-era behavior later.
