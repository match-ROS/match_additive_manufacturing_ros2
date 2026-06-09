# ROS 2 Print-Path Following Architecture

This document records the minimal package structure for the ROS 2 foundation. It is
intentionally small: the goal is to make simulation path-following testable before
porting complex ROS 1 behavior or adding MoveIt.

## Repository Boundaries

### `match_additive_manufacturing_ros2`

Owns additive-manufacturing application logic:

- test and print-path generation
- high-level path-following coordination
- generic mobile-base followers
- UR/TCP path-following nodes that consume external pose topics
- monitoring and diagnostics for path/nozzle/base error
- demo launch files that connect generic nodes to externally provided simulation topics

Avoid putting robot descriptions, Gazebo worlds, vendor drivers, or platform-specific
controller implementation here.

### `bunker_manipulator`

Owns platform and simulation bringup:

- Bunker/BunkUR robot descriptions
- Gazebo launch files and ros_gz bridges
- platform controller YAML
- UR velocity-controller launch integration
- later Bunker-specific MoveIt config or hardware interfaces
- external/vendor robot simulation sources when needed

Current evidence:

- `bunker_description` launches Bunker/BunkUR simulation and publishes `/robot_pose`
  from Gazebo model pose.
- `controllers_ros2` provides the KDL-based UR velocity controller.
- `bunker_sim/src/match_mobile_robotics_jazzy` already contains mobile-robot vendor
  packages for MiR/MUR-style simulation references.

### `match_additive_manufacturing`

ROS Noetic reference implementation only. Use it to understand prior behavior, but do
not blindly port nodes. The ROS 2 foundation should stay simpler until a behavior is
needed and tested in simulation.

## Package Structure

### Existing ROS 2 Packages

`parse_paths`

- Publishes simple `nav_msgs/msg/Path` test paths.
- Current executables:
  - `publish_sideways_arm_test_path`
  - `publish_front_side_arm_base_paths`
- Should grow into the general test-print-path generator package for line,
  rectangle, circle, and waypoint paths.

`move_to_path_idx`

- One-shot base motion helper.
- Consumes a path and external base pose.
- Publishes base velocity until the selected path index is reached and oriented.
- Useful for start-pose setup, not continuous path following.

`base_trajectory_follower`

- Generic simple mobile-base follower package for RB-VOGUI first, then Bunker.
- Subscribes to `nav_msgs/msg/Path` and external base pose.
- Publishes configurable `Twist` or `TwistStamped` base velocity commands.
- Includes proportional x/y/yaw command generation, lookahead target selection,
  velocity limits, final tolerances, and stale-pose zero-command behavior.

`ur_trajectory_follower`

- UR/TCP path-following and twist utilities.
- Includes current TCP pose from TF, path-index publishing, direction/orientation
  controllers, PID twist control, twist combining, and twist frame transforms.
- Should not own path generation or platform simulation setup.

`am_bringup`

- Demo launch and config glue for simulation experiments.
- Connects generic AM nodes to externally provided platform pose and command topics.
- Does not own robot descriptions, Gazebo worlds, or vendor bringup.

### Planned Packages

`print_path_monitoring`

- Monitoring only.
- Consumes external TCP/nozzle pose and optional reference path/pose.
- Publishes/logs error values.
- No control, compensation, or pose estimation.

## Topic Contracts

Use standard messages unless a custom message is clearly justified.

Base path:

- Type: `nav_msgs/msg/Path`
- Example topic: `/base_path` or `/mir_path_transformed`

Arm/TCP path:

- Type: `nav_msgs/msg/Path`
- Example topic: `/ur_path_transformed`

Base pose:

- Type: `geometry_msgs/msg/PoseStamped`
- Example topic: `/robot_pose`
- Provided externally by simulation, mocap, localization, or platform bringup.

TCP/nozzle pose:

- Type: `geometry_msgs/msg/PoseStamped`
- Example topic: `/current_tcp_pose`
- Provided externally by simulation/TF bridge or hardware pose publisher.

Base velocity command:

- Prefer `geometry_msgs/msg/Twist` for generic nodes.
- Allow `geometry_msgs/msg/TwistStamped` or platform-specific command adapters where
  required by controllers.
- RB-VOGUI support should allow x, y, and yaw velocity when the platform command
  interface supports it.

## Branch Plan

1. `feature/ros2-package-structure`
   - Document package ownership, topic assumptions, and platform separation.
   - No new controllers.
2. `feature/test-print-path-generator`
   - Generalize `parse_paths` for line, rectangle, circle, and waypoint paths.
   - Add config, launch, and README.
3. `feature/rbvogui-ur-simulation-setup`
   - Integrate the minimal public RB-VOGUI + UR simulation setup.
   - Document startup and pose/command topics.
4. `feature/simple-rbvogui-trajectory-follower`
   - Add the first simple base follower using external pose and `nav_msgs/msg/Path`.
5. `feature/simulation-path-following-demo`
   - Launch path generator, pose inputs, and follower together.
6. `feature/nozzle-pose-monitoring`
   - Add monitoring-only TCP/nozzle error reporting.
7. `feature/bunker-platform-adaptation-plan`
   - Document missing Bunker + UR simulation interfaces and TODOs.
8. `feature/bunker-simple-trajectory-follower`
   - Adapt the generic base follower to Bunker command/kinematic constraints.
9. `feature/port-mir-trajectory-follower-analysis`
   - Analyze ROS 1 `mir_trajectory_follower` and recommend port/simplify/rewrite.

## Decisions For Now

- Do not start with MoveIt. Add it only when a branch requires planning or collision
  checking.
- Do not estimate base/TCP poses in application packages.
- Keep platform-specific code out of generic AM packages.
- Keep each branch small enough that the affected packages build.
