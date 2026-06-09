# match_additive_manufacturing_ros2

ROS 2 application logic for additive-manufacturing path-following experiments.

This repository should stay platform-agnostic where practical. Robot descriptions,
Gazebo worlds, hardware drivers, platform-specific controllers, and vendor simulation
imports belong in platform repositories such as `bunker_manipulator`.

## Current Packages

- `parse_paths`: Generates simple test/reference paths as `nav_msgs/msg/Path`.
- `base_trajectory_follower`: Generic simple mobile-base path follower for simulation.
- `move_to_path_idx`: One-shot mobile-base motion to a selected path index from an
  externally supplied robot pose.
- `ur_trajectory_follower`: UR/TCP path-following and twist-composition utilities.
- `am_bringup`: Demo launch/config glue connecting generic AM nodes to simulator topics.

## Core Topic Assumptions

External simulation or platform bringup provides pose feedback:

- Mobile base pose: `geometry_msgs/msg/PoseStamped`, default `/robot_pose`.
- TCP/nozzle pose: `geometry_msgs/msg/PoseStamped`, default `/current_tcp_pose`.

Application packages do not estimate these poses. They consume pose topics, publish
reference paths, publish velocity commands, and expose gains/limits/tolerances as
parameters.

## Development Direction

The near-term simulation target is Robotnik RB-VOGUI + UR because omnidirectional
motion is useful for early print-path following tests. Bunker + UR support should be
added later by adapting platform bringup and command interfaces while reusing generic
application nodes.

See [docs/architecture.md](docs/architecture.md) for the package ownership rules and
branch plan.
