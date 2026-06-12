# match_additive_manufacturing_ros2

ROS 2 application logic for additive-manufacturing path-following experiments.

This repository should stay platform-agnostic where practical. Robot descriptions,
Gazebo worlds, hardware drivers, platform-specific controllers, and vendor simulation
imports belong in platform repositories such as `bunker_manipulator`.

## Current Packages

- `parse_paths`: Generates simple test/reference paths as `nav_msgs/msg/Path`.
  It also contains specialized paired path publishers, including the Robotnik
  base/UR arm sideways plus 45 degree demo paths.
- `base_trajectory_follower`: Generic simple mobile-base path follower for simulation.
- `move_to_path_idx`: One-shot mobile-base motion to a selected path index from an
  externally supplied robot pose.
- `ur_trajectory_follower`: UR/TCP path-following and twist-composition utilities.
- `am_bringup`: Demo launch/config glue connecting generic AM nodes to simulator topics.
- `print_path_monitoring`: Monitoring-only nozzle/TCP pose error diagnostics.

## Core Topic Assumptions

External simulation or platform bringup provides pose feedback:

- Mobile base pose: `geometry_msgs/msg/PoseStamped`, default `/robot_pose`.
- TCP/nozzle pose: `geometry_msgs/msg/PoseStamped`, default `/current_tcp_pose`.

Application packages do not estimate these poses. They consume pose topics, publish
reference paths, publish velocity commands, and expose gains/limits/tolerances as
parameters.

## Robotnik Paired Base/Arm Demo

The current integrated demo is the RB-VOGUI base plus UR arm paired-path flow. It
generates base and arm paths with the same number of waypoints, drives the base to
the first base waypoint, then uses one shared `/path_index` to advance both paths.

```bash
colcon build --symlink-install --packages-select \
  parse_paths move_to_path_idx base_trajectory_follower ur_trajectory_follower am_bringup
source install/setup.bash
ros2 launch am_bringup rbvogui_paired_base_only_demo.launch.py launch_sim:=true
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py launch_sim:=true
```

See [am_bringup/README.md](am_bringup/README.md) for topic checks, the start-pose
handoff, and the current Robotnik UR velocity-controller caveat.

## Development Direction

The near-term simulation target is Robotnik RB-VOGUI + UR because omnidirectional
motion is useful for early print-path following tests. Bunker + UR support should be
added later by adapting platform bringup and command interfaces while reusing generic
application nodes.

See [docs/architecture.md](docs/architecture.md) for the package ownership rules and
branch plan.
