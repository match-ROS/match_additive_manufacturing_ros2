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

## Reproducible Development Setup

Use [dependencies/print_system.jazzy.repos](dependencies/print_system.jazzy.repos)
to create the pinned ROS 2 print-system foundation.  The manifest deliberately
excludes ROS 1 reference repositories; keep those outside the colcon source tree.
The QTM and Keyence ROS 2 packages currently await publication from their mixed
source repositories, [match_mocap](https://github.com/match-ROS/match_mocap)
and [match_hardware_utilities](https://github.com/match-ROS/match_hardware_utilities).
They must be committed to named Jazzy branches and pinned in the manifest before
claiming a complete reproducible sensor-enabled workspace.

When the mixed sensor repositories are present locally, build only their ROS 2
package roots; an unqualified `colcon build` also discovers their legacy catkin
packages:

```bash
cd ~/workspaces/print2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths \
  src/match_additive_manufacturing_ros2 \
  src/match_mocap/mocap_toolbox_ros2 \
  src/match_hardware_utilities/keyence_profile_ros2
```

The system-wide topic, frame, safety, and operating-profile rules are defined in
[docs/print_system_contract.md](docs/print_system_contract.md).  New platform,
mocap, scanner, or process-control work must conform to that contract before it
is exposed through the operator GUI.

The current hardware-free build and test baseline is recorded in
[docs/offline_verification.md](docs/offline_verification.md).
The ordered development and hardware-commissioning stages are in
[docs/implementation_plan.md](docs/implementation_plan.md).
Outstanding implementation work, including the ROS 1 behaviors worth selectively
retaining in ROS 2, is tracked in [docs/open_todos.md](docs/open_todos.md).
For a MuR620 with MiR base and two UR10 arms, without Robotnik, use the staged
[MuR620 installation manual](docs/mur620_install.md).

## Development Direction

The near-term simulation target is Robotnik RB-VOGUI + UR because omnidirectional
motion is useful for early print-path following tests. Bunker + UR support should be
added later by adapting platform bringup and command interfaces while reusing generic
application nodes.

See [docs/architecture.md](docs/architecture.md) for the package ownership rules and
branch plan.
