# match_additive_manufacturing_ros2

ROS 2 application logic for additive-manufacturing path-following experiments.

This repository should stay platform-agnostic where practical. Robot descriptions,
Gazebo worlds, hardware drivers, platform-specific controllers, and vendor simulation
imports belong in platform repositories such as `bunker_manipulator`.

## Workspace Setup

The supported development environment is Ubuntu 24.04 with ROS 2 Jazzy. Install ROS 2
Jazzy first, then install the workspace tools:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-venv
```

Initialize `rosdep` once per computer if it has not been initialized already:

```bash
sudo rosdep init
rosdep update
```

Create a workspace, clone this repository, and import the pinned RB-VOGUI and UR
simulation repositories:

```bash
mkdir -p ~/wattle_daub_ros2_ws/src
cd ~/wattle_daub_ros2_ws/src
git clone https://github.com/match-ROS/match_additive_manufacturing_ros2.git

cd ~/wattle_daub_ros2_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/match_additive_manufacturing_ros2/dependencies/robotnik_rbvogui_tum.jazzy.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The `.repos` manifest downloads source repositories into `src`; it does not install
their system dependencies. `rosdep install` resolves the dependencies declared by
all packages in the workspace. Run it again after adding or changing source
repositories.

The workspace does not include site-specific hardware drivers, Vicon bridges, safety
systems, or robot hardware interfaces. For the RB-VOGUI simulator, see the
[RB-VOGUI package README](../robotnik/robotnik_rbvogui_tum/README.md). For hardware
operation and the operator GUI, see the [AM Operator GUI README](am_operator_gui/README.md).
For the generic simulation demos, see [am_bringup/README.md](am_bringup/README.md)
and the [workspace architecture](docs/architecture.md).

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
