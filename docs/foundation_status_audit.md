# ROS 2 Print-Path Foundation Status Audit

This audit records the current state of the simple simulation-based ROS 2
print-path-following foundation across `match_additive_manufacturing_ros2` and
`bunker_manipulator`.

## Proven Locally

Package structure and separation:

- `match_additive_manufacturing_ros2` owns generic additive-manufacturing logic:
  path generation, simple base following, nozzle monitoring, start-pose movement,
  UR path utilities, and demo bringup.
- `bunker_manipulator` owns platform setup and platform-specific overlays:
  RB-VOGUI/UR dependency manifests, Bunker simulation notes, Bunker controller
  notes, and Bunker follower launch/config defaults.
- Generic AM packages do not contain Robotnik- or Bunker-specific robot
  descriptions.

Test/reference paths:

- `parse_paths/test_path_generator` publishes `nav_msgs/msg/Path`.
- Supported shape helpers include line, rectangle, circle, and waypoint paths.
- Specialized publishers for the UR arm and paired arm/base paths live in
  `parse_paths`, not `ur_trajectory_follower`.

Base follower:

- `base_trajectory_follower/simple_base_follower` consumes a reference path and
  external base pose.
- It publishes `geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped`.
- It supports x/y/yaw proportional commands, lookahead target selection, velocity
  limits, goal tolerances, stale-pose timeout, and zero velocity on missing inputs.
- RB-VOGUI defaults allow lateral velocity.
- Bunker defaults set `max_vy: 0.0` and `kp_y: 0.0` for differential drive.

Demo launches:

- `am_bringup/rbvogui_path_following_demo.launch.py` connects the test path
  generator to the generic base follower using RB-VOGUI topic defaults.
- `bunker_description/bunker_path_following_demo.launch.py` connects the same
  generic nodes with Bunker diff-drive defaults and can optionally include the
  Bunker simulator.

Nozzle/TCP monitoring:

- `print_path_monitoring/nozzle_pose_monitor` compares an externally supplied TCP
  pose with either a reference pose or a path/index pair.
- It publishes monitoring values only. It does not command compensation.

ROS 1 MiR follower:

- `docs/ros1_mir_trajectory_follower_analysis.md` documents reusable concepts,
  risky complexity, migration tasks, and the recommendation not to blindly port the
  ROS 1 follower.

Local verification performed:

```bash
source install/setup.bash
python3 -m pytest -q \
  src/match_additive_manufacturing_ros2/parse_paths/test/test_test_path_shapes.py \
  src/match_additive_manufacturing_ros2/base_trajectory_follower/test/test_controller.py \
  src/match_additive_manufacturing_ros2/print_path_monitoring/test/test_error_metrics.py
```

Result: 11 passed. The only warnings were from pytest being unable to write cache
files in the read-only workspace root.

```bash
colcon build --symlink-install --packages-select \
  parse_paths base_trajectory_follower print_path_monitoring am_bringup \
  move_to_path_idx ur_trajectory_follower rbvogui_ur_sim_setup \
  bunker_description controllers_ros2
```

Result: all 9 selected packages built successfully.

Launch argument checks:

```bash
ros2 launch am_bringup rbvogui_path_following_demo.launch.py --show-args
ros2 launch bunker_description bunker_path_following_demo.launch.py --show-args
```

Result: both launch files loaded and exposed the expected parameters.

## Not Yet Proven

The foundation is locally buildable and the generic code is covered by focused unit
tests, but the full objective is not completely proven until these runtime checks are
performed in a real simulator session:

- Import Robotnik RB-VOGUI dependencies from
  `bunker_manipulator/rbvogui_ur_sim_setup/dependencies/rbvogui_simulation.jazzy.repos`.
- Launch RB-VOGUI simulation and confirm whether `rbvogui_plus` includes the needed
  UR arm model or only provides the mobile-base baseline.
- Confirm exact RB-VOGUI base pose source and whether a platform-side bridge is
  needed to publish `/robot_pose` as `geometry_msgs/msg/PoseStamped`.
- Confirm exact RB-VOGUI TCP/nozzle pose source and whether a bridge is needed to
  publish `/current_tcp_pose`.
- Confirm lateral velocity on `/robot/robotnik_base_control/cmd_vel_unstamped`.
- Launch Bunker simulation and confirm the active diff-drive command topic and type:
  `/diff_drive_controller/cmd_vel_unstamped` as `Twist`, or another topic/type.
- Confirm Bunker `/robot_pose` frame, update rate, and TF consistency.
- Confirm UR tool frame for Bunker, likely `ur_tool0`, before using TCP monitoring
  against a print path.

## Current Recommendation

Treat the repository state as a clean local foundation and the next milestone as
runtime validation, not additional controller complexity.

Run the RB-VOGUI simulator first and record the exact pose and command contracts. If
those contracts match the current defaults, the existing `am_bringup` demo is the
first integration test. If they differ, add minimal platform-side bridge nodes or
launch remappings outside generic AM packages.

For Bunker, keep using the generic base follower with `linear.y` disabled until a
runtime test proves a different interface is required.
