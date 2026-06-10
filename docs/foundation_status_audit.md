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
  generator to the generic base follower using RB-VOGUI topic defaults and the
  validated `robotnik_simple` world frame. The fixed demo path is published once
  with transient-local QoS.
- `bunker_description/bunker_path_following_demo.launch.py` connects the same
  generic nodes with Bunker diff-drive defaults and can optionally include the
  Bunker simulator.

RB-VOGUI runtime contracts:

- `rbvogui_ur_sim_setup/rbvogui_ur_standard_control.launch.py` starts a local
  standard-controller workaround for the RB-VOGUI + UR model.
- `/robot_pose` publishes `geometry_msgs/msg/PoseStamped` in frame
  `robotnik_simple`.
- `/current_tcp_pose` publishes `geometry_msgs/msg/PoseStamped` for
  `robot_arm_tool0` in frame `robotnik_simple`.
- The standard-controller bridge accepts `geometry_msgs/msg/Twist` on
  `/robot/robotnik_base_control/cmd_vel_unstamped`.
- Lateral x/y commands were runtime-validated through the platform-side swerve
  controller.
- The RB-VOGUI path-following demo drove the robot along the default 2 m line:
  `/base_path` published in `robotnik_simple`, `/robot_pose` reached about
  `x=1.93`, and the command topic returned to zero after the follower reported
  `goal reached`.

Bunker runtime contracts:

- `bunker_description/spawn_with_controllers.launch.py` starts Gazebo, the BunkUR
  model, `joint_state_broadcaster`, `diff_drive_controller`, and the UR controllers.
- `diff_drive_controller` is active and subscribes to
  `geometry_msgs/msg/TwistStamped` on `/diff_drive_controller/cmd_vel`.
- `/robot_pose` publishes `geometry_msgs/msg/PoseStamped` in frame `map`.
- TF resolves `map -> base_footprint` and `map -> ur_tool0`.
- A short stamped forward command moved `/robot_pose.x` from about `-0.0008` to
  `0.3427`.
- The full Bunker path-following demo moved along the default 1 m line:
  `/bunker_base_path` published in `map`, `/robot_pose.x` reached about `0.94`,
  and `/diff_drive_controller/cmd_vel` returned to zero after the follower reported
  `goal reached`.

Nozzle/TCP monitoring:

- `print_path_monitoring/nozzle_pose_monitor` compares an externally supplied TCP
  pose with either a reference pose or a path plus live or fixed index.
- It publishes monitoring values only. It does not command compensation.
- `am_bringup/bunker_tcp_monitoring_demo.launch.py` wires Bunker TF
  `map -> ur_tool0` through `/current_tcp_pose`, starts the paired arm/base path
  publisher, and monitors the TCP against `/ur_path_transformed`.
- The Bunker TCP monitoring demo was runtime-validated headless: `/current_tcp_pose`
  published in frame `map`, `/ur_path_transformed` published 50 arm poses in
  frame `map`, `/nozzle_position_error_norm` published about `1.3e-09`, and
  `/nozzle_yaw_error` published about `0.2525`.

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
ros2 launch am_bringup bunker_tcp_monitoring_demo.launch.py --show-args
```

Result: all three launch files loaded and exposed the expected parameters.

Bunker TCP monitoring runtime check:

```bash
ros2 launch am_bringup bunker_tcp_monitoring_demo.launch.py \
  launch_sim:=true headless:=true launch_rviz:=false
ros2 topic echo /current_tcp_pose --once
ros2 topic echo /ur_path_transformed --once
ros2 topic echo /nozzle_position_error_norm --once
ros2 topic echo /nozzle_yaw_error --once
```

Result: the launch started the headless Bunker simulator, published the TCP pose,
published the generated arm path, and produced nozzle/TCP monitoring outputs.

## Completion Notes

The foundation is locally buildable and the generic code is covered by focused unit
tests. RB-VOGUI pose, TCP pose, base velocity, and the default path-following demo
are runtime-validated. Bunker pose, base command, tool TF, and the default
path-following demo are also runtime-validated. Bunker TCP pose publication and
monitoring against a generated arm print path are now runtime-validated too.

## Current Recommendation

Treat the repository state as a clean local foundation and the next milestone as
arm-path execution or higher-level coordination, not additional base-controller
complexity.

Run the RB-VOGUI simulator first and record the exact pose and command contracts. If
those contracts match the current defaults, the existing `am_bringup` demo is the
first integration test. If they differ, add minimal platform-side bridge nodes or
launch remappings outside generic AM packages.

For Bunker, keep using the generic base follower with `linear.y` disabled and stamped
commands on `/diff_drive_controller/cmd_vel`. Use
`am_bringup/bunker_tcp_monitoring_demo.launch.py` as the smoke test for Bunker
TCP/nozzle monitoring before adding arm motion or compensation.
