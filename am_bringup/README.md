# am_bringup

Launch/config glue for additive-manufacturing simulation demos.

This package intentionally does not own robot descriptions or simulator bringup. Start
the platform simulator from `bunker_manipulator` or an imported vendor workspace, then
use these launches to connect generic AM nodes to the simulator topics.

## RB-VOGUI Path Following Demo

Prerequisites:

- Robotnik RB-VOGUI simulation running.
- External base pose topic available as `geometry_msgs/msg/PoseStamped`.
- Base velocity command topic available as `geometry_msgs/msg/Twist` or
  `geometry_msgs/msg/TwistStamped`.

Default AM topic assumptions:

- Base pose: `/robot_pose`
- Path frame: `robotnik_simple`
- Base path: `/base_path`
- Base command: `/robot/robotnik_base_control/cmd_vel_unstamped`

Run:

```bash
ros2 launch am_bringup rbvogui_path_following_demo.launch.py
```

If the Robotnik controller requires stamped velocity commands:

```bash
ros2 launch am_bringup rbvogui_path_following_demo.launch.py \
  cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel \
  output_stamped:=true
```

Try a circle path:

```bash
ros2 launch am_bringup rbvogui_path_following_demo.launch.py path_type:=circle
```

If a different simulator world frame is used:

```bash
ros2 launch am_bringup rbvogui_path_following_demo.launch.py \
  path_frame:=<world_frame>
```

## RB-VOGUI Paired Base/Arm Demo

The paired Robotnik demos use one shared `/path_index` for both the base path and
the arm path. This keeps the base and UR reference trajectories synchronized by
waypoint number: index `i` is the base target and arm target for the same step.

The demo can either generate temporary test paths or consume externally supplied
`/base_path` and `/ur_path_transformed` topics. For external paths, run with
`generate_test_paths:=false`; the base mover, shared index, base follower, and arm
controllers wait for those topic messages.

In the generated test mode, `move_to_start_pose` first sends a UR one-shot joint pose
on `/robot/joint_trajectory_controller/joint_trajectory`. A temporary prestart path
moves the RB-VOGUI to a base start area. When the base reaches that point,
`/start_pose_reached` triggers generation of the final paired path from the current
base pose and current TCP pose. The final base path starts at the current base pose,
and the final arm path starts at the current TCP pose.

The shared index publisher, base follower, and arm controller wait for your manual
`/start_condition` signal before following the final trajectory.

The paired paths are static and publish once by default with transient-local QoS.
Use `publish_once:=false` only when debugging repeated path publication.

Generated test trajectories are exported by default after the final path is created.
The default export folder is:

```bash
match_additive_manufacturing_ros2/components/robotnik_paired_demo
```

It contains `base_path.json`, `arm_path.json`, and `normal_vector.json`.

Generate paired paths, increment the shared index, and drive only the RB-VOGUI base:

```bash
ros2 launch am_bringup rbvogui_paired_base_only_demo.launch.py
```

Start the validated Robotnik simulation as part of the base-only demo:

```bash
ros2 launch am_bringup rbvogui_paired_base_only_demo.launch.py launch_sim:=true
```

Run the base plus UR control-node wiring:

```bash
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py
```

Use externally supplied path topics instead of the test generator:

```bash
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py \
  generate_test_paths:=false
```

Replay the previously exported generated trajectories on the next launch:

```bash
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py \
  use_exported_trajectories:=true
```

In this mode the launch publishes the exported `/base_path` and
`/ur_path_transformed` once with transient-local QoS, then the usual start-pose and
manual `/start_condition` flow is used.

Disable the pre-roll only when another node has already positioned the robot:

```bash
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py \
  move_to_start_pose:=false \
  wait_for_start_condition:=false
```

After the start pose has been reached, begin trajectory following manually:

```bash
ros2 topic echo /start_pose_reached --once
ros2 topic pub --once /start_condition std_msgs/msg/Bool "{data: true}"
```

The base+arm launch reuses the UR sideways control stack but disables its internal
path publisher and internal index publisher. The paired Robotnik path publisher owns
both paths, and the single `shared_path_index` node publishes `/path_index`.

The arm direction controller defaults to `direction_control_mode:=speed_orthogonal`.
It uses the timestamp-derived path speed as tangent feed-forward and adds bounded
cross-track correction in the plane orthogonal to the spray axis. Tune with:

```bash
ros2 launch am_bringup rbvogui_paired_base_arm_demo.launch.py \
  orthogonal_kp:=1.0 \
  orthogonal_max_velocity:=0.1
```

The Robotnik standard-control simulation exposes the UR
`joint_trajectory_controller` for the one-shot start pose and
`arm_forward_velocity_controller` for path following. The base+arm demo starts the
J-PARSE velocity bridge by default, waits for the start-pose delay, then switches
from the trajectory controller to the velocity controller.

Useful checks:

```bash
ros2 topic echo /path_index --once
ros2 topic echo /start_condition --once
ros2 topic echo /start_pose_reached --once
ros2 topic echo /base_path --once
ros2 topic echo /ur_path_transformed --once
ros2 topic echo /robot/robotnik_base_control/cmd_vel_unstamped
ros2 topic echo /robot/joint_trajectory_controller/joint_trajectory --once
ros2 topic hz /current_tcp_pose
ros2 topic echo /ur_error_world --once
ros2 topic echo /ur_twist_world --once
ros2 topic echo /jparse_velocity_controller_ur/twist_cmd --once
ros2 topic echo /robot/arm_forward_velocity_controller/commands --once
ros2 control list_controllers --controller-manager /robot/controller_manager
```

## Bunker TCP Monitoring Demo

Prerequisites:

- Bunker simulation running from `bunker_description`, or launch it through this demo.
- TF resolves `map -> ur_tool0`.
- `/robot_pose` publishes `geometry_msgs/msg/PoseStamped` in frame `map`.

Default AM topic assumptions:

- TCP/nozzle pose output: `/current_tcp_pose`
- TCP transform: `map <- ur_tool0`
- Arm path: `/ur_path_transformed`
- Base path: `/bunker_base_path`
- Monitor outputs: `/nozzle_position_error`, `/nozzle_position_error_norm`,
  `/nozzle_yaw_error`

Attach to an already-running Bunker simulation:

```bash
ros2 launch am_bringup bunker_tcp_monitoring_demo.launch.py
```

Start the Bunker simulation headless as part of the demo:

```bash
ros2 launch am_bringup bunker_tcp_monitoring_demo.launch.py launch_sim:=true
```

By default, the monitor compares `/current_tcp_pose` against index `0` of the
generated arm path. To use a live path-index source instead:

```bash
ros2 launch am_bringup bunker_tcp_monitoring_demo.launch.py fixed_path_index:=-1
```

## Test Procedure

In separate terminals:

```bash
ros2 topic echo /robot_pose --once
ros2 topic info /robot/robotnik_base_control/cmd_vel_unstamped -v
ros2 topic echo /base_path --once
ros2 topic echo /robot/robotnik_base_control/cmd_vel_unstamped
```

Expected behavior:

- `test_path_generator` publishes `/base_path`.
- `/base_path` and `/robot_pose` are in the same world frame.
- The demo path is published once with transient-local QoS.
- `simple_base_follower` publishes zero velocity until `/robot_pose` is fresh.
- Once pose and path are available, the follower publishes x/y/yaw velocity commands.
- If pose stops updating longer than `stale_pose_timeout`, the follower commands zero.
