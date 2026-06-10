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
