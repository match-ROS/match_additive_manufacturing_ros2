# base_trajectory_follower

Generic ROS 2 mobile-base path follower for early simulation tests.

The first node, `simple_base_follower`, is intentionally small and platform-light:

- subscribes to `nav_msgs/msg/Path`
- subscribes to an external base pose topic
- publishes `geometry_msgs/msg/Twist` by default
- can publish `geometry_msgs/msg/TwistStamped` when `output_stamped:=true`
- supports x, y, and yaw velocity for omnidirectional bases such as RB-VOGUI
- publishes zero velocity if no path, no pose, stale pose, or goal reached

It does not estimate pose, run localization, perform obstacle avoidance, or compensate
TCP/base motion. Those concerns belong in later branches.

## Run

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py
```

RB-VOGUI-style unstamped command topic:

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py \
  path_topic:=/base_path \
  robot_pose_topic:=/robot_pose \
  cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel_unstamped \
  output_stamped:=false
```

Stamped command fallback:

```bash
ros2 launch base_trajectory_follower simple_base_follower.launch.py \
  cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel \
  output_stamped:=true
```

## Important Parameters

- `lookahead_distance`: distance used to pick a forward target on the path.
- `stale_pose_timeout`: max allowed age of the latest pose before commanding zero.
- `kp_x`, `kp_y`, `kp_yaw`: proportional gains in robot frame.
- `max_vx`, `max_vy`, `max_wz`: command limits.
- `xy_goal_tolerance`, `yaw_goal_tolerance`: final pose tolerances.
- `allow_reverse`: if false, negative x velocity is clamped to zero.

## Expected Topic Contract

Base pose:

- `geometry_msgs/msg/PoseStamped`, default `/robot_pose`
- `geometry_msgs/msg/Pose` is also supported with `robot_pose_type:=pose`

Reference path:

- `nav_msgs/msg/Path`, default `/base_path`

Command:

- `geometry_msgs/msg/Twist`, default `/robot/robotnik_base_control/cmd_vel_unstamped`
- or `geometry_msgs/msg/TwistStamped` with `output_stamped:=true`
