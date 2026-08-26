# AM Operator GUI — Hardware Operation

## Local web GUI (preview, runs alongside the PyQt reference GUI)

The web interface listens only on `127.0.0.1:8000`, opens the browser automatically,
and uses the same persisted configuration and process manager as the reference GUI.
It is intentionally local because it can start robot processes and issue motion
commands.

```bash
cd ~/wattle_daub_ros2_ws
colcon build --packages-select am_operator_gui --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
src/match_additive_manufacturing_ros2/am_operator_gui/scripts/start_web_gui.sh
```

On its first run, the script creates an isolated `.web-venv` next to the package
and installs FastAPI, Uvicorn, and Jinja2 there; no system-wide `pip` installation
is needed. (On Ubuntu, install `python3-venv` once if it is not already present.)

For a clean local setup, use
`config/operator_gui_config.example.json` as the starting point for
`config/operator_gui_config.json` and select the trajectory directory in either
interface. The example deliberately contains no workstation-specific absolute path.

### Browser test with Playwright

The browser test starts the actual start script on a free local port, opens
Chromium, verifies key controls and setting persistence, and checks the mobile
layout. It uses a temporary configuration and does not command motion.

```bash
cd ~/wattle_daub_ros2_ws/src/match_additive_manufacturing_ros2/am_operator_gui
.web-venv/bin/python -m pip install -r requirements-web-test.txt
.web-venv/bin/python -m playwright install chromium
.web-venv/bin/python -m pytest -q test/test_web_e2e.py
```

For a non-default local port or to suppress automatic browser opening, set
`AM_OPERATOR_WEB_PORT` and `AM_OPERATOR_WEB_NO_BROWSER=1` before invoking the
start script.

The existing PyQt GUI remains available through `ros2 launch am_operator_gui
am_operator_gui.launch.py`. It now uses the same toolkit-neutral configuration,
process registry, ROS bridge, and command service as the web interface. Do not run
both interfaces as active controllers at the same time: they share ROS topics and
can manage the same processes.

This guide describes how to use `am_operator_gui` with a real mobile base, UR arm,
and Vicon tracking system. The GUI starts and supervises the AM path publisher,
pose adapters, controller stack, path-index publisher, and base/arm followers. It
does **not** start the vendor base driver, UR driver, Vicon bridge, or safety system.
Bring those up and validate them before commanding motion.

> **Safety:** Test first with the robot lifted/clear of people, a conservative speed,
> and a working hardware emergency stop. `Stop Following` publishes zero base and
> arm twist commands, but it is not a replacement for a safety-rated stop.

## 1. Install the workspace and dependencies

The supported target is Ubuntu 24.04 with ROS 2 Jazzy. Create a workspace and put
this repository in its `src` directory. Substitute the repository URL used by your
team.

```bash
mkdir -p ~/wattle_daub_ros2_ws/src
cd ~/wattle_daub_ros2_ws/src
git clone https://github.com/match-ROS/match_additive_manufacturing_ros2.git
cd ..

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

For the RB-VOGUI dependency set, import the pinned repositories included with this
project:

```bash
cd ~/wattle_daub_ros2_ws
vcs import src < src/match_additive_manufacturing_ros2/dependencies/robotnik_rbvogui_tum.jazzy.repos
rosdep install --from-paths src --ignore-src -r -y
```

The hardware-specific packages are site dependent and must also be present:

- vendor base driver and its velocity-command interface;
- UR ROS 2 driver/robot description and the configured `ros2_control` hardware
  interface;
- the local `am_jparse_controller` package providing the J-PARSE chain used here;
- a Vicon ROS bridge publishing the required `PoseStamped` streams; and
- `python3-pyqt5`, `tf2_ros`, `tf2_geometry_msgs`, `tf_transformations`, and
  NumPy/SciPy (normally installed by `rosdep`).

Build and source the workspace:

```bash
cd ~/wattle_daub_ros2_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 2. Start the hardware and validate its ROS interface

Start the base driver, UR driver/robot description, controller manager, and Vicon
bridge using the deployment-specific procedures. Then verify that the selected GUI
platform profile matches the actual interface. The defaults are intended for
Robotnik and Bunker, but they are not a hardware autodiscovery mechanism.

| Function | Robotnik default | Bunker default | Required type / condition |
| --- | --- | --- | --- |
| Base pose output | `/robot_pose` | `/robot_pose` | `geometry_msgs/msg/PoseStamped`, fresh, in the path frame |
| Base odometry input | `/robot/robotnik_base_control/odom` | `/odom` | `nav_msgs/msg/Odometry`, only for odometry pose mode |
| Base command | `/robot/robotnik_base_control/cmd_vel_unstamped` | `/diff_drive_controller/cmd_vel` | `Twist` for Robotnik, `TwistStamped` for Bunker |
| Base frame | `base_link` | `base_footprint` | TF frame configured in the GUI |
| Robot TF root | `odom` | `odom` | TF connected to the robot base |
| Arm joint states | `/robot/joint_states` | deployment-specific | `sensor_msgs/msg/JointState` |
| Arm velocity command | `/robot/arm/forward_velocity_controller/commands` | same default | Controller accepts the configured joint command |
| Arm controller manager | `/robot/arm/controller_manager` | same default | Controller manager is reachable |
| Vicon base marker | `/vicon/Base_RB/Base_RB` | configurable | `geometry_msgs/msg/PoseStamped` |
| Vicon tool marker | `/vicon/Tool_Flange/Tool_Flange` | configurable bridge input | `geometry_msgs/msg/PoseStamped` |

The hardware arm velocity controller must be named `forward_velocity_controller`.
The GUI switches away from `joint_trajectory_controller` before velocity tracking.
If your controller or command topic is different, adapt the GUI profile/launch code
before using the system; do not merely remap a topic without checking message type,
joint order, limits, and frame conventions.

Useful pre-flight checks:

```bash
ros2 topic echo /robot/joint_states --once
ros2 topic echo /vicon/Base_RB/Base_RB --once
ros2 topic echo /vicon/Tool_Flange/Tool_Flange --once
ros2 control list_controllers --controller-manager /robot/arm/controller_manager
ros2 run tf2_ros tf2_echo odom base_link
```

For the Robotnik profile, also inspect the unstamped command endpoint; for Bunker,
inspect the stamped endpoint. Use `ros2 topic info -v <topic>` to confirm the exact
message type and subscribers before any motion.

## 3. Vicon, frames, and pose-source choices

Set **External map** to the frame used by the exported paths (normally `map`). It
must be transformable to the Vicon pose header frame. Set **Robot base frame** and
**Robot TF root** to the real TF names, without a leading slash.

The normal hardware mode uses the Base_RB marker. The GUI publishes a calibrated
static transform between the robot base and the Vicon base reference, transforms the
Vicon base measurement into `map`, and publishes `/robot_pose`. It also converts the
Vicon tool marker into `/vicon/tool_transformed` and then
`/current_nozzle_tip_pose`.

Choose exactly one source for `/robot_pose`:

- **Default (Base_RB):** best when the Vicon base marker is visible and its static
  calibration is valid.
- **Use odometry for `/robot_pose`:** anchors the current odometry pose to
  `/base_path[Interpolated index]` once, then follows odometry. It avoids a missing
  base marker but will drift and must be started at the intended path index.
- **Fallback: Base Pose:** uses the Vicon tool pose and the live TF
  `robot_base_frame -> robot_arm_nozzle_tip` to calculate the base pose. It is for a
  missing Base_RB marker, not for a missing tool marker. The tool-marker-to-TCP
  calibration and robot kinematics must be correct.

The latter two checkboxes are mutually exclusive. Do not run another node that
publishes `/robot_pose` concurrently.

### Capture UR TCP Offset

Use **Capture UR TCP Offset** when the real tool flange/controller TCP differs from
the modeled `robot_arm_tool0` frame and the robot TF tree already contains the
calibrated transform `robot_arm_tool0 -> robot_arm_tool0_controller`. This is a
hardware-only convenience action: it reads that TF transform, stores its translation
and XYZW quaternion as `fixed_tool_offset`, and passes the values to the arm
controller/follower when they are next launched.

At GUI startup, the same transform is checked against the configured offset when it
is available. A mismatch produces a warning and makes the capture button red; a
matching offset makes it green.

No values are typed into the GUI. Before pressing the button, provide a valid,
calibrated TF transform with exactly those frame names, keep the robot stationary and
safe, and verify it first, for example:

```bash
ros2 run tf2_ros tf2_echo robot_arm_tool0 robot_arm_tool0_controller
```

Use it after a physical tool/TCP change or when deploying a corrected robot
description. Do **not** use it to calibrate the Vicon marker-to-tool transform or to
compensate an unknown path/Vicon registration error; those are separate calibrations.
After capture, restart the arm controllers and arm follower (or stop and run
**Launch All** again) so the saved offset is applied.

## 4. Add and select a new trajectory

Store each paired trajectory in its own folder under
`match_additive_manufacturing_ros2/components/`, for example:

```bash
cd ~/wattle_daub_ros2_ws/src/match_additive_manufacturing_ros2/components
mkdir wall_2026_07_20
cp robotnik_paired_demo/base_path.json wall_2026_07_20/
cp robotnik_paired_demo/arm_path.json wall_2026_07_20/
cp robotnik_paired_demo/normal_vector.json wall_2026_07_20/
```

The selected folder must contain:

- `base_path.json`
- `arm_path.json`
- `normal_vector.json`

Base and arm paths must have the same number of poses, valid increasing timestamps,
normalized orientations, and a common path frame. Use the **Browse** button in the
GUI to select the directory. The GUI launches `parse_paths` with
`load_exported_trajectories:=true`, so it publishes those JSON trajectories as
`/base_path` and `/ur_path_transformed`.

Use **Path transform X/Y/Z** and **Path rotation** only when the complete trajectory
needs a known rigid registration to the hardware workcell. Confirm the transformed
path in RViz before enabling motion; a path transform is not a substitute for Vicon
or TCP calibration.

## 5. Start the GUI

With the hardware interfaces already running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wattle_daub_ros2_ws/install/setup.bash
ros2 launch am_operator_gui am_operator_gui.launch.py use_sim_time:=false ros_domain_id:=38
```

Select the platform, trajectory folder, frames, pose topics, and the intended pose
source. Keep **Simulation** unchecked for hardware. Open RViz if needed and verify
the base path, arm path, robot pose, TCP/deposition pose, and TF tree visually.

## 6. Normal operating sequence

1. Make the hardware ready: brakes/enable state, E-stop, base driver, UR driver,
   Vicon, TF, joint states, and controller manager all operational.
2. Select the path folder and verify its registration in RViz.
3. Configure frames/topics and confirm that the GUI reports fresh `/robot_pose` and
   `/current_deposition_pose` data.
4. Click **Launch All**. On hardware it starts:
   - Vicon pose adapters (or the selected odometry/TCP fallback);
   - the exported path publisher;
   - the arm velocity-controller/J-PARSE stack;
   - shared path-index generation and reference interpolation;
   - base and arm followers, both gated by `/start_condition`.

   It intentionally does **not** move the robot to the start pose on hardware.
5. Use **Move Base to Start** and **Move Arm to Start** deliberately, one at a time,
   after verifying the selected index, speed limits, and clearance. These are active
   motion commands.
6. Confirm that the arm velocity controller is active and the GUI readiness state is
   green. In particular, the GUI requires paths, fresh base pose, fresh deposition
   pose, J-PARSE readiness, and controller readiness.
7. Click **Start Following** only after the preceding checks. It publishes the
   selected index and `/start_condition=true`; the already-running base and arm
   followers begin tracking together.
8. Use **Stop Following** to close the start gate and repeatedly publish zero base
   and arm twist commands. Use the physical E-stop whenever required.

`Launch All` is process orchestration, not an authorization to move. `Start
Following` is the motion start command.

## 7. Common failures and recovery

| Symptom | Likely cause | Check / recovery |
| --- | --- | --- |
| `/robot_pose: waiting` | missing Vicon/odom input, wrong map frame, invalid base calibration, or stale data | echo the selected source; verify `map` transform and base/root TF names; choose the appropriate pose-source fallback |
| `/current_nozzle_tip_pose` or deposition pose missing | tool marker unavailable, wrong Vicon tool topic, or marker-to-TCP transform is wrong | check `/vicon/Tool_Flange/Tool_Flange`, `/vicon/tool_transformed`, and the tool calibration before continuing |
| Fallback Base Pose does not become ready | `robot_base_frame -> robot_arm_nozzle_tip` TF is absent or the Vicon tool pose cannot transform to `map` | inspect TF chain and frame names; do not use this fallback with an uncalibrated TCP |
| Robot pose jumps or path is offset | wrong Vicon/base static calibration, wrong path registration, or mixed frames | stop, inspect in RViz, recalibrate, and use one consistent world frame |
| Base does not move after Start Following | wrong command topic/type, inactive base driver, missing subscriber, or start gate still false | inspect command topic with `ros2 topic info -v`; echo `/start_condition` and base command output |
| Arm does not move | controller switch failed, J-PARSE not ready, joint-state/robot-description mismatch, or wrong joint command topic | inspect controller list and `/am/jparse_ready`, `/am/arm_controller_ready`; verify joint names and controller command topic |
| Odometry pose starts at an unexpected location | selected path index does not represent the current physical pose | stop; select the correct **Interpolated index** before enabling odometry mode and restart the pose adapters |
| GUI says motion is not ready | one of paths, base pose, deposition pose, J-PARSE, or controller readiness is absent/stale | read the GUI log; resolve the named input rather than forcing Start Following |

For an additional diagnostic view, run:

```bash
ros2 topic echo /base_path --once
ros2 topic echo /ur_path_transformed --once
ros2 topic echo /robot_pose --once
ros2 topic echo /current_deposition_pose --once
ros2 topic echo /start_condition --once
ros2 topic echo /path_index --once
```

When recovering from an uncertain pose, controller state, or path registration, stop
following, make the hardware safe, correct the root cause, then restart the affected
adapters/controllers and repeat the pre-flight checks. Do not resume solely from a
stale GUI status or a previous TF calibration.
