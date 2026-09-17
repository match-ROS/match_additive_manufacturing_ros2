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

### Sync sources and run JParse on the robot

**Sync Workspace** copies the local workspace's entire `src/` tree over SSH to
`robot@192.168.0.200:~/b04_gui_ws/src/`. SSH key access and rsync are available on
this robot. Restart the GUI after updating it to pick up the new destination.
The action overwrites changed files, keeps destination-only files, and does not
build or launch anything. Check the `sync_workspace` process log for completion.

After syncing, build on the robot:

```bash
ssh robot@192.168.0.200
source /opt/ros/jazzy/setup.bash
cd ~/b04_gui_ws
colcon build --symlink-install --packages-select am_jparse_controller
source install/setup.bash
```

With the robot drivers running and the same ROS domain/discovery configuration
as the operator PC, launch JParse using the GUI's controller twist topic:

```bash
ros2 launch am_jparse_controller am_jparse_velocity_controller.launch.py \
  twist_topic:=/jparse_velocity_controller_ur/twist_cmd \
  fixed_tool_offset_xyz:='[-0.25, 0.0, 0.015]' \
  fixed_tool_offset_quaternion_xyzw:='[0.0, -0.7071067812, 0.0, 0.7071067812]' \
  command_joint_names_csv:=robot_arm_shoulder_pan_joint,robot_arm_shoulder_lift_joint,robot_arm_elbow_joint,robot_arm_wrist_1_joint,robot_arm_wrist_2_joint,robot_arm_wrist_3_joint
```

Use the same tool offset and velocity limits as your GUI configuration; the
example above uses the standard Robotnik tool offset. This starts JParse only:
it requires the twist transform publisher, robot description, joint states,
spray distance, and an active forward velocity controller. The GUI's **Start
Controllers** action starts a local JParse instance too, so do not combine it
with this remote instance. Sync does not change where GUI launch actions run.

### Robot clock diagnostics

Both interfaces offer a normally hidden **Debugging info** panel: expand it in
the web GUI, or enable its checkbox in Qt. While open, it checks
`robot@192.168.0.200` over key-based SSH every ten seconds, with an eight-second
timeout. It displays SSH reachability, the estimated robot minus PC clock offset
and measurement uncertainty, configured/system, network and fallback NTP servers,
the currently selected server, and synchronization status. The remote probe uses
Python 3 and `timedatectl`; no sudo is needed. A selected server alone does not
mean synchronization succeeded. Failed checks clear the previous clock reading.

For this workstation (`192.168.0.222`), Chrony already serves NTP to the robot
network. To point the robot's `systemd-timesyncd` at it, run from this PC:

```bash
bash src/match_additive_manufacturing_ros2/am_operator_gui/scripts/configure_robot_time.sh
```

Enter the robot's sudo password in the terminal. The script installs
`/etc/systemd/timesyncd.conf.d/99-am-operator-pc.conf`, overriding the old server
list, and restarts the time service. Keep the PC's address stable and the PC
running. Apply the correction with motion stopped, then restart affected ROS
nodes to reset TF histories. To undo, remove that drop-in and restart
`systemd-timesyncd`. Existing copies of the drop-in are backed up before changes.

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

When launching through ROS, select the browser UI explicitly:

```bash
ros2 launch am_operator_gui am_operator_gui.launch.py ui:=web
```

The default `ui:=qt` continues to start the reference Qt application. Do not run
both interfaces as active controllers at the same time.

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

### Vicon world frame

For the active `components/david_path` setup, **Control frame** and **External map**
are `vicon_world`; **Robot base frame** is `robot_base_footprint` and **Robot TF root**
is `robot_odom`. The imported arm/base path files also use `vicon_world`.
Changing a frame label preserves coordinates; it does not calculate a registration
transform between independently calibrated worlds.

The web readiness checks use **Control frame**. Path freshness allows 2.5 seconds
for the 1 Hz path publisher; measured pose freshness remains 0.75 seconds.
Restart pose adapters and followers after changing their frame settings.

### Web GUI: nozzle transforms

On hardware, **Launch Transformations**, **Pose Adapters**, and **Launch All**
start a static `robot_arm_tool0 -> robot_arm_nozzle_tip` TF using the selected
platform's **Robot flange → nozzle** offset. Restart the pose adapters after
changing that offset. Simulation already supplies the nozzle frame through its
URDF, so this additional publisher is disabled there. Hardware must still supply
the live base-to-tool0 kinematic chain. Do not run another publisher for the same
nozzle child frame alongside this adapter.

The **Nozzle transforms** section places two independent calibrations side by side
(stacked on narrow screens):

- **Vicon EE / marker → nozzle** sets the **Measured Vicon EE topic** and the
  nozzle pose relative to that measured frame. Hardware topic checks and the
  Vicon adapter use this input. Translation is in metres in the measured frame;
  rotation is XYZW quaternion or RPY in degrees (roll X, pitch Y, yaw Z).
  Restart **Pose Adapters** after changing the input or calibration.
- **Robot flange → nozzle** describes the nozzle relative to `robot_arm_tool0`
  for the kinematic/controller chain, separately for each platform. Restart arm
  controllers and the follower after changing it. It does not change Vicon calibration.

Both describe the same nozzle; their values coincide only when the Vicon measured
frame coincides with the robot flange. The Vicon defaults retain the previous
hardcoded calibration and must match your marker setup. If Vicon already measures
nozzle-tip pose, use zero translation and quaternion `[0, 0, 0, 1]`.

Hardware feedback always follows the chain:
`measured Vicon EE → Vicon calibration → /vicon/tool_transformed → control-frame
conversion → /current_nozzle_tip_pose → spray-distance offset → /current_deposition_pose`.
The follower compares the deposition pose with `/arm_trajectory_reference`.
Base reconstruction uses an independent Vicon nozzle reference:
`measured Vicon EE → vicon_fallback_nozzle_transform → /vicon/nozzle_fallback`
(TF frame `vicon_nozzle_fallback`). The base adapter matches this measurement to
the kinematic `robot_arm_nozzle_tip` reference. Their correspondence is identity;
the calibrated EE-to-reference transform is applied by the independent adapter.
Changing the fallback calibration does not affect the deposition pose or its
Vicon calibration. If no separate calibration is configured, the second adapter
uses the deposition calibration for compatibility. Restart the GUI service and
pose adapters after deploying this change. Validate the fallback calibration
over multiple arm configurations; the current calibration used a fixed posture.

Existing custom `arm_pose_topic` selections such as `/vicon/EE/root` become the
`vicon_input_topic` when loading an older web configuration. They now pass through
Vicon calibration. The downstream `arm_pose_topic` is fixed to
`/vicon/tool_transformed`; it is no longer an editable web setting. Existing
calibration values are retained, so review the Vicon offset when migrating a
configuration that previously fed a raw EE pose directly to the nozzle adapter.
These settings apply to the web GUI; the legacy Qt launcher retains its own launch wiring.

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

For a manual RViz preview, start **Publish Path**, then **Show Index Poses**.
This independently resamples the coupled paths into the same 5 mm tracking index
space as Path Index. It publishes `PoseStamped` messages on
`/selected_arm_index_pose` and `/selected_base_index_pose`, following the GUI's
selected index (and the live index when Path Index is running). Both operator
RViz configurations include these displays with Transient Local durability.
The preview starts only through its own button; **Launch All** does not start it.
Use **Stop Index Poses** to stop publishing. No motion or progress commands are
published by the preview. Publish Path must supply valid coupled paths; an
out-of-range index produces no new pose, so RViz may retain the previous pose.
