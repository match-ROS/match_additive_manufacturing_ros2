# MuR620 (MiR + two UR10) ROS 2 Jazzy installation

This manual installs the non-Robotnik part of the ROS 2 printing foundation for
one physical MuR620: its MiR base and two UR10 arms. It deliberately excludes
Robotnik, Bunker, Gazebo, and printing motion. A successful installation is not
permission to move a robot or extrude material.

## Current scope and safety boundary

`match_mobile_robotics_jazzy` provides the MuR hardware launch:
`mur_launch_hardware/mur_620.launch.py`. It has separate left/right UR IP,
reverse, script-sender, trajectory, and script-command ports, and MUR-specific
calibration profiles for `mur620a` through `mur620d`.

`match_additive_manufacturing_ros2` currently has tested **simulation** MuR
profiles only. In particular, `mur620_left_arm_sim` is not a real-hardware
profile and must never be selected for the physical robot. The real-MuR AM
profile, calibrated nozzle transform, physical localization source, and staged
motion approval remain commissioning work.

## Existing setup scripts

The mobile repository contains two useful scripts:

- `ROS2_setup.sh` bootstraps Ubuntu 24.04/Jazzy, ROS tooling, Gazebo, Nav2,
  MoveIt, ros2_control, and imports `ros2.repos`. It is broad, modifies apt,
  may initialize rosdep, and by default edits `.bashrc`; review it before use.
  To prevent `.bashrc` edits: `UPDATE_BASHRC=0 bash ROS2_setup.sh`.
- `setup_mur_hardware_host.sh` checks or configures real-host prerequisites:
  serial permissions for lift columns, realtime limits, and optionally UR
  dashboard reachability. Run `--check` before `--apply`.

Neither script installs Robotnik packages.

For the focused workflow in this manual, use
[`scripts/setup_mur620_workspace.sh`](../scripts/setup_mur620_workspace.sh).
It keeps privileged apt installation opt-in and never launches hardware:

```bash
./scripts/setup_mur620_workspace.sh --install-system
```

## 1. Create the workspace and fetch source

```bash
sudo apt update
sudo apt install -y \
  git python3-rosdep python3-vcstool python3-colcon-common-extensions \
  ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros2controlcli ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-moveit ros-jazzy-tf-transformations \
  python3-can python3-serial python3-websocket

mkdir -p ~/workspaces/mur620_print_ws/src
cd ~/workspaces/mur620_print_ws/src
git clone https://github.com/match-ROS/match_additive_manufacturing_ros2.git
git clone https://github.com/match-ROS/match_mobile_robotics_jazzy.git
cd match_mobile_robotics_jazzy
git submodule update --init --recursive \
  ur_robot/Universal_Robots_ROS2_Driver \
  ur_robot/Universal_Robots_Client_Library \
  ewellix/ewellix_lift ewellix/ewellix_lift_common ewellix/serial
cd ..
vcs import . < match_mobile_robotics_jazzy/ros2.repos
```

The `ros2.repos` file currently imports `ira_laser_tools`; install its PCL
dependencies if that package is used:

```bash
sudo apt install -y ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs
```

If the supplied UR driver package is used instead of the UR source submodules,
install it and omit the two UR source roots from the build command below:

```bash
sudo apt install -y ros-jazzy-ur-robot-driver
```

Do not build the apt driver and source driver together unless deliberately
resolving the upstream-package override.

## 2. Resolve dependencies without Robotnik

From the workspace root:

```bash
cd ~/workspaces/mur620_print_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # only if /etc/ros/rosdep/sources.list.d/20-default.list is absent
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys 'robotnik_rbvogui_tum'
```

The AM-owned J-PARSE package is used when the additive manufacturing stack is
launched. The standalone MuR launch still defaults to its native mur_control
J-PARSE controller.

## 3. Build the MuR and AM packages

Use explicit source roots so colcon does not discover unrelated workspaces.
The `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` flag is required when the Ewellix
serial dependency is built from source.

```bash
cd ~/workspaces/mur620_print_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths \
    src/match_additive_manufacturing_ros2 \
    src/match_mobile_robotics_jazzy/mir_robot \
    src/match_mobile_robotics_jazzy/mur_robot \
    src/match_mobile_robotics_jazzy/ur_robot/Universal_Robots_Client_Library \
    src/match_mobile_robotics_jazzy/ur_robot/Universal_Robots_ROS2_Driver \
    src/match_mobile_robotics_jazzy/ewellix/ewellix_lift \
    src/match_mobile_robotics_jazzy/ewellix/ewellix_lift_common \
    src/match_mobile_robotics_jazzy/ewellix/serial \
    src/ira_laser_tools \
  --packages-up-to mur_launch_hardware am_operator_gui \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source install/setup.bash
```

If this reports a missing package, use `rosdep` for the named dependency rather
than adding Robotnik packages. For an AM-only offline build, omit the mobile
source roots and build the AM packages separately.

## 4. Verify the real host before connecting devices

Run the checker first. Set the workspace path explicitly because its default is
not this manual's workspace:

```bash
cd ~/workspaces/mur620_print_ws
WS="$PWD" REPO="$PWD/src/match_mobile_robotics_jazzy" \
  bash src/match_mobile_robotics_jazzy/setup_mur_hardware_host.sh --check
```

After reviewing its output, apply only the required host configuration:

```bash
WS="$PWD" REPO="$PWD/src/match_mobile_robotics_jazzy" \
  bash src/match_mobile_robotics_jazzy/setup_mur_hardware_host.sh --apply
```

Log out and back in after group changes. Do not enable dashboard network checks
until the UR IP/name mapping is known; then use, for example:

```bash
MUR_CHECK_UR_NETWORK=true MUR_UR_HOSTS='192.168.12.101 192.168.12.102' \
WS="$PWD" REPO="$PWD/src/match_mobile_robotics_jazzy" \
  bash src/match_mobile_robotics_jazzy/setup_mur_hardware_host.sh --check
```

## 5. Staged hardware bringup

Select the exact physical calibration profile (`mur620a`, `mur620b`, `mur620c`,
or `mur620d`). Do not substitute a profile merely because it has a similar
name: each carries arm mounting and UR kinematics calibration values.

First load descriptions and common launch wiring with every actuator path
disabled:

```bash
ros2 launch mur_launch_hardware mur_620.launch.py \
  robot_name:=mur620a robot_profile:=mur620a \
  launch_mir:=false use_arms:=false launch_ur_l:=false launch_ur_r:=false \
  launch_lift_l:=false launch_lift_r:=false launch_bms:=false \
  launch_jparse_idk:=false launch_moveit:=false
```

Then commission in this order, recording a rosbag, TF tree, controller list,
configuration snapshot, and pass/fail result at each stage:

1. MiR network/API only (`launch_mir:=true`), with arm/lift/BMS paths disabled.
2. BMS CAN and lift columns, if the selected profile uses lifts.
3. Left UR driver only (`launch_ur_l:=true launch_ur_r:=false`), with no AM
   follower or automatic path execution.
4. Right UR driver only.
5. Both UR drivers; verify distinct IP addresses and the default non-overlapping
   left/right reverse/script/trajectory ports supplied by the launch file.
6. Native `mur_control` J-PARSE endpoints and zero-command behavior.
7. Localization, calibrated nozzle transform, AM path publication, dry
   base/arm motion, contour monitoring, and only then bounded correction.

The MiR interface is independent of Robotnik. In this launch it is optional
(`launch_mir:=false` by default) and uses `mir_launch_hardware`/the MiR driver
when enabled; ensure the MiR REST/API endpoint and its source topic contract
are validated before enabling base motion.

## Do not do yet

Do not use `am_operator_gui` to start following on a physical MuR620 until a
dedicated real-MuR profile has been added and verified. The present GUI safety
and command-chain evidence covers MuR simulation only. The remaining required
gates are physical localization, QTM/Keyence calibration, controller-loss stop
validation, nozzle calibration, and process-equipment commissioning.
