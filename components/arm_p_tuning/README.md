# Standalone arm P tuning paths

The five tests are derived from the first pose and tool orientation in
`components/david_path`. Each returns to its starting pose and includes dwells.
Their paired base path is constant. The original print paths and GUI are unchanged.

| Test | Motion | Exported duration |
| --- | --- | --- |
| `01_along_track` | ±30 mm along the initial print tangent | 45 s |
| `02_cross_track_corners` | Two 20 mm squares, exercising cross-track recovery | 26 s |
| `03_spray_axis` | ±10 mm along tool Z at 5 mm/s | 23 s |
| `04_orientation` | ±3° about tangent, lateral and spray axes | 57 s |
| `05_david_excerpt` | First four David poses, out and back at 10 mm/s | 38.1 s |

## Use the existing web GUI

Use the GUI to launch the pose adapters, arm controller stack, arm follower and
RViz as usual. If you use Launch All, first press **Stop Following**, then stop
its **Publish Path**, **Path Index** and **Base Follower** components. Leave the
arm follower, pose adapters and controllers running. Those three stopped
components are replaced by the command below (the base follower stays off).
Do not launch duplicate publishers or progress nodes while the test is running.
The GUI's saved print folder does not need to change.

From this repository, in a terminal with the same ROS workspace and ROS domain
as the GUI:

```bash
ros2 launch ./components/arm_p_tuning.launch.py test:=01_along_track
```

Select another test with the `test` argument. No rebuild is required for this
launch file. It publishes paths and timestamp-based references; it does not
launch an arm/base driver, move to the start, or send a true start condition.
It also publishes zero on `/desired_arm_speed` so the existing arm follower
uses the exported segment speeds instead of the GUI's configured print speed.
The existing GUI velocity override still scales trajectory progress and
feedforward. The GUI's saved speed setting is not changed.

If David's print path needs a registration transform, supply the same values:

```bash
ros2 launch ./components/arm_p_tuning.launch.py test:=01_along_track \
  'path_transform_xyz:=[0.0, 0.0, 0.0]' path_transform_yaw_deg:=0.0
```

In RViz, use **Fixed Frame: `vicon_world`** and a **Path** display for
`/ur_path_transformed` or `/ur_path_tracking`. The existing Robotnik RViz
configuration already displays `/ur_path_transformed`. Inspect the full test's
clearance and posture, with spraying off and the base stationary. Generated
paths have not been checked against the physical robot or obstacles.

After references and index zero appear, the existing **Move Arm To Start**
button targets the published test start. Then use **Start Following** to run
it. There is no new GUI panel or automatic tuning workflow.

## Cancel and repeat

Use the GUI's **Stop Following** button whenever you want to cancel tracking.
The equivalent command from another sourced terminal is:

```bash
ros2 topic pub --times 10 --rate 10 /start_condition std_msgs/msg/Bool '{data: false}'
```

This closes the existing tracking/progress gate; it is a software stop, not an
emergency stop. **Stop Following before pressing Ctrl+C in the launch terminal.**
Ctrl+C alone removes the path/progress publishers while the GUI arm follower
may still correct toward its last reference. No automatic stop is added.

To repeat from the beginning, Stop Following, Ctrl+C the command-line launch,
and run it again. Wait for index zero, then press Start Following. Restarting
this launch resets phase without changing the existing controller code.

The direction P gains can be changed between runs from the command line:

```bash
ros2 param set /ur_direction_controller along_track_kp 1.2
ros2 param set /ur_direction_controller orthogonal_kp 0.8
ros2 param set /ur_direction_controller kp_z 0.5
```

These are examples, not validated hardware gains. Change only one at a time.
The orientation controller caches its gains at startup; its P change requires
relaunching the arm follower with the desired value. Orientation Ki/Kd should
remain zero for these P trials. Move-to-start gains belong to the separate
move controller, not the path follower.

When finished, Stop Following and Ctrl+C the test launch. Relaunch the ordinary
GUI path publisher, path index and base follower for the saved print folder.
Restart the arm follower to restore its saved GUI gains/speed after CLI tuning.

## Regenerate

```bash
python3 components/generate_arm_p_tuning_paths.py
```

`--anchor-index N` selects another source pose; `--output DIR` writes another
suite. Regeneration replaces generated test JSONs only.
