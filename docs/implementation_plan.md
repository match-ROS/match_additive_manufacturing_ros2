# ROS 2 Printing Implementation Plan

This is the execution order for reaching a safe ROS 2 print workflow while
hardware is unavailable.  A stage only advances after its stated evidence is
available; this prevents hardware debugging from masking missing interfaces or
unsafe defaults.

## Repository ownership

| Responsibility | Repository | First implementation target |
| --- | --- | --- |
| Print paths, progress, operator workflow, tracking, monitoring, process safety | `match_additive_manufacturing_ros2` | profile contracts, start/stop gates, replay, contour consumer, `am_process_control` |
| MuR/MiR description, Gazebo, base and arm controllers | `match_mobile_robotics_jazzy` | reliable controller-spawner order and MuR arm endpoint contract |
| Bunker-specific platform integration | `bunker_manipulator` | retain as separate platform profile |
| Qualisys/QTM transport and mocap calibration | `match_mocap` | recorded-QTM replay and registration before live QTM receiver |
| Keyence TCP/profile transport | `match_hardware_utilities` | `keyence_profile_ros2` recorded-profile replay, then LJ-X8400 adapter |

The pinned platform repository URLs and revisions are in
[`dependencies/print_system.jazzy.repos`](../dependencies/print_system.jazzy.repos).
Before the sensor-enabled workspace can be reproducible, publish
`mocap_toolbox_ros2` in
[`match_mocap`](https://github.com/match-ROS/match_mocap) and
`keyence_profile_ros2` in
[`match_hardware_utilities`](https://github.com/match-ROS/match_hardware_utilities)
to named Jazzy branches, then add their exact commits to that manifest. Do not
pin the current ROS 1-only upstream heads: they would omit the ROS 2 packages.
The shared topic and safety rules are in
[`print_system_contract.md`](print_system_contract.md).

## Stage 0 — reproducible developer machine

1. Create a clean workspace and import the pinned manifest with `vcs import`.
2. Initialise required Bunker/UR vendor submodules.
3. Source ROS 2 Jazzy, build the application packages into an isolated overlay,
   and run `colcon test-result --verbose`.
4. Keep ROS 1 repositories outside the ROS 2 colcon source tree; use them only
   as behavioural reference and as a source of recorded bags/configuration.

Exit evidence: the offline build and test command in
[`offline_verification.md`](offline_verification.md) is green on a new machine.

## Stage 1 — print-system contract and offline tests

1. Keep the four operating profiles explicit: `localization_only`,
   `mocap_localization`, `localization_contour_monitor`, and
   `localization_contour_control`.
2. Add fixture-based tests for malformed paths, pose freshness, paired-path
length/timing, start-gate behavior, and zero commands.
3. Require every platform profile to declare pose topic, command type/topic,
frame names, limits, and whether arm control is verified.

Exit evidence: no start signal is emitted if any readiness gate or required
follower is missing, and no unverified platform can start another platform's
arm stack.

## Stage 2 — MuR base-only simulation

1. Build `match_mobile_robotics_jazzy` through `mur_launch_sim` into a clean
overlay, then launch MuR headlessly with ground truth and fake localization.
2. The launch serializes controller-spawner activation and uses the Jazzy
`gz_ros2_control` plugin filename.  Run the simulation in its own ROS domain
and Gazebo partition when another simulator is present; the verified result is
an active `/mur620a/controller_manager`, `joint_state_broadcaster`, and
`mobile_base_controller`.
3. Verify `/mur620a/ground_truth/pose` (`PoseStamped`),
`/mur620a/ground_truth/odom` (`Odometry`), and
`/mur620a/mobile_base_controller/cmd_vel` (`TwistStamped`) live.
4. Start the AM base follower against a synthetic `/base_path`; test zero,
bounded forward, and turn commands, then test stale pose/path and stop.

Exit evidence: the follower moves the simulated base and every interlock case
produces zero output.  The base-only MuR motion gate is complete: a forward
reference moved the simulated base from x=44.0 to x=44.551 and the follower
then published zero at its goal; node-level tests cover the start gate and
stale-pose stop.  No arm controller, J-PARSE helper, or MoveIt stack is
launched for this profile.

## Stage 3 — MuR arm integration

The first native left-arm command-chain gate is now observed in simulation:
the manager is `/mur620a/controller_manager`, J-Parse subscribes to
`/mur620a/jparse_velocity_controller_l/twist_cmd` (`TwistStamped` in
`UR10_l/base_link`), and publishes to the active
`/mur620a/forward_velocity_controller_l/commands` endpoint.  The AM profile
`mur620_left_arm_sim` is deliberately separate from the base-only profile. Its
small bridge transforms follower world twists into that native input and checks
both subscriptions before reporting readiness. Automatic arm move-to-start is
still blocked: a calibrated nozzle/tool transform and independently validated
MuR start-motion path are required first.

The GUI stop route is profile-driven. In the MuR profile it sends a stamped
zero directly to `/mur620a/jparse_velocity_controller_l/twist_cmd` in
`UR10_l/base_link` and a zero `TwistStamped` to the MuR base command topic; it
does not depend on the AM twist-transform process being alive. A transition
from all-ready to any-not-ready clears `/start_condition` and publishes these
stops. This is an offline safety behavior, pending live controller-loss
verification in simulation and later hardware.

1. Specify the right-arm frame and controller contract in
`match_mobile_robotics_jazzy`: URDF links, joint order, controller manager,
input topic, message type, and controller switch sequence.
2. Add launch and command tests in that repository.  The AM repository should
only consume this published contract; it must not hard-code MuR frames or
Robotnik controller names.
3. Add the verified MuR arm profile to `am_operator_gui`, then test start pose,
follower, controller loss, and repeated zero command in simulation.

Exit evidence: base and arm follow a paired path in simulation while the stop
gate and controller timeout halt both safely.

## Stage 4 — localization and scanner replay

1. A ROS 2 `mocap_toolbox_ros2` package now exists in `match_mocap`: it replays
JSON-lines rigid-body fixtures, applies a configured planar map registration,
publishes the selected `/robot_pose`, and reports input freshness diagnostics.
The initial replay launch was verified from `/qualisys/test/pose` through to a
map-frame `/robot_pose`.  Its `qtm_receiver` now also selects a named QTM 6D
body, explicitly converts the normal QTM millimetre stream to metres, reconnects
on transport failure, and diagnoses missing SDK/invalid body/stale input. Test
the adapter against QTM and then validate timecode/timestamp policy, dropout,
and calibration datasets; retain the hardware-free replay path.
2. `match_hardware_utilities/keyence_profile_ros2` now preserves the ROS 1
profile boundary: `/profiles` (`PointCloud2`), `/profiles_float`, and
`/profiles_pitch_m`, in metres.  Its recorded-profile replay builds and has
been live-checked on an isolated ROS domain. Its `ljx8_profile_driver` now
wraps the supplied LJ-X8k Linux vendor SDK, explicitly converts native units
and invalid returns, reconnects on failures, and reports diagnostics. Add a
fake-SDK/fake-controller test and then verify the physical controller; do not
put scanner-specific networking or map-frame processing in the AM repository.
3. `print_path_monitoring` contains the contour estimator and a bounded,
default-off correction node. It correlates the raw Keyence profile with a
recorded reference, reports signed lateral and height error only with sufficient
finite/observable overlap, and diagnoses stale/invalid profile input. When
explicitly enabled, the correction node contributes limited world Y/Z velocity
only while the errors are fresh and `/start_condition` is true; otherwise it
continuously contributes zero. The GUI exposes this as an explicit persisted
operator setting, defaulting to disabled. Validate its sign, world-axis/frame
assumption, and sensor calibration with representative recorded profiles before
hardware use.

Exit evidence: all three legacy operation modes can run from recorded data:
mocap localization, localization only, and localization plus contour
monitoring/control.

## Stage 5 — process control and hardware commissioning

1. `am_process_control` now provides the pure, default-off policy and a ROS 2
node.  It requires an armed state, explicit acknowledgement, `/start_condition`,
and fresh flow feedback before it can ramp `/process/valve_target`; a stop or
stale feedback commands zero immediately. Its `flow_serial_bridge` now ports
the Arduino's prefixed CSV flow stream, publishes filtered left/right and a
selected safety-channel measurement, and has a pseudo-terminal parser test.
The mockable valve/Dynamixel mapping remains downstream of the one policy
output. `dynamixel_valve_adapter` now publishes the legacy left/right Dynamixel
goal topics only from that policy target, defaults disabled, and watchdogs
stale input to configured closed positions. Bind it to the actual ROS 2
Dynamixel workbench/vendor service and test with a mock before connecting
process equipment.
2. Commission hardware in this order: base localization only; arm only;
base+arm dry run; mocap comparison; scanner monitoring; scanner correction;
process material output last.
3. At each step record rosbag, controller state, topic rates, TF tree, safety
interlock result, and a signed test checklist.  A failed step returns to its
simulation/replay test rather than being patched directly on hardware.

Exit evidence: a dry run and then a supervised material run meet the contract
and have reproducible commissioning records.

## CI gate

Each actively ported repository now carries a small Jazzy GitHub Actions
workflow: `match_additive_manufacturing_ros2` builds the hardware-free AM
packages and runs their unit/launch-wiring tests; `match_mocap` builds and tests
`mocap_toolbox_ros2`; `match_hardware_utilities` builds and tests
`keyence_profile_ros2`.  All workflows run a narrow static Python gate for
syntax/undefined-name errors. They deliberately exclude Gazebo, QTM, Keyence,
and process hardware; those stay manual/nightly/commissioning gates. The first
remote workflow results must be recorded after these changes are pushed.
