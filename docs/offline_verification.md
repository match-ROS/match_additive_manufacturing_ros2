# Offline Verification Baseline

This record establishes the first repeatable, hardware-free verification gate
for the pinned print-system foundation.  It was run on 2026-07-23 at revision
`a15ce88378675961cf8272b77096570a63514714`.

## Build

The following packages built successfully in a temporary overlay:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to \
  am_operator_gui am_bringup print_path_monitoring
```

The resulting package set was:

- `base_trajectory_follower`
- `move_to_path_idx`
- `parse_paths`
- `ur_trajectory_follower`
- `print_path_monitoring`
- `am_operator_gui`
- `am_bringup`

The only build diagnostics were Python `setuptools` warnings about the distro
`pytest-repeat` egg metadata.  They did not indicate a package build failure.

## Tests

```bash
colcon test --packages-select \
  base_trajectory_follower parse_paths ur_trajectory_follower \
  print_path_monitoring am_operator_gui
colcon test-result --verbose
```

| Package | Result |
| --- | --- |
| `base_trajectory_follower` | 9 passed |
| `parse_paths` | 15 passed |
| `ur_trajectory_follower` | 20 passed |
| `print_path_monitoring` | 9 passed |
| `am_operator_gui` | 50 passed, 1 skipped |

Total: **103 passed, 1 skipped, 0 failed**.

The skipped test is the optional web GUI end-to-end test.  It remains explicitly
skipped until its browser/runtime dependency is part of the reproducible
development environment.

## Current complete AM-package gate

On 2026-07-23, all eight AM packages were rebuilt together in a clean temporary
Jazzy overlay using `colcon build --symlink-install`, followed by `colcon test`
and `colcon test-result --all --verbose`. The `am_process_control` package was
corrected to declare `tests_require=['pytest']`; without that declaration its
otherwise passing policy tests were silently undiscoverable by `colcon test`.

| Package | Result |
| --- | --- |
| `am_operator_gui` | 63 passed, 1 skipped |
| `am_process_control` | 10 passed |
| `base_trajectory_follower` | 10 passed |
| `move_to_path_idx` | no tests |
| `parse_paths` | 15 passed |
| `print_path_monitoring` | 16 passed |
| `ur_trajectory_follower` | 20 passed |
| `am_bringup` | no tests |

The complete gate was rerun after the ROS graph tests were added: **134 tests,
0 errors, 0 failures, 1 skipped**. The only build stderr
was the known distribution `pytest-repeat` egg-metadata warning. The skipped
web GUI end-to-end test remains intentionally excluded until its browser/runtime
dependency is made reproducible.

The repository's `offline-ci` workflow now mirrors this gate: it builds all AM
packages, runs `colcon test`, and reports `colcon test-result --all --verbose`.
`controllers_ros2` and `robotnik_rbvogui_tum` are deliberately skipped only
during rosdep resolution because they are platform runtime dependencies not
needed to build or test the hardware-free AM source set. The workflow YAML was
parsed locally; remote GitHub execution remains pending publication.

The installed `sideways_arm_control.launch.py` and
`mur_arm_velocity_stack.launch.py` were also loaded with `ros2 launch
--show-args` in the clean overlay (using a writable temporary ROS log
directory). This proves Python launch parsing and the declared interface only:
the contour-correction and native MuR controller processes still require their
separate graph and hardware commissioning checks. The same parser checks are
now part of `offline-ci`.

`print_path_monitoring` additionally has a synthetic ROS graph test for
`contour_correction`. It observes a zero `Twist` before the start gate, a
bounded Y/Z correction only after fresh lateral/height inputs and a true start
condition, and a zero correction after the input watchdog expires. The package
passes **16 tests** through `colcon test` in the clean overlay. This is an
offline topic/policy result, not evidence of Keyence calibration or physical
correction direction.

`am_process_control` now also has synthetic ROS graph coverage of
`process_safety_node`. It observes zero output at startup, permits output only
after target, arm, acknowledgement, start condition, and feedback have all
arrived, then verifies that feedback expiry alone closes the valve output. The
package passes **10 tests** through `colcon test` in the clean overlay. This
does not drive a physical valve or Dynamixel; it verifies the required ROS
interlock boundary before that hardware adapter is commissioned.

## Safety-gate update

The 2026-07-23 offline safety update added a MuR620 simulation platform
profile and made the GUI fail closed:

- `Start Following` now emits neither `/path_index_command` nor
  `/start_condition=true` when a readiness gate or either follower is missing.
- MuR simulation is allowed to run its verified base path-following stack, but
  the GUI refuses to start the Robotnik-specific arm controller stack.  The
  MuR arm stays unavailable until it has its own verified controller launch,
  frame contract, and command endpoint.

The complete selected-package build and test gate was rerun in a clean temporary
overlay after this change:

| Package | Result |
| --- | --- |
| `base_trajectory_follower` | 9 passed |
| `move_to_path_idx` | no tests |
| `parse_paths` | 15 passed |
| `ur_trajectory_follower` | 20 passed |
| `print_path_monitoring` | 9 passed |
| `am_operator_gui` | 54 passed, 1 skipped |
| `am_bringup` | package test target passed |

`colcon test-result --verbose`: **108 tests, 0 errors, 0 failures, 1 skipped**.
The skipped test remains the optional web GUI end-to-end test.

## Next gate

The next offline gate is a headless MuR simulation launch that conforms to
[print_system_contract.md](print_system_contract.md).  It must prove the pose
source, base command type, start gate, and zero-command behaviour.  MuR arm
control is a separate subsequent gate: first define and test its controller
endpoint and frame contract, then integrate it with the arm follower.

The current MuR source dependency closure builds successfully (12 packages).
An isolated headless launch (`ROS_DOMAIN_ID=42` and its own `GZ_PARTITION`) now
proves all base-controller prerequisites: `/mur620a/controller_manager` comes
up, `joint_state_broadcaster` and `mobile_base_controller` reach `active`, and
the command and odometry topics are present.  The fix in
`match_mobile_robotics_jazzy` aligns the model plugin filename with the Jazzy
`gz_ros2_control` demos and starts controller spawners serially without adding
an invalid empty final event handler.  Isolation matters on this machine
because another Gazebo instance otherwise shares the default discovery graph.

This is a base-controller availability pass, not yet a follower-motion pass.
The operator profile explicitly starts no MuR arm controllers, J-PARSE
helpers, or MoveIt processes.

## MuR base-follower motion gate

The next isolated run used the AM `simple_base_follower` with the MuR profile:

- pose: `/mur620a/ground_truth/pose` (`PoseStamped`)
- command: `/mur620a/mobile_base_controller/cmd_vel` (`TwistStamped`)
- frame: `mur620a/base_footprint`
- mode: differential drive, explicit `/start_condition`, 0.1 m/s maximum

A two-point path and a forward reference from `(44.0, 44.0)` to `(44.6, 44.0)`
moved the isolated simulated robot to `(44.551, 44.000)`.  The follower then
reported `goal reached`, and the observed controller command was zero in all
linear and angular axes.  The runtime test suite also covers the same node
contract without Gazebo: no command before the start gate, bounded
differential-drive output after it, and zero output on either a false start
condition or a stale pose.  Result: **10 passed** for
`base_trajectory_follower`.

This establishes the base-only movement and stop gate.  It does not validate
the MuR arm, paired base-arm following, or process output.

## MuR native left-arm command-chain gate

The full MuR dependency closure (`--packages-up-to mur_launch_sim`) built 12
packages in a clean temporary overlay.  In an isolated headless simulation the
following controller state was observed: `mobile_base_controller`,
`joint_state_broadcaster`, `forward_velocity_controller_l`, and
`forward_velocity_controller_r` were active; the trajectory controllers were
inactive.  Graph inspection then showed the native left J-Parse subscriber on
`/mur620a/jparse_velocity_controller_l/twist_cmd` and the active forward
velocity controller subscriber on
`/mur620a/forward_velocity_controller_l/commands`.

`am_operator_gui` now has a separate `mur620_left_arm_sim` profile.  It starts
the native MuR arm controllers and a small bridge that transforms the AM
world-frame follower topic `/mur620a/arm_following/twist_world` into
`UR10_l/base_link`; the bridge reports readiness only when the two native
subscriptions exist.  The profile explicitly refuses automatic arm
move-to-start, because the nozzle transform and a safe calibrated MuR motion
path have not been validated.  GUI tests: **58 passed, 1 skipped**; the package
also builds in a clean overlay.

This is a command-chain availability result, not an arm-motion result.  It
does not yet demonstrate a valid tool/nozzle transform, a safe trajectory,
arm motion, paired following, controller-loss stopping, or GUI-enabled
printing.

The QTM transport configuration is now validated before the optional `qtm-rt`
SDK or network connection is used: a non-empty host, rigid-body name and frame,
a port in `1..65535`, and a finite positive position scale are required.
Malformed values are reported as a terminal configuration diagnostic rather
than entering a reconnect loop; genuine connection failures retain the normal
retry behavior. The QTM/replay suite is **23 passed** through `colcon test` and
the package builds in a clean temporary Jazzy overlay. It includes a synthetic
ROS graph test of map registration and rejection of reordered source stamps.
This is still an offline transport result; stationary-body QTM measurement,
timing, and registration calibration are required hardware gates.

The GUI zero-command boundary is now profile-specific. Its MuR route publishes
directly to the native left J-Parse input and the stamped MuR base command
topic, rather than relying on the arm bridge. When any previously satisfied
readiness gate drops, the GUI clears `/start_condition` and publishes both
zeros. Command-construction/readiness-loss tests cover this behavior; the GUI
suite is **60 passed, 1 skipped** and the package builds in a clean overlay.
This is not yet a live controller-loss simulation result.

## Continuous-integration definition

New Jazzy GitHub Actions workflows exist in the AM, mocap, and hardware
repositories. They build only the hardware-free packages, run their offline
pytest suites, and use Ruff's syntax/undefined-name selection as a static
Python gate. The exact Ruff selection was run locally across all three source
and test trees and passed. Remote Actions evidence is not available yet because
the local changes have not been pushed; Gazebo and physical-device tests remain
outside these pull-request workflows.

## Mocap replay and registration gate

`match_mocap/mocap_toolbox_ros2` is a new Jazzy package retained alongside the
ROS 1 reference packages.  Its replay node accepts JSON-lines fixtures of
timestamped rigid-body poses; its registration node applies a configurable
map-from-mocap translation and yaw and is the sole publisher of `/robot_pose`.

The included nominal fixture was launched in a separate ROS domain.  The graph
contained `/qualisys/test/pose`, transformer diagnostics, and `/robot_pose`;
the observed output was a map-frame pose at `(1.0, 2.0, 0.3)` with identity
orientation.  Four pure fixture/registration tests pass and the package builds
in a clean temporary overlay.  Live QTM transport, packet loss, and calibration
dataset tests remain subsequent work.

The live-transport foundation is now also present as
`mocap_toolbox_ros2/qtm_receiver`. It uses the upstream optional `qtm-rt` SDK
at runtime, selects one configured 6D rigid body by name, converts its explicit
millimetre input scale to metres, and sends a `PoseStamped` to the registration
node. Its pure packet/XML tests pass as part of the six-test mocap suite. A live
launch without the optional SDK was checked in an isolated ROS domain: it
correctly published a warning diagnostic naming the missing dependency and no
misleading pose. Real QTM connection, named-body verification, QTM timecode,
packet-loss, and calibration datasets remain hardware-stage evidence.

The SDK-facing QTM session is now separated from the ROS node and covered by a
fake-QTM connection test: it requests 6D settings, checks named-body discovery
before starting the stream, converts a deterministic packet, and reports
malformed packets without terminating the stream. The mocap package has **9
passed** fixture/transport tests and builds in a clean overlay. This is not QTM
timecode validation; the receiver continues to use receipt time until a tested
timecode policy is added.

The map-registration boundary now has a source-timestamp ordering gate. In its
default live configuration `mocap_pose_transformer` rejects a timestamp that
regresses beyond the configured tolerance, does not refresh freshness from that
packet, and reports the count/newest accepted stamp in diagnostics. Pure policy
tests cover normal, reordered, tolerated-small-regression, invalid-policy, and
header conversion cases; the mocap package now has **15 passed** tests. A
looping historical replay may explicitly disable this gate because recorded
timestamps restart; that option is not suitable for live localization.

## Keyence profile replay gate

`match_hardware_utilities/keyence_profile_ros2` is a new Jazzy package placed
beside, rather than inside, the old catkin Keyence packages.  It preserves the
print-facing ROS 1 output contract: `/profiles` is a one-row `PointCloud2`
containing `(x, 0, z)` points in metres; `/profiles_float` contains raw heights
with invalid returns as `NaN`; `/profiles_pitch_m` contains the lateral pitch
in metres.  It intentionally does not transform into `map`: registration and
contour processing are consumers owned by the AM repository.

The package has two pure fixture parsing tests and built in a clean temporary
Jazzy overlay.  An isolated live launch replayed the installed two-profile
fixture to `/keyence_test/profiles` and `/keyence_test/pitch`; the observed
cloud had the expected `keyence_frame`, three `float32` XYZ points, and the
observed pitch was `0.01 m`.  The first non-replay hardware task is an
LJ-X8400 vendor-protocol adapter with reconnect/error handling and a fake TCP
peer.  A replay source must never be represented as a live scanner driver.

The same package now contains `ljx8_profile_driver`, a ROS 2 port of the
LJ-X8400 polling path that loads the vendor Linux SDK only when the operator
explicitly supplies `library_path`. It maps the vendor 10-nanometre coordinate
unit and invalid height sentinel to the established SI profile contract. The
package now has **12 passed** profile/fake-SDK tests and builds in a clean Jazzy
overlay. The fake SDK validates the endpoint before calling the vendor API and
exercises open, optional measurement start, raw-buffer decoding, stop, close
cleanup, and stale-profile diagnostics. The live driver records profile receipt
using a monotonic clock; its diagnostics warn for a connected controller with
no initial profile or a profile older than `max_input_age`, expose
`profile_age_sec`, and rate-limit reconnect attempts using `reconnect_delay`.
Malformed vendor data is handled as a read failure and closes the client before
retrying. An isolated live launch without controller parameters verified a
warning diagnostic (`library_path and host parameters are required`) and no
misleading scanner data. A physical-controller compatibility, timing, and
calibration test is still mandatory before this driver can be used in a
printing profile.

## Contour monitoring gate

`print_path_monitoring` now has `contour_profile_monitor` and a pure estimator
for the Keyence `PointCloud2` profile contract. It compares a raw profile with
a recorded reference, reports signed lateral and height error only when there
is enough finite, non-flat overlap, and marks missing/invalid/stale input as a
warning. It publishes `/contour/lateral_error` and `/contour/height_error`.

Tests cover a known two-sample lateral shift, a known height shift with noise,
flat/missing/incompatible profiles, and direct decoding of the standard 12-byte
XYZ `PointCloud2` layout. The package test suite result is **12 passed**, and it
built in a clean temporary Jazzy overlay. An isolated graph check proved the
Keyence replay and contour-monitor nodes and their topics are present; direct
CLI sample observation in this sandbox remained inconclusive, so the first
future integration run must record the actual error topics from a rosbag or
native subscriber.

The subsequent offline correction foundation adds `contour_correction`, which
maps fresh lateral/height errors to a bounded world-Y/world-Z `Twist` at
`/contour/twist_world`. It starts disabled, contributes zero on a stale/missing
input or false `/start_condition`, and is the third input of the existing arm
twist combiner. The GUI now displays **Enable bounded contour correction**;
its persisted value and the launch argument both default to `false`, while a
live checkbox selection is used for the next arm-follower launch. The relevant
GUI command-construction suite passed **44 tests**, and a clean overlay build
of `print_path_monitoring`, `ur_trajectory_follower`, and `am_operator_gui`
succeeded on 2026-07-23. This verifies the offline command boundary only:
Keyence-to-world calibration, correction sign on the physical mounting, and
hardware-safe limits remain commissioning gates.

## Process safety-policy gate

`am_process_control` is a new AM-owned package that deliberately contains no
serial, foam, valve, or Dynamixel implementation.  Its policy produces zero by
default and only releases a rate-limited `/process/valve_target` after the
operator has armed and acknowledged it, printing is enabled, and flow feedback
is fresh.  Feedback timeout, a false start condition, or disarming causes an
immediate zero command.

Three pure tests cover the default-off/acknowledgement gate, stale-feedback and
stop handling, output rate limiting, and invalid target rejection.  The package
built in a clean temporary Jazzy overlay.  The next task is a mockable physical
adapter consuming this one output topic; it must be tested against a fake serial
endpoint before it is given access to any process hardware.

The current increment also ports the deployed Arduino flow protocol into
`flow_serial_bridge`: optional `FLOW,` prefix plus
`time_ms,channel,raw,voltage,current,percent,engineering` CSV samples. It
filters each channel, publishes left/right values and one configured safety
measurement, but has no actuator output. The parsing/mapping suite now has
**6 passing tests**, including data read from a pseudo-terminal, and the package
builds in a clean temporary Jazzy overlay. The remaining process task is a
ROS 2 Dynamixel/vendor adapter that consumes only `/process/valve_target`, with
a mock service test before it receives hardware access.

`dynamixel_valve_adapter` is now the guarded generic output layer: it accepts
only `/process/valve_target`, publishes the former
`servo_target_pos_left/right` `Int16` contract, starts disabled, and returns to
the configured closed positions if the target becomes stale. Its unit tests
cover disabled startup, stale-target closure, clamping, and invalid target
rejection; the process suite result is now **9 passed**. No Dynamixel hardware
or workbench service is installed on this machine, so binding those guarded
topics to a real vendor service and testing it with a mock remains required.
