# Robot-side ASYNCHRONOUS test, 2026-09-16

`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` was tested in the actual
Robotnik base bringup environment. The observed stalls became shorter, but the
configuration alone is insufficient. This is a stationary, short-window test,
not a realtime guarantee or a validation of arm-side asynchronous publication.

## Startup and scope

`/home/robot/bringup.sh` sources `/home/robot/ros_config.sh`, which sources
`/home/robot/robot_settings/robot_params/bringup.env`. The script then kills all
screens and starts the base, devices, localization, perception, navigation,
manipulation and behavior sessions. Changing the workstation shell cannot
change the environment of these already-running robot processes.

For this experiment only the base launch was stopped using SIGINT and restarted
in a screen named `base`, sourcing the same robot configuration, then exporting
the publication-mode setting before `ros2 launch robot_bringup
base_complete.launch.py`. The controller process environment was checked. No
transport XML or XML QoS override was used. The arm driver and other bringup
sessions were not restarted. No motion command was requested. GUI Following
and move-to-start processes remained disabled. The pre-test workstation
stationary check received 959 joint-state messages with maximum absolute
reported velocity 0.00045; no command messages were received. A robot-local
check received no samples and was not accepted as evidence of stationary state.

## Measurements

Each empty-node test uses the previous 20-second remote measurement and
18-second workstation domain-38 probe. Controller activation was verified
before the three valid asynchronous repetitions. The robot was discovered.

| Configuration / trigger | Maximum logged base cycle |
|---|---:|
| Original environment, empty node | 1398 ms |
| Original environment, second window with additional stationary-check participant | 1818 ms |
| ASYNCHRONOUS, empty node, repetitions 1 / 2 / 3 | 381 / 53 / 35 ms |
| ASYNCHRONOUS, GUI Launch All with Following disabled | 390 ms |

The desired base cycle is 20 ms. During the asynchronous empty-node windows,
the base discovery socket still lost approximately 143–161 UDP packets/s.
During GUI startup the base discovery socket lost 1028 packets/s and the
unchanged arm driver's discovery socket lost 1075 packets/s. The whole robot
Ethernet interface averaged 5.8 MB/s receive and 2.7 MB/s transmit in that
window; those counters include other traffic and do not isolate GUI payloads.

The before/after comparison also includes a base process restart, changed DDS
participant identity and natural run variation. No full alternating A/B/A/B
trial was performed. The consistent shorter cycles in the three valid probe
windows support mitigation, not a precise percentage improvement or proof of
the exact blocking path. Discovery receive losses clearly remain.

## Additional startup defect

The first asynchronous base start aborted during configuration:

```text
rclcpp::exceptions::ParameterAlreadyDeclaredException
parameter 'base.back_left.traction.limits.acceleration.min' has already been declared
RobotnikController::parameters_timer_callback()
  -> RobotnikControllerParams::read(...)
  -> NodeParameters::declare_parameter(...)
```

The full report is in `base_async_startup.log`. No timing result from this
crashed controller is valid; a reported zero overrun count is not an improvement.
Restoring the original environment succeeded. A second asynchronous start,
with no test probes starting during initialization, also succeeded. The
duplicate declaration is therefore not proven to be caused by asynchronous
publication. Concurrent/repeated parameter initialization in the vendor
controller is a separate issue to investigate; the trace alone does not prove
the exact race. `base_async_retry_startup.log` records the successful retry.

## Outcome and next step

The test did not change `bringup.env`, `.bashrc` or any installed robot package.
GUI-managed components were stopped after the GUI test. The base was returned
to its original environment afterward. The remaining experimental start
wrappers and screen logs are under `/tmp` on the robot.

Do not treat ASYNCHRONOUS alone as the repair. Next compare the robot Ethernet
transport/netmask profile against this result while retaining multicast for
other Ethernet-subnet PCs. Independently fix the normal TF publication in the
vendor realtime update path and investigate the duplicate parameter declaration.
Any later persistent publication-mode setting belongs in `bringup.env` (or an
equivalent environment sourced by the actual launcher), after validation.

Raw measurements and validity notes: `async_test_measurements.json`.
