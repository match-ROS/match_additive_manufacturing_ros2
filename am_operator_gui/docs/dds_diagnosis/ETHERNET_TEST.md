# Robot-side Ethernet DDS profile comparison, 2026-09-16

The supplied `fastdds_robot.xml` was loaded by the actual base bringup,
first with `SYNCHRONOUS`, then with `ASYNCHRONOUS`. Both variants mitigated
the previously observed long base stalls. Neither eliminated controller
overruns or DDS discovery receive losses. Asynchronous publication was not
consistently better: its single GUI-startup measurement was worse than the
synchronous profile measurement.

## Scope and verification

Only the base screen was gracefully stopped and restarted, sourcing the same
`/home/robot/ros_config.sh` as the normal bringup. Arm, devices, localization,
perception, navigation and behavior sessions retained their original settings.
The workstation and GUI retained their existing DDS environments. This is a
base-side transport experiment, not a full-robot or both-hosts DDS migration.

The base process environment contained:

```bash
FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_robot_diagnostic_candidate.xml
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=38
RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS # then ASYNCHRONOUS
```

`RMW_FASTRTPS_USE_QOS_FROM_XML` was unset. SHA-256 of both the robot temporary
file and local `fastdds_robot.xml`:
`908b4a0a851b485b1e6ef298e457e560ea05d7d25f3d1b8680c4a625807b2a30`.
The robot Ethernet address was `192.168.0.200/24` on `eno1`.
The profile uses UDP loopback/Ethernet allowlists with netmask filtering plus
local shared memory and retains multicast discovery. No single-PC peer
restriction was added. Connectivity from a third PC was not directly tested.

Both startups succeeded without the earlier duplicate-parameter exception.
All three base controllers were configured and active before measurement.
A separate 10-second connectivity check for each variant received about
500 `robot_odom -> robot_base_footprint` transforms, hundreds of base joint
states and thousands of arm joint states. Both controller list services
responded; the arm forward velocity controller remained active. No trajectory,
Following or move-to-start command was requested. The pre-test joint-state
check indicated stationary hardware; GUI movement processes were stopped.

## Results

Each probe measurement starts on the robot before an 18-second empty
domain-38 node is started on the workstation. It records 20 seconds of new
controller log entries and per-socket kernel UDP drops. Each variant has
three probe windows and one 20-second hardware GUI Launch All window with
Following disabled. GUI components are stopped after each GUI test.

| Configuration | Probe 1 / 2 / 3: maximum logged base cycle | GUI startup: maximum logged base cycle |
|---|---|---:|
| Ethernet profile, synchronous | 58 / 31 / 42 ms | 31 ms |
| Ethernet profile, asynchronous | No logged overrun / 34 ms / no logged overrun | 76 ms |

The desired base cycle is 20 ms. No logged overrun does not mean a zero-time
cycle or prove every cycle met its deadline. Earlier measurements without the
robot transport profile reached 1398 ms for an empty participant and 390 ms
for GUI startup with asynchronous publication alone; these are historical
comparisons, not interleaved identical-run A/B/A/B controls.

Base discovery-socket losses in the three probe windows:

- Synchronous profile: 61 / 31 / 33 packets/s.
- Asynchronous profile: 10 / 18 / 13 packets/s.

During GUI startup, base discovery losses were 25 packets/s synchronously
and 82 packets/s asynchronously. The unchanged arm driver's discovery socket
still lost 1591 and 1550 packets/s respectively. This experiment does not
validate an arm-side fix. Whole-interface Ethernet receive/transmit averages
were about 8.0/4.4 MB/s in the synchronous GUI window and 7.9/4.4 MB/s in the
asynchronous one; those counters include other traffic and cannot establish
GUI-only bandwidth or link saturation.

Received timestamp ages were not interpreted as pure communication latency:
an SSH midpoint clock comparison estimated the robot clock approximately
0.52–0.53 seconds behind the PC, with approximately 0.29–0.31 seconds round
trip and corresponding uncertainty. Clock synchronization is a separate
item to verify for TF consumers. The tests did not change either clock.

## Limits and final state

These are short stationary startup tests with process restarts, different DDS
participant identities and natural timing variation. They do not establish
sustained realtime behavior, prove the precise blocking call, or choose an
optimal publication mode from one GUI run per variant. The vendor TF
publication in the update path remains a design issue. The arm's unchanged
discovery losses still require investigation or a coordinated robot-wide test.

The original base environment was restored afterward: no publication-mode
override and no transport-profile file. All base controllers were verified
active, both controller list services responded, and TF/joint samples arrived.
GUI-managed processes were stopped and Following was idle. No persistent
bringup configuration, installed package or shell startup file was modified.

Raw results, startup environments, connectivity checks, clock samples and
probe discovery output: `ethernet_test_measurements.json`. Startup logs:
`ethernet_sync_startup.log` and `ethernet_async_startup.log`.
The orchestration scripts and experimental wrappers remain under `/tmp`.

Next, validate a consistent Ethernet profile across the remaining robot
bringup processes and repeat GUI startup measurements. Keep multicast for
other PCs on the Ethernet subnet. Continue the vendor controller fixes in
parallel with configuration validation; these results are mitigation evidence,
not a completed repair.
