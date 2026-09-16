# Robotnik DDS / GUI startup diagnosis, 2026-09-16

Follow-up: robot-side asynchronous publication was subsequently tested on the
base bringup. It reduced the observed stalls but did not fix them; see
[ASYNC_TEST.md](ASYNC_TEST.md). The earlier statements below about no hardware
restart describe the original investigation, before that follow-up.

The robot-side Ethernet transport profile was then tested both synchronously
and asynchronously. Both mitigated the observed base stalls, but neither
eliminated overruns or discovery losses; see [ETHERNET_TEST.md](ETHERNET_TEST.md).

Latest validation: nine rotated-order GUI startup trials, including three
original-environment controls, are documented in
[REPEATED_LAUNCH_ALL.md](REPEATED_LAUNCH_ALL.md). They confirm the Ethernet
profile's mitigation and do not establish a clear asynchronous advantage.

Subsequent robot-wide test using `./bringup.sh`: the synchronous Ethernet
profile mitigated base stalls but worsened the observed arm overrun counts
and did not repair its UDP losses. See [FULL_BRINGUP_TEST.md](FULL_BRINGUP_TEST.md).

Missing-description follow-up: the full publisher had aborted on an invalid
Lift/Arm URDF attachment. Both controlled normal and base-profile starts
delivered `/robot/robot_description` to the workstation. Different installed
Lift Xacro versions were identified; see
[DESCRIPTION_START_COMPARISON.md](DESCRIPTION_START_COMPARISON.md).

## Result and limits

Starting a ROS participant in domain 38 reproducibly disrupts the running
Robotnik base controller even when the new node has no application subscriptions.
This is not explained by the GUI subscribing to camera images or point clouds.
The controller's desired period is 20 ms; measured logged update overruns reached
1–2.7 seconds during GUI startup and repeated minimal-node tests.

There are two concrete implementation/configuration problems:

1. The installed Robotnik base controller publishes TF directly from its update
   call chain. A normal ROS publication is not realtime-safe. This is a plausible
   blocking path consistent with the update-time logs, not a sampled stack trace
   proving which mutex or system call caused each observed delay.
2. The robot advertises its Docker bridge address `172.18.0.1` to other computers.
   The workstation owns the same address. A traced Cyclone DDS client selected
   that address and sent discovery replies to its own host. This wrong-address
   selection was directly observed. It does not, by itself, prove the cause of
   the Fast DDS controller's long update times.

No tested workstation-only change fully repaired the controller stalls. A
change to the running robot controller environment or controller implementation
is needed for the next validation. The candidate XML files here are **not
automatically loaded**, and are **not a verified production fix**.

## Measured comparisons

Both computers run Jazzy / Fast DDS 2.14.5 / rmw_fastrtps_cpp 8.4.3
(package build dates differ). The robot has Cyclone DDS 0.10.5 and
rmw_cyclonedds_cpp 2.2.3 installed, but its hardware processes run Fast DDS.
Cyclone client tests used copies of those libraries under `/tmp/cyclone_probe`
on the workstation; no system package installation or middleware switch was made.

The refined tests start a remote 20-second measurement before launching the
workstation probe. The probe disables rosout and parameter services, subscribes
to no application topics, and runs for 18 seconds. Internal ROS graph discovery
still exists. The measurement counts additions to controller logs and kernel UDP
socket drop counters. Network counters cover the whole interface, not just GUI
traffic. Full results and probe discovery output are in `measurements.json`.

| Workstation probe | Maximum logged base cycle | Robot discovered? | Result |
|---|---:|---|---|
| Fast DDS default, repetitions 1/2/3 | 1376 / 1700 / 1878 ms | Yes | Reproduces fault |
| Fast DDS Ethernet allowlist | 1211 ms | Yes | Insufficient |
| Fast DDS Ethernet + netmask filter, repetitions 1/2/3 | 53 / 969 / 804 ms | Yes | Insufficient; first good result did not repeat |
| Cyclone default | No base overruns logged | No | Invalid as a fix: no robot connectivity |
| Cyclone explicit Ethernet + robot peer | No base overruns logged | No | Wrong remote Docker locator selected |
| Cyclone Ethernet + `DontRoute=true` | 1511 ms | Yes | Connectivity repaired, robot controller stalls remain |

Earlier controls: an empty node in domain 39 produced no controller overruns or
UDP drops in the eight-second window. The same node in domain 38 produced a
1554 ms base cycle. After stopping GUI-managed/test processes, the final
eight-second window had no new controller overruns or UDP drops. Absence of a
logged overrun is not a measurement of every cycle and is not an RT guarantee.

During the original GUI test, all data-processing components were started
cumulatively with Following disabled and no move-to-start process. Vicon and
RViz were already off. Starting pose adapters alone produced 687 ms base cycles;
after adding all components the maximum was 2720 ms. This demonstrates startup
disruption, not a controlled measurement of steady-state CPU load per component.

## Exact controller call chain

The robot loads `/opt/ros/jazzy/lib/librobotnik_controllers.so`, owned by
`ros-jazzy-robotnik-controllers`. Static symbol/disassembly inspection shows:

```text
robotnik_controllers::RobotnikController::update(...)
  -> Odometry::compute_and_publish_odometry(...)
  -> Odometry::publish_odometry()
  -> tf2_ros::TransformBroadcaster::sendTransform(...)
```

In the inspected binary, `update` calls `compute_and_publish_odometry` at
`0x1007c3`, that function calls `publish_odometry` at `0x163938`, and
`publish_odometry` jumps directly to `sendTransform` at `0x162602`.
The odometry-message publication uses a `RealtimePublisher` with `trylock`;
the TF call follows directly and is not handed to that publisher's worker thread.
Addresses identify this inspected binary only; they are not patch locations.

Controller log examples separate hardware read/update/write times. The large
base delays are in **update**, while read/write take tens of microseconds.
The arm also recorded an RTDE pipeline overflow earlier and a failed ROS service
response. Both hardware managers logged failure to acquire FIFO RT scheduling.
These facts do not establish that every arm symptom has the same cause.

## Preferred durable fix: decouple TF publication

Fix `robotnik_controllers::Odometry::publish_odometry()` in the vendor source:

- Create a `realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>` outside
  the update loop, keeping the current TF topic, remappings and QoS.
- Preallocate its single-transform message and frame strings during configure.
- In update, use a nonblocking `trylock`/`try_publish` path, copy the current
  stamp and numeric transform, and hand publication to the non-RT worker.
- If the worker is busy, skip that telemetry sample; never wait in update.
- Ensure the direct `TransformBroadcaster::sendTransform()` call no longer runs
  in the realtime update chain. Cleanly stop the publisher worker at teardown.

The vendor source is not present in the inspected workspace. Do not binary-patch
the installed `.so`. Apply the source fix through a vendor update or a matching
source checkout and rebuild. This corrects a realtime design issue even if DDS
discovery remains expensive. It has not yet been deployed or runtime-validated.

## Immediate configuration experiment on the robot

The supplied `fastdds_robot.xml` and `fastdds_workstation.xml` retain multicast
discovery, constrain UDP to loopback and the robot Ethernet subnet, retain local
shared memory, and filter out remote locators outside those subnets. They do not
limit communication to a single remote PC. Other PCs on `192.168.0.0/24` can
still participate with the same domain. Routed/Wi-Fi-only peers need a deliberate
additional interface/routing configuration.

The workstation XML parsed and discovered the robot during the measured tests.
The robot XML also parsed successfully on the robot with asynchronous publishing
in an isolated domain-39 validation node; no hardware driver used that profile.
The robot XML is intended for `192.168.0.200`, the workstation one for
`192.168.0.222`; edit the local address when using another machine.

For a controlled robot bringup test, set these in the environment that actually
starts the robot drivers (a service may not read `.bashrc`):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=38
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTRTPS_DEFAULT_PROFILES_FILE=/absolute/path/fastdds_robot.xml
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
```

On the GUI workstation use the workstation XML. An optional
`ROS_STATIC_PEERS='192.168.0.200'` can coexist with multicast. A static peer does
not solve discovery load or discard the robot's incorrect advertised addresses.

`ASYNCHRONOUS` moves data sending out of the publisher's calling thread. It is a
mitigation to test, **not proof of hard realtime safety**: enqueueing, allocation
and DDS synchronization can still matter. Do not add `RMW_FASTRTPS_USE_QOS_FROM_XML=1`
to these profiles: they do not specify publisher history/memory policies.
The candidate does not enable UDP `non_blocking_send`, which can silently drop
outgoing datagrams under pressure.

These changes require restarting the affected robot processes. No robot driver
was restarted during this investigation. Do not resume queued motion while
doing the change. Keep Following and move-to-start off through validation.

Separately fix the robot user's RT scheduling permissions according to the
ros2_control instructions below. Verify that a newly started manager actually
obtains FIFO scheduling; permissions alone do not fix a blocking publish call.
Larger UDP buffers may reduce drops but also do not remove this blocking path.

If configuration mitigation is insufficient and the vendor fix is not yet
available, test a consistent robot-side middleware change or Fast DDS Discovery
Server deployment. A PC-only Cyclone switch was explicitly tested and did not
resolve the stall once connectivity was correct. Those broader changes need
their own interoperability and timing validation; they are not proven fixes here.

## Validation before calling it repaired

1. Keep all motion commands disabled. Confirm controller services, fresh joint
   states and `robot_odom -> robot_base_footprint -> robot_arm_base_link` TF.
2. Repeat the empty-domain-38 probe at least three times while recording robot
   controller update overruns and per-socket UDP drops. Also test several new
   participants starting together, as Launch All does.
3. Start GUI components in the tested order without Following. Verify no
   seconds-long base stalls and reliable controller service responses.
4. Check sustained TF/joint-state freshness, not only endpoint existence or a
   previously latched `ready=true`. Validate the 50 Hz base and 500 Hz arm timing
   requirements with appropriate instrumentation; a short clean log is not enough.
5. Verify multicast discovery from another PC on the robot subnet. Only after
   these stationary checks should normal operator-controlled motion testing resume.

## Sources

- [ros2_control realtime_tools: normal ROS publishing is not realtime safe](https://control.ros.org/jazzy/doc/realtime_tools/doc/index.html)
- [rmw_fastrtps publication modes](https://github.com/ros2/rmw_fastrtps#change-publication-mode)
- [Fast DDS 2.14 interface/netmask filtering](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/interfaces.html)
- [Fast DDS default listening-port mapping](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html)
- [Open report: existing-node CPU spikes when a new node starts](https://github.com/ros2/rmw_fastrtps/issues/741) — similar symptom, not proof of identical root cause.
- [Cyclone interface selection and DontRoute](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)
- [ros2_control FIFO permissions and determinism](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
