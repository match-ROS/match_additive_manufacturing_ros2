# Live robot pose selection

The hardware pose adapter now owns the single /robot_pose output and world-to-robot
TF connection. Both GUI checkboxes apply live, independently, without stopping
followers or restarting pose adapters:

| Fallback: Base Pose | Odometry | Behavior |
| --- | --- | --- |
| Off | Off | Vicon base marker with marker-to-base calibration |
| Off | On | Vicon base; bridge missing measurements with odometry |
| On | Off | Vicon nozzle/TCP pose plus inverse base-to-nozzle kinematic chain |
| On | On | TCP-derived base; bridge missing measurements with odometry |

Odometrie is anchored to the last successful primary pose and its contemporary
odometry sample. The anchor is refreshed whenever a valid primary sample returns.
There is no initialization from the path or a fabricated initial pose. A gap is
recognized after the existing stale_timeout (default 0.5 s); each new valid odometry
sample then propagates relative motion from the anchor. While the primary source
is fresh, odometry only updates its internal state. Without an anchor or with invalid
or stale odometry, it cannot fill the gap. An odometry parent-frame change invalidates
the anchor. The odometry child frame is converted to the configured base frame via TF.

Selecting Fallback immediately ignores base-marker measurements, even if available.
The GUI wires its TCP input to `/vicon/nozzle_fallback` (TF frame
`vicon_nozzle_fallback`), calibrated separately by
`vicon_fallback_nozzle_transform`. The deposition chain continues to use
`/vicon/tool_transformed` and `vicon_nozzle_transform`. Both adapters consume the
same raw Vicon EE topic; only the deposition adapter publishes the marker TF.
Deselection resumes base-marker measurements. The existing anchor remains usable
while waiting for the newly selected primary source. Primary measurements are used
as measured upon return; their correction relative to drifted odometry may cause a
pose change. No smoothing or path-progress changes are introduced.

The first deployment requires restarting the GUI service and pose adapters once,
with motion stopped. Subsequent checkbox changes are live. No robot bringup restart
or new console entry point is required. The existing external_base_reference console
entry point starts LiveRobotPose; the original class remains available to standalone
code/tests. Static base-marker calibration is always started so switching back to
base tracking is possible.
