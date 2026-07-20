# Bunker vs. Robotnik base-path accuracy

## Test setup

- Simulation: headless Gazebo, one isolated run per platform.
- Path: `map` frame, straight line from `(0, 0, 0)` to `(1, 0, 0)`.
- Sampling: 40 poses at 0.1 s intervals (39 moving segments), shared `/path_index`
  progress at 10 Hz.
- Bunker controller: pure pursuit with differential-drive mode.
- Robotnik controller: PID with its standard holonomic command interface, with
  lateral velocity held at zero for this straight path.
- Measurement: `/robot_pose` compared to `/base_trajectory_reference` using
  `trajectory_accuracy_monitor`.

The monitor uses simulation time, which can run faster than wall time in a
headless run. The values below therefore normalize the trial by first averaging
all samples at each moving waypoint, then calculating metrics across the 39
waypoints. Endpoint-only samples are excluded because both followers hold their
final reference after completing the path.

## Results

| Platform | Mean error | RMSE | P95 | Max |
| --- | ---: | ---: | ---: | ---: |
| Bunker (pure pursuit, diff drive) | 0.4949 m | 0.5054 m | 0.9050 m | 0.9306 m |
| Robotnik (PID) | 0.4766 m | 0.5063 m | 0.8970 m | 0.9478 m |

Robotnik has a 7.9 mm lower P95 error in this one-run comparison. Bunker has a
17.2 mm lower maximum error. The RMSE differs by less than 1 mm, so this trial
does not establish a material accuracy advantage for either platform.

Both trials completed with the intended controller mode and received the full
40-pose path. Repeat the paired trials several times before using these results
as a tuning decision; the existing accuracy reporter's three-runs-per-condition
rule is appropriate for that decision.
