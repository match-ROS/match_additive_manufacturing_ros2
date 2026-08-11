# Open TODOs

This is the authoritative, searchable list of open implementation work for this
repository. Search for `Open TODOs` or `TODO(ros1-migration)` when reviewing
outstanding work with Codex.

## ROS 1 benefits to retain selectively in ROS 2

The ROS 1 MiR follower combined indexing, timing, diagnostics, and control in one
node. ROS 2 should retain the useful behavior below as small, independently tested
components rather than porting that design wholesale. The historical rationale is
in [ros1_mir_trajectory_follower_analysis.md](ros1_mir_trajectory_follower_analysis.md).

### `TODO(ros1-migration):` Test index-aware completion for paths with repeated geometry

**Why:** A base path can pass through its final geometric pose before its shared arm/
base index reaches the last waypoint. Completion must require the final shared index,
not only pose tolerance. This is the useful ROS 1 "index exhausted" completion rule.

**Where to implement:** Keep the behavior in
`base_trajectory_follower/base_trajectory_follower/simple_base_follower.py`; add a
focused regression test in `base_trajectory_follower/test/` that uses an external
index and a path which revisits its final pose.

**Done when:** The follower continues publishing a non-zero correction before the
last index and reports completion only after both the final index and final-pose
tolerances are satisfied.

### `TODO(ros1-migration):` Make timestamp pacing an explicit coordinated-mode contract

**Why:** ROS 1 used path timestamps to pace base/arm progress. ROS 2 already has
`timestamp` and `desired_speed` progress modes in
`ur_trajectory_follower/ur_trajectory_follower/increment_path_index.py`; the missing
work is a documented, integrated contract proving that both base and arm consume the
same timestamp-paced index under the operator GUI.

**Where to implement:** Add integration tests and launch/profile parameters in
`ur_trajectory_follower`, then expose the selected progress mode through
`am_operator_gui`'s operator-service launch configuration. Keep velocity control in
`base_trajectory_follower`; do not re-create ROS 1's monolithic controller.

**Done when:** A timestamped paired path advances at its recorded timing in simulation,
the index is monotonic and identical for both followers, and the GUI shows which
progress mode is active.

### `TODO(ros1-migration):` Evaluate pose-confirmed arm progress as an alternative to timing

**Why:** ROS 1's `path_idx_advancer.launch` advances the master path index from the
measured global TCP pose, rather than from a timer. It supports radius, collinearity,
and virtual-line crossing metrics. This naturally holds the next arm reference when
the TCP lags; it publishes an index, goal pose, and normal, not an arm velocity.

**Where to implement:** Add a focused pose-progress component in
`ur_trajectory_follower` (or a mutually exclusive `measured_pose` mode of
`increment_path_index.py`). It should consume a frame-checked, fresh
`/current_deposition_pose` and submit progress through `/path_index_command`.
Only one component may own the master `/path_index` for a run; it must not run
alongside timestamp/desired-speed advancement.

**Done when:** Radius, segment/collinearity, and virtual-line crossing are isolated
unit-tested with degenerate, repeated, and three-dimensional path segments. Launch
tests prove stale pose input freezes progress, valid progress is monotonic and bounded,
and simulation shows the arm reference does not advance while the measured TCP is
behind it.

### `TODO(ros1-migration):` Add a bounded, optional base/arm index-skew adapter

**Why:** ROS 1 maps the arm-derived index through an offset vector, but keeps a
separate MiR waypoint index. That physical base index advances only after the base
reaches its waypoint. The controller uses the difference between the modified arm
reference and the reached base index to scale **base** velocity; it does not send an
index-offset velocity command to the arm or skip base waypoints.

**Where to implement:** Add a separate adapter in `ur_trajectory_follower` that
maps the master `/path_index` to a bounded `/base_progress_reference`. Extend
`base_trajectory_follower/simple_base_follower.py` with an opt-in
`velocity_sync` mode: retain its own physically reached waypoint anchor, consume the
base-progress reference only to calculate bounded phase/index error, and apply that
error to its base velocity. The adapter must not change the master index or arm
reference.

**Done when:** The base waypoint anchor advances only after its pose/tolerance rule is
satisfied; positive/negative index error changes base speed within configured limits;
the reference and base anchor are monotonic and bounded; stale input fails safely; and
unit tests cover repeated geometry, boundaries, and both signs of phase error.

### `TODO(ros1-migration):` Integrate moving-base TCP compensation in GUI operation

**Why:** The ROS 1 stack accounted for base motion while commanding the arm. ROS 2
already provides `ur_vel_induced_by_base` / `base_motion_compensation.py`, but the GUI
trajectory-following profile does not wire this correction into the arm command path.

**Where to implement:** Extend the MuR/RB-VOGUI operator profile in `am_operator_gui`
and `ur_trajectory_follower/launch/sideways_arm_control.launch.py`. Add a distinct
`compensation_base_frame` (mobile-base frame, not the arm command frame) and a fresh
base-velocity/odometry topic. Enable the existing `ur_vel_induced_by_base` plus
`combine_twists` pipeline: sum the world-frame arm-following, orientation/contour,
and negative base-induced TCP twists, then transform the result once into the arm
command frame before J-PARSE/controller output.

Use an additive combiner for these simultaneous Cartesian contributions, as ROS 1 did;
do not use a `twist_mux` to select between them. A mux/arbitrator remains appropriate
only for mutually exclusive control ownership such as E-stop, manual jog, or homing.
Add a final command-freshness/safety gate because the current combiner retains the last
message from each input.

**Done when:** Simulation shows materially lower TCP-to-path error during simultaneous
base and arm motion; all summed twists are expressed about the same TCP/nozzle point in
the same world frame; stale odometry/TF while base following is active latches a visible
fault and inhibits the final arm command; and launch tests cover the frame, sign,
lever-arm, and stale-input cases.

### `TODO(ros1-migration):` Promote tracking-error diagnostics into an explicit safety gate

**Why:** ROS 1 defined maximum tracking errors. ROS 2 currently measures error in
`print_path_monitoring` but does not make the operating decision explicit.

**Where to implement:** Keep error calculation in
`print_path_monitoring/trajectory_accuracy_monitor.py`. Add a separate safety/process
gate in `am_process_control` (or its successor) and display its state in
`am_operator_gui`; monitoring must not directly command robot motion.

**Done when:** Configurable position/orientation/time error limits produce a latched,
operator-visible fault, safely stop or inhibit following through a documented interface,
and are covered by unit and launch tests.

### `TODO(ros1-migration):` Restore optional layer-progress reporting as an operator feature

**Why:** ROS 1 published print-layer progress. This is useful operator feedback but
should not be embedded in the low-level follower.

**Where to implement:** Add a small progress-status publisher beside
`increment_path_index.py`, parameterized by path/layer metadata, and render it in
`am_operator_gui`.

**Done when:** It reports a validated layer number and percentage for layered paths,
falls back cleanly when metadata is absent, and cannot affect motion commands.

## Non-goals

Do not port ROS 1 constructor blocking, unstamped poses, controller-owned debug TF,
or its all-in-one timestamp/feedforward loop. ROS 2 components should remain
frame-aware, asynchronous, and individually testable.
