# AM J-PARSE Controller

This package converts a Cartesian TwistStamped command into arm joint velocity
commands for every additive-manufacturing platform. Platform launch files supply
the robot description, joints, frames, and command topics.

The effective controlled point is tool0, followed by the fixed tool offset,
the nozzle tip, and the current spray distance along local +Z.

The controller adds the fixed transform to its KDL chain and applies the
current value from spray_distance_topic to the Jacobian on every update.
Consequently, the Cartesian command, measured deposition pose, and joint
velocity solution use the same deposition point.

`/am/jparse_ready` uses reliable transient-local QoS (the ROS 2 equivalent of
a ROS 1 latched status) and is published immediately on a state change, then
as a configurable 1 Hz heartbeat.  It is intentionally not republished at
the 500 Hz control rate.

Standalone platform launches keep their own native controllers. The AM launch
stacks select this package explicitly.
