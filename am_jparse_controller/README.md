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

Standalone platform launches keep their own native controllers. The AM launch
stacks select this package explicitly.
