# Arm move-to-start

`move_ur_to_path_idx` publishes Cartesian velocity commands at 100 Hz by default.
Translation and rotation each use an acceleration-limited ramp followed by a
finite moving average, which also limits jerk. Limits apply to the vector norm,
not separately to each Cartesian axis.

The standalone arm launch exposes these parameters:

| Parameter | Default | Unit |
| --- | ---: | --- |
| `publish_rate` | 100 | Hz |
| `max_linear_acceleration` | 0.10 | m/s² |
| `max_linear_jerk` | 0.40 | m/s³ |
| `max_angular_acceleration` | 0.30 | rad/s² |
| `max_angular_jerk` | 1.20 | rad/s³ |

The existing speed caps remain 0.12 m/s and 0.5 rad/s. The averaging window is
`2 * acceleration / jerk` (0.5 seconds with the defaults). Lower limits can
increase travel time and braking distance; they require validation for the
actual arm, tool, and feedback timing.

Translation brakes smoothly to zero before orientation alignment begins.
At arrival, both command vectors settle to zero before the node rechecks pose
tolerances and publishes completion. If braking carries the pose outside the
tolerance, alignment resumes. Completion describes the commanded stop and pose
check; it does not verify measured joint velocities.

Process interruption still sends an immediate zero command. The downstream
joint controller and its timeout behavior are unchanged. Cartesian smoothing
does not impose joint acceleration or jerk limits after the Jacobian mapping.
