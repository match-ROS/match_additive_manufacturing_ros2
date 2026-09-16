# Arm joint acceleration limit

In Erweiterte Einstellungen → Arm-Geschwindigkeit und Beschleunigung,
J-PARSE Gelenkbeschleunigung sets `max_joint_acceleration` in rad/s² for all joints.
Settings are saved per platform. Restart the GUI service after updating its Python
code, rebuild am_jparse_controller, then restart Controllers to apply the setting.
Do this with Following and Move Arm To Start stopped.

Default: 0.5 rad/s² (approximately 28.6 degrees/s²). This is a conservative tuning
starting point, not a verified UR20 hardware maximum. Feasible acceleration depends
on payload, center of gravity, inertia and joint configuration. UR20 technical
specifications provide joint speed limits, not a universal joint acceleration limit:
https://www.universal-robots.com/manuals/EN/HTML/SW10_12_1/Content/prod-usr-man/complianceUR20/H_g5_sections/appendix_g5/tech_spec_UR20.htm

After inverse kinematics and joint velocity limiting, a common scale bounds the
increment of every joint velocity. Timing uses a monotonic clock, capped at one
nominal controller period so scheduler stalls do not permit a large command jump.
Start/restart ramps from zero. Debug achieved twist uses the limited velocities.
Explicit zero twist, command timeout, stale joints and computation failures publish
zero immediately and reset the limiter; these stops are exceptions to the acceleration
bound. This software limiter is not a robot safety function or a jerk limiter.

A 0.2 rad/s change takes at least 0.4 s at the default limit. Lower limits increase
tracking lag; verify deposition accuracy during controlled motion. Stale Vicon feedback
and path-index advancement are separate issues and are not changed here.
