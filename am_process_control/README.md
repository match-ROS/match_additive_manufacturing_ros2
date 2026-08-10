# AM process control

This package is the mandatory, fail-closed policy boundary before any foam,
valve, serial, or Dynamixel hardware adapter.  It outputs zero by default and
only ramps toward `/process/target` when all of these are true:

- `/process/armed` is true;
- an explicit `/process/acknowledged` is true;
- `/start_condition` is true; and
- `/process/flow_measurement` is fresh.

The output is `/process/valve_target` (`std_msgs/msg/Float32`).  A future
hardware adapter may consume that topic but must not expose a second direct
actuator command path.  Any disarm, stale feedback, or stop signal commands
zero immediately; normal rising/falling target changes are rate limited.

`flow_serial_bridge` ports the deployed Arduino stream. It accepts optional
`FLOW,` prefixed rows in the form
`time_ms,channel,raw,voltage,current,percent,engineering`, low-pass filters
each channel, and publishes left/right values plus one configured
`/process/flow_measurement` safety channel. It never drives an actuator.

`dynamixel_valve_adapter` is the only generic motor-output bridge. It accepts
only `/process/valve_target`, publishes the legacy `servo_target_pos_left` and
`servo_target_pos_right` `Int16` goal topics, starts disabled, and commands the
configured closed positions whenever input is stale. A physical Dynamixel
workbench/service adapter can consume those topics during commissioning.
