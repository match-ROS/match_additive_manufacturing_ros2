This folder stores component-specific path and trajectory artifacts for the ROS 2 demos.

The legacy offline print-path generators are available under:

- `doubleCurvedTElement/print_path/`
- `rectangleRoundedCorners/print_path/`

They were copied from the ROS 1 repository because they only generate/read path
data and do not depend on `rospy`; their Python source is intentionally kept
unchanged. Use `ros2 launch parse_paths component_paths.launch.py` to convert
these series into the JSON trajectory files consumed by the ROS 2 path
loaders before starting the followers.

`robotnik_paired_demo/` is the default export location for the generated RB-VOGUI paired
base and arm test trajectories.

The folder also contains small helper scripts for post-processing the exported
trajectories, such as shifting the arm path in Z.
