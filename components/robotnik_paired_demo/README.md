Default export folder for the RB-VOGUI paired base and arm demo.

When `rbvogui_paired_base_arm_demo.launch.py` runs with generated test paths and
`export_trajectories:=true`, it writes:

- `base_path.json`
- `arm_path.json`
- `normal_vector.json`

On a later run, pass `use_exported_trajectories:=true` to publish these files instead
of regenerating the test paths from the current poses.

If you want to retime the exported paths for a different linear speed, run
`retime_robotnik_paired_demo_paths.py` from this directory. By default it rewrites
`base_path.json` and `arm_path.json` in place to `0.1 m/s`.
