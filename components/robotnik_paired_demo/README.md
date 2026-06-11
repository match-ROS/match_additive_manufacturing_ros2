Default export folder for the RB-VOGUI paired base and arm demo.

When `rbvogui_paired_base_arm_demo.launch.py` runs with generated test paths and
`export_trajectories:=true`, it writes:

- `base_path.json`
- `arm_path.json`
- `normal_vector.json`

On a later run, pass `use_exported_trajectories:=true` to publish these files instead
of regenerating the test paths from the current poses.
