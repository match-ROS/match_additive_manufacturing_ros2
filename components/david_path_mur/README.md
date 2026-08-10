# David path for MuR620

This is the MuR620 differential-drive variant of `../david_path`.  Its
`base_path.json` uses the forward XY path tangent as each pose orientation;
`arm_path.json` is shifted by `(0.4, -0.4, 0.0)` m in the exported David
frame and then scaled by `0.75` about each paired base waypoint. This keeps
the trajectory in the practical workspace of the MuR left arm while the base
path remains unchanged. Tool orientations and the normal vector are unchanged.

Regenerate it with:

```bash
python3 components/generate_trajectory_from_points_of_david.py \
  --targets components/david_path/260617_1621_tcp_planes.json \
  --base-positions components/david_path/260618_1131base_footprint_base_positions.json \
  --output-dir components/david_path_mur \
  --arm-offset-xyz 0.4,-0.4,0.0 \
  --arm-relative-scale 0.75 \
  --base-yaw-from-path
```
