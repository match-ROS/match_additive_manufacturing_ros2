#!/usr/bin/env python3

import json
import math


def quat_multiply(q1, q2):
    """
    Quaternion multiplication.
    Quaternions are given as [x, y, z, w].
    Returns q1 ⊗ q2.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def normalize_quaternion(q):
    norm = math.sqrt(sum(v * v for v in q))
    return [v / norm for v in q]


# Define your rotation angles in radians
angle_x = 0.0               # 0 degrees around X
angle_y = 0.0             # 0 degrees around Y
angle_z = math.pi / 2.0              # 0 degrees around Z

# 1. Quaternion for X-axis rotation
qx = [
    math.sin(angle_x / 2.0),
    0.0,
    0.0,
    math.cos(angle_x / 2.0)
]

# 2. Quaternion for Y-axis rotation
qy = [
    0.0,
    math.sin(angle_y / 2.0),
    0.0,
    math.cos(angle_y / 2.0)
]

# 3. Quaternion for Z-axis rotation
qz = [
    0.0,
    0.0,
    math.sin(angle_z / 2.0),
    math.cos(angle_z / 2.0)
]

q_rot = quat_multiply(qz, quat_multiply(qy, qx))  # ZYX order
q_rot = normalize_quaternion(q_rot)

input_file = "match_additive_manufacturing_ros2/components/bunker_paired_demo/base_path.json"
output_file = "match_additive_manufacturing_ros2/components/bunker_paired_demo/base_path_rotated.json"

with open(input_file, "r") as f:
    data = json.load(f)

for pose in data["poses"]:
    o = pose["orientation"]

    q_old = [
        o["x"],
        o["y"],
        o["z"],
        o["w"],
    ]

    # Rotate around the pose's LOCAL x-axis
    q_new = quat_multiply(q_old, q_rot)
    q_new = normalize_quaternion(q_new)

    pose["orientation"]["x"] = q_new[0]
    pose["orientation"]["y"] = q_new[1]
    pose["orientation"]["z"] = q_new[2]
    pose["orientation"]["w"] = q_new[3]

with open(output_file, "w") as f:
    json.dump(data, f, indent=2)

print(f"Saved rotated path to {output_file}")