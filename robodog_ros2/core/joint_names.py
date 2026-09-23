"""
Canonical joint naming convention.

Prevents index-shifting bugs between the legacy 3x4 NumPy joint-angle
matrices (axis x leg), the ROS 2 sensor_msgs/JointState arrays, and the
URDF. Every other module should go through the conversion helpers here
rather than hand-rolling index math.

Legacy matrix convention (unchanged from the original codebase):
    rows (axis_index):    0 = abduction, 1 = hip, 2 = knee
    columns (leg_index):  0 = front-right, 1 = front-left,
                           2 = back-right, 3 = back-left
"""
import numpy as np

LEG_NAMES = ["front_right", "front_left", "back_right", "back_left"]
AXIS_NAMES = ["abduction", "hip", "knee"]

NUM_LEGS = len(LEG_NAMES)
NUM_AXES = len(AXIS_NAMES)

# Canonical, ordered list of the 12 joint names, e.g.:
# "leg_front_right_abduction", "leg_front_right_hip", "leg_front_right_knee",
# "leg_front_left_abduction", ...
JOINT_NAMES = [
    f"leg_{leg}_{axis}" for leg in LEG_NAMES for axis in AXIS_NAMES
]

assert len(JOINT_NAMES) == 12


def matrix_to_joint_list(joint_angles: np.ndarray):
    """Flatten a (3, 4) [axis, leg] joint-angle matrix into a length-12
    list ordered to match JOINT_NAMES (leg-major, axis-minor).

    Parameters
    ----------
    joint_angles : numpy array (3, 4)

    Returns
    -------
    list[float] of length 12, aligned with JOINT_NAMES
    """
    values = []
    for leg_index in range(NUM_LEGS):
        for axis_index in range(NUM_AXES):
            values.append(float(joint_angles[axis_index, leg_index]))
    return values


def joint_list_to_matrix(values):
    """Inverse of matrix_to_joint_list: turn a length-12 sequence ordered
    per JOINT_NAMES back into a (3, 4) [axis, leg] matrix.
    """
    assert len(values) == 12, f"Expected 12 joint values, got {len(values)}"
    matrix = np.zeros((NUM_AXES, NUM_LEGS))
    idx = 0
    for leg_index in range(NUM_LEGS):
        for axis_index in range(NUM_AXES):
            matrix[axis_index, leg_index] = values[idx]
            idx += 1
    return matrix


def joint_index(leg_index: int, axis_index: int) -> int:
    """Index into a JOINT_NAMES-ordered flat list for a given (leg, axis)."""
    return leg_index * NUM_AXES + axis_index