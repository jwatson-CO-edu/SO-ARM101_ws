"""Helpers for turning a flattened 4x4 homogeneous transform into a validated PoseStamped.

Kept free of rclpy/Node dependencies (just numpy + geometry_msgs) so it's easy to unit
test or reuse from a plain script.
"""

from typing import Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped


def is_valid_homogeneous_transform(matrix: np.ndarray, tol: float = 1e-3) -> Tuple[bool, str]:
    """Check that `matrix` is a proper 4x4 rigid-body transform: [[R, t], [0, 0, 0, 1]]
    with R a valid rotation (orthonormal, determinant +1 - not a reflection).

    Returns (is_valid, reason); `reason` is empty when valid. Rejecting bad input here,
    before it ever reaches MoveIt, turns a confusing downstream IK/planning failure into a
    clear, specific error message back to the action client.
    """
    if matrix.shape != (4, 4):
        return False, f"expected a 4x4 matrix, got shape {matrix.shape}"

    # A homogeneous transform's bottom row is fixed by construction; if it isn't
    # [0, 0, 0, 1] the input likely isn't a homogeneous transform at all (e.g. a raw
    # rotation matrix, or a transform with an unnormalized scale term).
    bottom_row = matrix[3, :]
    if not np.allclose(bottom_row, [0.0, 0.0, 0.0, 1.0], atol=tol):
        return False, f"bottom row must be [0, 0, 0, 1], got {bottom_row.tolist()}"

    # A rotation matrix is orthonormal: its columns are unit length and mutually
    # perpendicular, so R @ R.T == I. This catches non-rigid transforms (e.g. one with
    # scaling or shear baked into the upper-left 3x3 block).
    rotation = matrix[:3, :3]
    should_be_identity = rotation @ rotation.T
    if not np.allclose(should_be_identity, np.eye(3), atol=tol):
        return False, "rotation block is not orthonormal (R @ R.T != I)"

    # Orthonormal with determinant -1 is a reflection, not a rotation (e.g. built by
    # mistake by negating one axis of an otherwise-valid rotation matrix).
    det = np.linalg.det(rotation)
    if not np.isclose(det, 1.0, atol=tol):
        return False, f"rotation block is not a proper rotation (det(R) = {det:.4f}, expected +1)"

    if not np.all(np.isfinite(matrix)):
        return False, "matrix contains non-finite values"

    return True, ""


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Shepperd's method: 3x3 rotation matrix -> (x, y, z, w) quaternion.

    Picks whichever of the four cases (trace, or largest diagonal entry) keeps the
    denominator `s` well away from zero, which naive formulas built from a single
    case (e.g. always dividing by trace) can hit for 180-degree rotations.
    """
    trace = np.trace(rotation)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    return float(x), float(y), float(z), float(w)


def flat_pose_to_matrix(pose_matrix_flat) -> np.ndarray:
    """Row-major float64[16] (as sent over the MoveToPose action) -> 4x4 ndarray.

    "Row-major" matters here: reshape(4, 4) fills the array row by row, so element 3 of
    the flat array is matrix[0, 3] (the X translation), not matrix[3, 0]. A column-major
    caller would silently get a transposed - and wrong - matrix with no error raised.
    """
    return np.array(pose_matrix_flat, dtype=float).reshape(4, 4)


def matrix_to_pose_stamped(matrix: np.ndarray, frame_id: str, stamp=None) -> PoseStamped:
    """4x4 homogeneous transform -> geometry_msgs/PoseStamped in `frame_id`.

    Does not validate `matrix`; call is_valid_homogeneous_transform() first.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    if stamp is not None:
        pose_stamped.header.stamp = stamp

    # Translation is just the last column of the top 3 rows.
    pose_stamped.pose.position.x = float(matrix[0, 3])
    pose_stamped.pose.position.y = float(matrix[1, 3])
    pose_stamped.pose.position.z = float(matrix[2, 3])

    # ROS poses use quaternions, not rotation matrices, so the upper-left 3x3 block has to
    # be converted before MoveIt can consume it as a goal pose.
    x, y, z, w = quaternion_from_rotation_matrix(matrix[:3, :3])
    pose_stamped.pose.orientation.x = x
    pose_stamped.pose.orientation.y = y
    pose_stamped.pose.orientation.z = z
    pose_stamped.pose.orientation.w = w

    return pose_stamped
