import math
from typing import Iterable

import numpy as np


# Kinematics helper functions
def x_rotation(theta: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the x-axis."""
    return np.array([
        [1, 0, 0],
        [0, math.cos(theta), -math.sin(theta)],
        [0, math.sin(theta), math.cos(theta)]])


def y_rotation(theta: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the y-axis."""
    return np.array([
        [math.cos(theta), 0, math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0, math.cos(theta)]])


def z_rotation(theta: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the z-axis."""
    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta), math.cos(theta), 0],
        [0, 0, 1]])


def get_extrinsic_rotation(rpy) -> np.ndarray:
    """Gets the extrinsic rotation matrix defined by roll about x, then pitch about y, then yaw
    about z. This is the rotation matrix used in URDF.

    Args:
        rpy: A numpy array containing the roll, pitch, and yaw angles in radians.

    Returns:
        A 3x3 numpy array for the resulting extrinsic rotation.
    """
    x_rot = x_rotation(rpy[0])
    y_rot = y_rotation(rpy[1])
    z_rot = z_rotation(rpy[2])
    return z_rot @ y_rot @ x_rot


def inv_tf(tf: np.ndarray) -> np.ndarray:
    """Get the inverse of a homogeneous transform"""
    inv_tf = np.eye(4)
    inv_tf[0:3, 0:3] = np.transpose(tf[0:3, 0:3])
    inv_tf[0:3, 3] = -1.0 * np.transpose(tf[0:3, 0:3]) @ tf[0:3, 3]
    return inv_tf


def get_dh_frame(dh_params: Iterable[float]) -> np.ndarray:
    """Get the tf for the given dh parameters.

    The DH parameters are, in order:
     - d Translation along the original z axis.
     - theta Rotation about the original z axis, in radians.
     -       The actuation of the joint must be added to this parameter for
             revolute joints.
     - r Translation along the new x axis (sometimes called `a`).
     - alpha Rotation about the new x axis, in radians.

    """
    d = dh_params[0]
    theta = dh_params[1]
    r = dh_params[2]
    alpha = dh_params[3]

    dh_frame = np.eye(4)

    dh_frame[0, 0] = math.cos(theta)
    dh_frame[0, 1] = -math.sin(theta) * math.cos(alpha)
    dh_frame[0, 2] = math.sin(theta) * math.sin(alpha)
    dh_frame[0, 3] = r * math.cos(theta)

    dh_frame[1, 0] = math.sin(theta)
    dh_frame[1, 1] = math.cos(theta) * math.cos(alpha)
    dh_frame[1, 2] = -math.cos(theta) * math.sin(alpha)
    dh_frame[1, 3] = r * math.sin(theta)

    dh_frame[2, 1] = math.sin(alpha)
    dh_frame[2, 2] = math.cos(alpha)
    dh_frame[2, 3] = d
    return dh_frame
