"""
transforms.py
-------------
Homogeneous transformation utilities ported from:
  rotate.m, trans.m, hom2vect.m
"""

import numpy as np


# Named axis constants (1-based like MATLAB, kept for readability)
X_AXIS = 1
Y_AXIS = 2
Z_AXIS = 3


def rotate(axis: int, angle: float) -> np.ndarray:
    """
    Build a 4x4 homogeneous rotation matrix around the given axis.

    Parameters
    ----------
    axis  : X_AXIS (1), Y_AXIS (2), or Z_AXIS (3)
    angle : rotation angle in radians

    Returns
    -------
    4x4 numpy array
    """
    c, s = np.cos(angle), np.sin(angle)
    if axis == X_AXIS:
        return np.array([
            [1,  0,  0, 0],
            [0,  c, -s, 0],
            [0,  s,  c, 0],
            [0,  0,  0, 1],
        ], dtype=float)
    elif axis == Y_AXIS:
        return np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1],
        ], dtype=float)
    elif axis == Z_AXIS:
        return np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=float)
    else:
        raise ValueError(f"axis must be 1, 2, or 3; got {axis}")


def trans(x: float, y: float, z: float) -> np.ndarray:
    """
    Build a 4x4 homogeneous translation matrix.

    Parameters
    ----------
    x, y, z : translation components in mm

    Returns
    -------
    4x4 numpy array
    """
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def hom2vect(matrix_hom: np.ndarray) -> np.ndarray:
    """
    Extract the translational part of a 4x4 homogeneous matrix as a 3-vector.

    Parameters
    ----------
    matrix_hom : 4x4 homogeneous transformation matrix

    Returns
    -------
    shape-(3,) numpy array  [x, y, z]
    """
    if matrix_hom.shape != (4, 4):
        raise ValueError("Input must be a 4x4 matrix.")
    return matrix_hom[:3, 3].copy()
