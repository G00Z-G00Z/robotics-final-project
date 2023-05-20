import numpy as np
from collections.abc import Callable
from functools import wraps, reduce


def deg_args(f: Callable):
    """
    Transforms all the arguments, assuming to be deg, to rads
    """

    @wraps(f)
    def wrapper(*args, **kwargs):
        new_args = [np.deg2rad(arg) for arg in args]
        new_kwargs = {key: np.deg2rad(value) for key, value in kwargs.items()}
        return f(*new_args, **new_kwargs)

    return wrapper


@deg_args
def rotation_3d_deg(yau_z: float = 0, theta_y: float = 0, phi_x: float = 0):
    """
    Returns the rotation matrix in 2d, given the angles
    The angles are calculated assuming the axis is the axis of rotation
    """
    R_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(phi_x), -np.sin(phi_x)],
            [0, np.sin(phi_x), np.cos(phi_x)],
        ]
    )

    R_y = np.array(
        [
            [np.cos(theta_y), 0, np.sin(theta_y)],
            [0, 1, 0],
            [-np.sin(theta_y), 0, np.cos(theta_y)],
        ]
    )

    R_z = np.array(
        [
            [np.cos(yau_z), -np.sin(yau_z), 0],
            [np.sin(yau_z), np.cos(yau_z), 0],
            [0, 0, 1],
        ]
    )

    return R_z @ R_y @ R_x, {"R_z": R_z, "R_y": R_y, "R_x": R_x}


def add_translation_to_rotation_3d(rotation_3d: np.ndarray, p: np.ndarray):
    return np.block([[rotation_3d, np.reshape(p, (3, 1))], [np.zeros((1, 3)), 1]])


def get_homogenous_point_3d(p: np.ndarray):
    return np.block([p, 1])


def mul_homogenous_matrixes(transform_matrices: list[np.ndarray]) -> np.ndarray:
    """
    Multiplies a list of homogeneous matrixes, from right to left
    """
    return reduce(lambda prev, next: next @ prev, transform_matrices[::-1])
