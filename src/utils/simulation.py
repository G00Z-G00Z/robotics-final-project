from typing import Any
import numpy as np
from .matrices import *
import zmqRemoteApi

NO_ROTATION_MATRIX, _ = rotation_3d_deg(0, 0, 0)
NO_TRANSLATION_VEC = np.array([0, 0, 0])

"""Type alias for Any"""
Simulation = Any


def reset_arm(sim: Simulation, list_joint_ids: list[int], angle_deg: int = 0):
    """
    Resets the arm position to angle position.
    If angle not provided, resets to 0 deg
    """
    for j in list_joint_ids:
        sim.setJointTargetPosition(j, np.deg2rad(angle_deg))


def get_homogenous_mat_for_joint(theta: float, alpha: float, a: float, d: float):
    """
    Esto se lee como una rotacion $\theta$ en $z$, una translacion $d$ en $z$, translacion $a$ en $x$ y rotacion $\alpha$ en $x$,
    """
    rotation_z, _ = rotation_3d_deg(yau_z=theta)
    rotation_z = add_translation_to_rotation_3d(rotation_z, NO_TRANSLATION_VEC)
    rotation_x, _ = rotation_3d_deg(phi_x=alpha)
    rotation_x = add_translation_to_rotation_3d(rotation_x, NO_TRANSLATION_VEC)

    translation_x = add_translation_to_rotation_3d(
        NO_ROTATION_MATRIX, np.array([a, 0, 0])
    )
    translation_z = add_translation_to_rotation_3d(
        NO_ROTATION_MATRIX, np.array([0, 0, d])
    )

    H = rotation_z @ translation_z @ translation_x @ rotation_x
    return H
