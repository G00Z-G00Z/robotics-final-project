#!/usr/bin/env python
from functools import reduce

import zmqRemoteApi
import numpy as np
from time import sleep
from pathlib import Path
import pandas as pd
from collections.abc import Callable
from functools import wraps


df_generated = pd.read_csv("../denavit-generated.csv")
df_online = pd.read_csv("../data/ur5-kinematics.csv")

print("Generated")
print(df_generated)
print("Online")
print(df_online)


def print_rounded(m):
    print(np.round(m, 4))


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

NO_ROTATION_MATRIX, _ = rotation_3d_deg(0, 0, 0)
NO_TRANSLATION_VEC = np.array([0, 0, 0])

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


def get_transformation_matrices(
    theta: np.ndarray, alpha: np.ndarray, a: np.ndarray, d: np.ndarray
):
    H_mats = [
        get_homogenous_mat_for_joint(theta=theta, a=a, alpha=alpha, d=d)
        for theta, alpha, a, d in zip(theta, alpha, a, d)
    ]

    return H_mats


H_mats_generated = get_transformation_matrices(
    df_generated["theta"], df_generated["alpha"], df_generated["a"], df_generated["d"]
)
H_mats_online = get_transformation_matrices(
    df_online["theta_[rad]"],
    df_online["alpha_[rad]"],
    df_online["a_[m]"],
    df_online["d_[m]"],
)
H_total = mul_homogenous_matrixes(H_mats_generated)


client = zmqRemoteApi.RemoteAPIClient()

sim = client.getObject("sim")

UR5_sim_obj_id = "/UR5"
base_id = sim.getObject(UR5_sim_obj_id)
no_joints = 6
joint_sim_names = [None for _ in range(6)]

joint_sim_names[0] = f"{UR5_sim_obj_id}/joint"

for i in range(1, len(joint_sim_names)):
    link_joint_str = "/link/joint" * i
    joint_sim_names[i] = f"{UR5_sim_obj_id}{link_joint_str}"

connection_name = f"{UR5_sim_obj_id}/connection"


joint_id = [sim.getObject(name) for name in joint_sim_names]
connection_id = sim.getObject(connection_name)


def reset_arm():
    """
    Resets the arm position to 0 deg
    """
    for j in joint_id:
        sim.setJointTargetPosition(j, np.deg2rad(0))


try:
    sim.startSimulation()
    reset_arm()

    print(sim.getObjectPosition(connection_id, sim.handle_world))
    angles = [0 for _ in range(6)]
    # angles[1] = 90
    angles = [np.deg2rad(a) for a in angles]
    assert len(angles) == no_joints

    for a, j in zip(angles, joint_id):
        sim.setJointTargetPosition(j, a)

    sleep(1)

    for idx, j in enumerate(joint_id):
        coord = sim.getObjectPosition(j, sim.handle_world)
        print(f"j{idx}: {np.round(coord,4)}")

    coord = sim.getObjectPosition(connection_id, sim.handle_world)
    print(f"cn: {np.round(coord,4)}")

    print("Final coordinate in simulation")
    print_rounded(coord)
    print("Final coordinate according to method")
    print_rounded(H_total[0:3, 3])

except Exception as e:
    sim.stopSimulation()
    raise e
finally:
    sim.stopSimulation()
