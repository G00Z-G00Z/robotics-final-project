from typing import Any
import numpy as np
from .utils import window_iter
import contextlib
from time import sleep
from dataclasses import dataclass
from .matrices import (
    rotation_3d_deg,
    add_translation_to_rotation_3d,
    mul_homogenous_matrixes,
    get_homogenous_point_3d,
    mul_homogenous_matrixes,
)

NO_ROTATION_MATRIX, _ = rotation_3d_deg(0, 0, 0)
NO_TRANSLATION_VEC = np.array([0, 0, 0])

"""Type alias for Any"""
Simulation = Any


@dataclass
class Joint:
    _id: str
    _sim: Simulation

    _current_angle: float = 0

    def __init__(self, sim: Simulation, id: str):
        self._sim = sim
        self._id = id

    @property
    def position(self) -> np.ndarray:
        """The position property."""
        position = self._sim.getObjectPosition(self._id, self._sim.handle_world)
        return np.round(np.array(position), 4)

    @property
    def angle(self) -> float:
        """Gets the current angle"""
        return self._current_angle

    @angle.setter
    def angle(self, deg: float):
        self.set_angle(deg)

    @property
    def id(self) -> str:
        """id of the joint"""
        return self._id

    def set_angle(self, angle_deg: float):
        self._current_angle = angle_deg
        self._sim.setJointTargetPosition(self.id, np.deg2rad(angle_deg))

    def reset(self):
        self.set_angle(0)


class Link:
    """
    Represents a link in the arm
    """

    _prev_joint: Joint
    _next_joint: Joint
    _sim: Simulation

    _initial_homogeneous_matrix: np.ndarray

    def __init__(self, sim: Simulation, prev_joint: Joint, next_joint: Joint):
        self._prev_joint = prev_joint
        self._next_joint = next_joint
        self._sim = sim
        self._prev_joint.reset()
        self._next_joint.reset()
        self._initial_homogeneous_matrix = self.get_transformation_matrix()

    @property
    def prev_joint(self) -> Joint:
        return self._prev_joint

    @property
    def next_joint(self) -> Joint:
        return self._next_joint

    @property
    def initial_homogeneous_matrix(self) -> np.ndarray:
        return self._initial_homogeneous_matrix

    def get_transformation_matrix(self) -> np.ndarray:
        """
        Returns the transformation matrix between the previous joint and the next joint
        """
        return get_transformation_matrix_between_joins(
            self._sim, self.prev_joint.id, self.next_joint.id
        )


class RobotArm:
    _links: list[Link]

    def __init__(self, links: list[Link]):
        self._links = links

    def reset_arm(self):
        for link in self._links:
            link.prev_joint.reset()
            link.next_joint.reset()

    def get_predicted_position(
        self, angles_deg: list[float], initial_pos_arg: np.ndarray | None = None
    ) -> np.ndarray:
        """
        Returns the predicted position of the end effector
        It applies a z rotation on each joint, and then applies the initial transformation matrix to each link
        """
        initial_pos: np.ndarray = (
            np.array([0, 0, 0]) if initial_pos_arg is None else initial_pos_arg
        )

        assert len(initial_pos) == 3, "Initial position must be a 3d vector"

        assert len(angles_deg) == len(
            self._links
        ), "You must provide one angle per link"
        angles: list[float] = [np.deg2rad(angle) for angle in angles_deg]

        H_mats: list[np.ndarray] = []

        for link, angle in zip(self._links, angles):
            rotation_angle_m, _ = rotation_3d_deg(yau_z=angle)
            rotation_h_matrix = add_translation_to_rotation_3d(
                rotation_angle_m, NO_TRANSLATION_VEC
            )
            H_mats.append(rotation_h_matrix @ link.initial_homogeneous_matrix)

        predicted_position = np.block([0, 0, 0, 1])
        initial_translation = np.block([initial_pos, 1])
        H_total = mul_homogenous_matrixes(H_mats)

        return (H_total @ predicted_position) + initial_translation

    def set_position_arm(self, angles_deg: list[float]):
        """
        Sets the arm position to the angles provided
        """
        assert len(angles_deg) == len(
            self._links
        ), "You must provide one angle per link"
        angles: list[float] = [np.deg2rad(angle) for angle in angles_deg]

        for link, angle in zip(self._links, angles):
            link.prev_joint.angle = angle

        return self._links[-1].next_joint.position


@contextlib.contextmanager
def start_simulation(sim: Simulation):
    """
    Context manager for starting and stopping a simulation
    """

    try:
        sim.startSimulation()
        yield sim
    finally:
        sim.stopSimulation()
        sleep(1)


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


def get_transformation_matrix_between_joins(
    sim: Simulation, id_prev: str, id_next: str
):
    """
    Returns the transformation matrix between a pair of joints
    """
    transformation_matrix_flat = sim.getObjectMatrix(id_next, id_prev)
    transformation_matrix_incomplete = np.round(
        np.array(transformation_matrix_flat).reshape([3, 4]), 5
    )
    transformation_matrix_complete = np.block(
        [[transformation_matrix_incomplete], [np.array([0, 0, 0, 1])]]
    )
    return transformation_matrix_complete


def get_transformation_matrices_for_joint_list(
    sim: Simulation, join_id_list: list[str]
) -> list[np.ndarray]:
    """
    Travels the joints id and returns the transformation matrices between them
    """

    h_mats = [
        get_transformation_matrix_between_joins(sim, prev_id, next_id)
        for prev_id, next_id in window_iter(join_id_list, 2)
    ]

    return h_mats
