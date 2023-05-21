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
    mul_homogenous_matrixes,
)

NO_ROTATION_MATRIX, _ = rotation_3d_deg(0, 0, 0)
NO_TRANSLATION_VEC = np.array([0, 0, 0])

"""Type alias for Any"""
Simulation = Any


@dataclass
class Joint:
    """
    Represents a joint in the arm
    """

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
        """Gets the current angle in deg"""
        return self._current_angle

    @angle.setter
    def angle(self, deg: float):
        self.set_angle(deg)

    @property
    def id(self) -> str:
        """id of the joint"""
        return self._id

    def set_angle(self, angle_deg: float):
        """Moves the joint to a specific angle in deg"""
        self._current_angle = angle_deg
        self._sim.setJointTargetPosition(self.id, np.deg2rad(angle_deg))

    def reset(self):
        """Resets the joint to 0 deg"""
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
        """
        Reseets the arm to have all joints be 0 deg
        """
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
        ), f"You must provide one angle per link. There are {len(self._links)} links and you provided {len(angles_deg)} angles"

        H_mats: list[np.ndarray] = []

        for link, angle in zip(self._links, angles_deg):
            rot_z_mat, _ = rotation_3d_deg(yau_z=angle)
            rot_z_mat_h = add_translation_to_rotation_3d(rot_z_mat, NO_TRANSLATION_VEC)
            H_mats.append(rot_z_mat_h @ link.initial_homogeneous_matrix)

        # Put a 0 instead of a one to not carry over the sum
        initial_translation = np.block([initial_pos, 0])

        H_total = mul_homogenous_matrixes(H_mats)

        return (H_total @ np.array([0, 0, 0, 1])) + initial_translation

    def set_position_arm(self, angles_deg: list[float]):
        """
        Sets the arm position to the angles provided in deg
        """
        assert len(angles_deg) == len(
            self._links
        ), f"You must provide one angle per link. There are {len(self._links)} links and you provided {len( angles_deg )} angles"

        for link, angle in zip(self._links, angles_deg):
            link.prev_joint.set_angle(angle)

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
