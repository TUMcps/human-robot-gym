"""This file adds a subclass to the Rethink gripper with a valid init_qpos.

Supposedly due to a bug in robosuite, the Rethink gripper is initialized with a gripper joint angle
that is outside of the actual joint limits. This causes the gripper to snap to valid joint values
in the first steps after initialization.

This subclass fixes this issue by setting the init_qpos to a valid joint angle.

Author:
    Felix Trost (FT)

Changelog:
    27.3.23 FT Created file
"""
import numpy as np
import traceback

from robosuite.models.grippers import RethinkGripper


class RethinkValidGripper(RethinkGripper):
    """Rethink gripper with valid init_qpos."""
    @property
    def init_qpos(self):
        """Override parent property by choosing a value within the actual joint limits."""
        traceback.print_stack()
        return np.array([0.011499, -0.011499])

    @property
    def qpos_range(self) -> np.ndarray:
        """Range of the gripper joint positions.

        Returns:
            np.ndarray: Range of the gripper joint positions. 2D array of shape (2, 2):
                qpos_range[0, :] = minimal qpos values for each joint
                qpos_range[1, :] = maximal qpos values for each joint
        """
        return np.array(
            [
                [-0.0118366, 0.0118366],
                [0.011499, -0.011499],
            ]
        )
