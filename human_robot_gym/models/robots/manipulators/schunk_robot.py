"""This file describes the Schunk model.

The Schunk robot uses pinocchio to model kinematics and dynamics.

Owner:
    Jakob Thumm (JT)

Contributors:
    Felix Trost (FT)

Changelog:
    2.5.22 JT Formatted docstrings
    27.3.23 FT Changed default gripper to RethinkValidGripper
"""

import numpy as np

from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)
from human_robot_gym.utils.mjcf_utils import xml_path_completion
import human_robot_gym.models.grippers  # noqa: F401


class Schunk(PinocchioManipulatorModel):
    """
    Schunk is a sensitive single-arm robot designed by Schunk.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):  # noqa: D107
        super().__init__(
            fname=xml_path_completion("robots/schunk/robot.xml"),
            urdf_file=xml_path_completion("robots/schunk/robot.urdf"),
            package_dirs=xml_path_completion("robots/"),
            idn=idn,
        )

        # Set joint damping
        self.set_joint_attribute(
            attrib="damping",
            values=np.array((0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001)),
        )

    @property
    def default_mount(self):
        """Get default mount."""
        return "RethinkMount"

    @property
    def default_gripper(self):
        """Get default gripper."""
        return "RethinkValidGripper"

    @property
    def default_controller_config(self):
        """Get default controller config."""
        return "default_panda"

    @property
    def init_qpos(self):
        """Get the initial joint position."""
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    @property
    def base_xpos_offset(self):
        """Get the base offset."""
        # TODO: Tune these values
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        """Get the top offset."""
        # TODO: Tune these values
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        """Get the horizontal radius."""
        # TODO: Tune these values
        return 0.5

    @property
    def arm_type(self):
        """Get the arm type."""
        return "single"
