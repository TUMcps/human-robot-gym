"""This file describes the Kinova3 model.

The Kinova3 robot using pinocchio to model kinematics and dynamics.

"""

import numpy as np

import human_robot_gym as hrgym
from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)
from human_robot_gym.utils.mjcf_utils import xml_path_completion
import human_robot_gym.models.grippers  # noqa: F401
import robosuite
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.robots import ROBOT_CLASS_MAPPING, Bimanual, BIMANUAL_ROBOTS, SingleArm

def load_kinova3pinn_models():
    ROBOT_CLASS_MAPPING["Kinova3Pinn"] = SingleArm

class Kinova3Pinn(PinocchioManipulatorModel):
    """

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):  # noqa: D107
        super().__init__(
            fname=xml_path_completion("robots/kinova3/robot.xml"),
            urdf_file=xml_path_completion("robots/kinova3/kinova3.urdf"),  # robot.urdf
            package_dirs=xml_path_completion("robots/"),
            idn=idn,
        )

        # Set joint damping
        self.set_joint_attribute(
            attrib="damping",
            values=np.array((0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001)),
        )

    @property
    def default_mount(self):
        """Get default mount."""
        return "RethinkMount"

    @property
    def default_gripper(self):
        """Get default gripper."""
        return "RethinkValidGripper" # RethinkValidGripper  Robotiq85Gripper

    @property
    def default_controller_config(self):
        """Get default controller config."""
        return "default_kinova3"

    @property
    def init_qpos(self):
        """Get the initial joint position."""
        # return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        return np.array([-0.377, 0.53, 0.0314, 1.6, 0, 0.356, -1.6])

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