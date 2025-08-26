"""This file describes a simplified dynamic obstacle model.

This obstacle only hase one joint element.
The obstacle object is fully defined in assets/human/single_point_human.xml.

Owner:
    Jakob Thumm (JT)

Changelog:
    25.8.25 JT Created single point human obstacle model
"""

from robosuite.models.objects.xml_objects import MujocoXMLObject

from human_robot_gym.utils.mjcf_utils import xml_path_completion


class SinglePointHumanObject(MujocoXMLObject):
    """Dynamic obstacle that is loaded from an XML file.

    The human can be controlled by setting the x, y, and z components of each human joint.

    Args:
        name (str): Name of the human object.
    """

    def __init__(self, name):  # noqa: D107
        super().__init__(
            xml_path_completion("human/single_point_human.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )
        self._setup_joint_names()

    def _setup_joint_names(self):
        """Define the name of all controllable and observable joints."""
        self.joint_elements = []
        self.joint_names = []
        for joint in self.joint_elements:
            self.joint_names.append(self.naming_prefix + joint + "_x")
            self.joint_names.append(self.naming_prefix + joint + "_y")
            self.joint_names.append(self.naming_prefix + joint + "_z")
        # Observables:
        self.obs_joint_elements = []
