"""This file describes the human model.

Humans have multiple joint elements, some of which are observable.
The human object is fully defined in assets/human/human.xml.

Owner:
    Jakob Thumm (JT)

Contributors:
    Julian Balletshofer (JB)
Changelog:
    2.5.22 JT Formatted docstrings
    15.7.22 JB added properties for right and left hand and head
"""

from robosuite.models.objects.xml_objects import MujocoXMLObject

from human_robot_gym.utils.mjcf_utils import xml_path_completion


class HumanObject(MujocoXMLObject):
    """Human object that is loaded from an XML file.

    The human can be controlled by setting the x, y, and z components of each human joint.

    Args:
        name (str): Name of the human object.
    """

    def __init__(self, name):  # noqa: D107
        super().__init__(
            xml_path_completion("human/human.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )
        self._setup_joint_names()
    
    @property
    def left_hand(self):
        """Get the joint name of the left hand."""
        return self.naming_prefix + "L_Hand"
    @property
    def right_hand(self):
        """Get the joint name of the right hand."""
        return self.naming_prefix + "R_Hand"
    @property
    def head(self):
        """Get the joint name of the head."""
        return self.naming_prefix + "Head"


    def _setup_joint_names(self):
        """Define the name of all controllable and observable joints."""
        self.joint_elements = [
            "L_Hip",
            "R_Hip",
            "Torso",
            "L_Knee",
            "R_Knee",
            "Spine",
            "L_Ankle",
            "R_Ankle",
            "Chest",
            "L_Toe",
            "R_Toe",
            "Neck",
            "L_Thorax",
            "R_Thorax",
            "Head",
            "L_Shoulder",
            "R_Shoulder",
            "L_Elbow",
            "R_Elbow",
            "L_Wrist",
            "R_Wrist",
            "L_Hand",
            "R_Hand",
        ]
        self.joint_names = []
        for joint in self.joint_elements:
            self.joint_names.append(self.naming_prefix + joint + "_x")
            self.joint_names.append(self.naming_prefix + joint + "_y")
            self.joint_names.append(self.naming_prefix + joint + "_z")
        # Observables:
        self.obs_joint_elements = [
            "Torso",
            "Chest",
            "Head",
            "L_Shoulder",
            "R_Shoulder",
            "L_Elbow",
            "R_Elbow",
            "L_Hand",
            "R_Hand",
        ]
