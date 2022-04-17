from robosuite.models.objects.xml_objects import MujocoXMLObject

from human_robot_gym.utils.mjcf_utils import xml_path_completion

class HumanObject(MujocoXMLObject):
    """
    Human object that is loaded from an XML file.
    The human can be controlled by setting the x, y, and z components of each human joint.
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("human/human.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )
        self._setup_joint_names()

    def _setup_joint_names(self):
        """
        Define the name of all controllable joints
        """
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
            "R_Hand"
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
            "R_Hand"
        ]