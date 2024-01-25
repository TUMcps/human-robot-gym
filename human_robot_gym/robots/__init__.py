"""Defines robot class mapping."""
import human_robot_gym.models.robots  # noqa: F401
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.models.robots.robot_model import REGISTERED_ROBOTS  # noqa: F401
from robosuite.robots.single_arm import SingleArm

# Robot class mappings -- must be maintained manually
ROBOT_CLASS_MAPPING["Schunk"] = SingleArm
ROBOT_CLASS_MAPPING["PandaZero"] = SingleArm
