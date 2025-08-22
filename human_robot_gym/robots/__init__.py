"""Defines robot class mapping."""
import human_robot_gym.models.robots  # noqa: F401
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.models.robots.robot_model import REGISTERED_ROBOTS  # noqa: F401
from robosuite.robots.fixed_base_robot import FixedBaseRobot

# Robot class mappings -- must be maintained manually
ROBOT_CLASS_MAPPING["Schunk"] = FixedBaseRobot
ROBOT_CLASS_MAPPING["PandaZero"] = FixedBaseRobot
