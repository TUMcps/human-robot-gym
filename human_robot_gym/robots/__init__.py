import human_robot_gym.models.robots
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.models.robots.robot_model import REGISTERED_ROBOTS
from robosuite.robots.single_arm import SingleArm

# Robot class mappings -- must be maintained manually
ROBOT_CLASS_MAPPING["Schunk"] = SingleArm