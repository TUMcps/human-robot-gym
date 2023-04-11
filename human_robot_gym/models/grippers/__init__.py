"""This package defines all gripper models."""
# flake8: noqa
from human_robot_gym.models.grippers.rethink_valid_gripper import RethinkValidGripper
from robosuite.models.grippers import ALL_GRIPPERS, GRIPPER_MAPPING

GRIPPER_MAPPING["RethinkValidGripper"] = RethinkValidGripper

ALL_GRIPPERS = GRIPPER_MAPPING.keys()
