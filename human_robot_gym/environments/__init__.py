"""This package defines all environments and necessary gym wrappers."""
from robosuite.environments.base import REGISTERED_ENVS, MujocoEnv  # noqa: F401

from human_robot_gym.environments.manipulation.human_env import HumanEnv  # noqa: F401
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman  # noqa: F401
from human_robot_gym.environments.manipulation.reach_human_cartesian_env import ReachHumanCart  # noqa: F401
from human_robot_gym.environments.manipulation.pick_place_human_cartesian_env import PickPlaceHumanCart  # noqa: F401

ALL_ENVIRONMENTS = REGISTERED_ENVS.keys()
