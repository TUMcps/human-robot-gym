"""This package defines all environments and necessary gym wrappers."""
from robosuite.environments.base import REGISTERED_ENVS, MujocoEnv  # noqa: F401

ALL_ENVIRONMENTS = REGISTERED_ENVS.keys()
