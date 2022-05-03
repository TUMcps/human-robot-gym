"""This file defines how gym environments are made.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""

from typing import Union, List

from robosuite.wrappers.gym_wrapper import GymWrapper
from robosuite.environments.base import make
from gym import Env
from gym.envs.registration import spec


def make_gym(env: str, robots: Union[str, List[str]], id: str, **kwargs) -> Env:
    """Wrap an environment with a gym wrapper.

    Args:
        robots: list of robots in the environment.
        id: Gym environment id.

    Returns:
        Wrapped gym environment.
    """
    gym_env = GymWrapper(env=make(env, robots=robots, **kwargs))
    gym_env.spec = spec(id)
    return gym_env
