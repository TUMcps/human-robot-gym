from typing import Union, List
import gym

from robosuite.wrappers.gym_wrapper import GymWrapper
from robosuite.environments.base import make
from gym import Env
from gym.envs.registration import spec

def make_gym(
    env: str, 
    robots: Union[str, List[str]],
    id: str,
    **kwargs) -> Env: 

    gym_env = GymWrapper(env=make(env, robots=robots, **kwargs))
    gym_env.spec = spec(id)
    return gym_env