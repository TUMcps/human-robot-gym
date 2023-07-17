"""This file describes functions for creating the human-robot-gym environments.

Contributors:
    Martin Winter (MW)

Changelog:
    16.07.23 MW moved all SB3 specific code form human_robot_gym/utils/env_util.py to
    human_robot_gym/utils/env_util_SB3.py
    """
from typing import Optional, Dict, Any, Type, Callable, Union, List
from functools import partial
import gym

from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnv
from stable_baselines3.common.env_util import make_vec_env as sb3_make_vec_env
from human_robot_gym.utils.env_util import make_gym_env, make_goal_env, make_expert_obs_env


def make_vec_env(
    env_id: str,
    type: str = "env",
    obs_keys: Optional[List[str]] = None,
    expert_obs_keys: Optional[List[str]] = None,
    n_envs: int = 1,
    seed: Optional[int] = None,
    start_index: int = 0,
    monitor_dir: Optional[str] = None,
    wrapper_class: Optional[Callable[[gym.Env], gym.Env]] = None,
    env_kwargs: Optional[Dict[str, Any]] = None,
    vec_env_cls: Optional[Type[Union[DummyVecEnv, SubprocVecEnv]]] = None,
    vec_env_kwargs: Optional[Dict[str, Any]] = None,
    monitor_kwargs: Optional[Dict[str, Any]] = None,
    wrapper_kwargs: Optional[Dict[str, Any]] = None,
) -> VecEnv:
    """
    Create a wrapped, monitored ``VecEnv``.
    By default it uses a ``DummyVecEnv`` which is usually faster
    than a ``SubprocVecEnv``.

    Args:
        env_id: the environment ID or the environment class
        type: The type of environment to create. Can be "env" or "goal_env".
        obs_keys: The observation keys to use for the environment.
        expert_obs_keys: If set, add observations for an expert policy to the info dictionary
        n_envs: the number of environments you wish to have in parallel
        seed: the initial seed for the random number generator
        start_index: start rank index
        monitor_dir: Path to a folder where the monitor files will be saved.
            If None, no file will be written, however, the env will still be wrapped
            in a Monitor wrapper to provide additional information about training.
        wrapper_class: Additional wrapper to use on the environment.
            This can also be a function with single argument that wraps the environment in many things.
        env_kwargs: Optional keyword argument to pass to the env constructor
        vec_env_cls: A custom ``VecEnv`` class constructor. Default: None.
        vec_env_kwargs: Keyword arguments to pass to the ``VecEnv`` class constructor.
        monitor_kwargs: Keyword arguments to pass to the ``Monitor`` class constructor.
        wrapper_kwargs: Keyword arguments to pass to the ``Wrapper`` class constructor.

    Returns:
        The wrapped environment
    """
    assert type in ["env", "goal_env"], "The type of environment must be either 'env' or 'goal_env'."
    if type == "env":
        if expert_obs_keys is None:
            env_callable = partial(make_gym_env, env_id, env_kwargs, obs_keys=obs_keys)
        else:
            env_callable = partial(
                make_expert_obs_env,
                env_id, env_kwargs,
                obs_keys=obs_keys,
                expert_obs_keys=expert_obs_keys
            )
    else:
        env_callable = partial(make_goal_env, env_id, env_kwargs, obs_keys=obs_keys)
    return sb3_make_vec_env(
        env_id=env_callable,
        n_envs=n_envs,
        seed=seed,
        start_index=start_index,
        monitor_dir=monitor_dir,
        wrapper_class=wrapper_class,
        env_kwargs=None,
        vec_env_cls=vec_env_cls,
        vec_env_kwargs=vec_env_kwargs,
        monitor_kwargs=monitor_kwargs,
        wrapper_kwargs=wrapper_kwargs,
    )
