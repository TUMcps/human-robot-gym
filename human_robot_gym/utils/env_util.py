"""This file describes functions for creating the human-robot-gym environments.

Contributors:
    Felix Trost (FT)

Changelog:
    15.02.23 FT added support for expert observation environments
"""
import struct
from typing import Optional, Dict, Any, Type, Callable, Union, List
from functools import partial
import gym
from gym.wrappers import TimeLimit
import robosuite
from robosuite.wrappers.gym_wrapper import GymWrapper
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnv
from stable_baselines3.common.env_util import make_vec_env as sb3_make_vec_env
from human_robot_gym.wrappers.goal_env_wrapper import GoalEnvironmentGymWrapper
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper


def make_robosuite_env(
    env_id: str,
    env_kwargs: Optional[Dict[str, Any]] = None
) -> gym.Env:
    """Make the robosuite environment."""
    return robosuite.make(env_id, **env_kwargs)


def add_time_limit(
    env: gym.Env,
    max_episode_steps: int = 1000
) -> gym.Env:
    if env.spec is None:
        env.spec = struct
    if max_episode_steps is not None:
        env = TimeLimit(env, max_episode_steps=max_episode_steps)
    return env


def make_gym_env(
    env_id: str,
    env_kwargs: Optional[Dict[str, Any]] = None,
    obs_keys: Optional[List[str]] = None,
) -> gym.Env:
    """Make the gym environment and add the optional TimeLimit wrapper.

    We add the TimeLimit wrapper here because it would require a second Monitor wrapper later.
    """
    env = GymWrapper(make_robosuite_env(env_id, env_kwargs), keys=obs_keys)
    env = add_time_limit(env, max_episode_steps=env_kwargs.get("horizon", None))
    return env


def make_goal_env(
    env_id: str,
    env_kwargs: Optional[Dict[str, Any]] = None,
    obs_keys: Optional[List[str]] = None,
) -> gym.Env:
    """Make the goal environment and add the optional TimeLimit wrapper.

    We add the TimeLimit wrapper here because it would require a second Monitor wrapper later.
    """
    env = GoalEnvironmentGymWrapper(make_robosuite_env(env_id, env_kwargs), keys=obs_keys)
    env = add_time_limit(env, max_episode_steps=env_kwargs.get("horizon", None))
    return env


def make_expert_obs_env(
    env_id: str,
    env_kwargs: Optional[Dict[str, Any]] = None,
    obs_keys: Optional[List[str]] = None,
    expert_obs_keys: Optional[List[str]] = None,
) -> gym.Env:
    """Make the expert obs environment and add the optional TimeLimit wrapper.

    We add the TimeLimit wrapper here because it would require a second Monitor wrapper later.
    """
    env = ExpertObsWrapper(make_robosuite_env(env_id, env_kwargs), agent_keys=obs_keys, expert_keys=expert_obs_keys)
    env = add_time_limit(env, max_episode_steps=env_kwargs.get("horizon", None))
    return env


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
