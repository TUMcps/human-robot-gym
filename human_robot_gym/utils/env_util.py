"""This file describes functions for creating the human-robot-gym environments.

Contributors:
    Felix Trost (FT)
    Martin Winter (MW)

Changelog:
    15.02.23 FT added support for expert observation environments
    16.07.23 MW moved all SB3 specific code to human_robot_gym/utils/env_util_SB3.py
"""
import struct
from typing import Optional, Dict, Any, List
import gym
from human_robot_gym.wrappers.time_limit import TimeLimit
import robosuite
from robosuite.wrappers.gym_wrapper import GymWrapper
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
