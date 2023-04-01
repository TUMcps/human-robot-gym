from typing import Any, Callable, Dict, List
import argparse
from omegaconf import DictConfig

import gym

from robosuite.controllers import load_controller_config

from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.env_util import make_vec_env


def parse_args() -> Dict[str, Any]:
    """Initialize the argument parser."""
    parser = argparse.ArgumentParser(
        usage="%(prog)s [OPTION] [FILE]...",
        description="Train a robot on the human reach task using SAC and HER.",
    )

    parser.add_argument(
        "config",
        help="Config file name for the training. Example is 'schunk_sac_her_safe.json'",
        type=str,
    )

    parser.add_argument(
        "--wandb", help="Log this run with weights and biases.", action="store_true"
    )

    return parser.parse_args()


def get_controller_configs(config: DictConfig) -> List[Dict[str, Any]]:
    controller_config_path = file_path_completion(config.robot.controller_config)
    robot_config_path = file_path_completion(config.robot.robot_config)

    controller_config = merge_configs(
        load_controller_config(custom_fpath=controller_config_path),
        load_controller_config(custom_fpath=robot_config_path),
    )

    return [controller_config]


def create_environment(config: DictConfig) -> VecEnv:
    wrapper_class = get_environment_wrap_fn(config)
    env = make_vec_env(
        env_id=config.environment.env_id,
        type=config.training.env_type,
        obs_keys=config.training.obs_keys,
        expert_obs_keys=config.training.expert_obs_keys,
        n_envs=config.training.n_envs,
        seed=config.training.seed,
        start_index=config.training.start_index,
        monitor_dir=config.training.monitor_dir,
        wrapper_class=wrapper_class,
        env_kwargs=config.environment.kwargs,
        vec_env_cls=DummyVecEnv if config.training.n_envs == 1 else SubprocVecEnv,
        vec_env_kwargs=config.training.vec_env_kwargs,
        monitor_kwargs=config.training.monitor_kwargs,
        wrapper_kwargs=None,
    )

    return env


def get_environment_wrap_fn(config: DictConfig) -> Callable[[gym.Env], gym.Env]:
    def wrap_fn(env):
        return env

    return wrap_fn
