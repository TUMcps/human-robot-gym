"""This file defines utility functions for setting up trainings from hydra config files.

Author:
    Felix Trost (FT)
    Martin Winter (MW)

Changelog:
    04.04.23 (FT) File created
    16.07.23 (MW) Moved all SB3 specific code to human_robot_gym/utils/training_utils_SB3.py
"""
from typing import Any, Callable, Dict, List
from copy import deepcopy

from omegaconf import OmegaConf
import wandb
from wandb.sdk.wandb_run import Run

import numpy as np

import gym

from robosuite.controllers import load_controller_config

from human_robot_gym.demonstrations.experts import Expert, REGISTERED_EXPERTS

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.env_util import make_gym_env, make_expert_obs_env
from human_robot_gym.utils.config_utils import TrainingConfig, DataCollectionConfig

from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper
from human_robot_gym.wrappers.action_based_expert_imitation_reward_wrapper import (
    CartActionBasedExpertImitationRewardWrapper,
    JointActionBasedExpertImitationRewardWrapper,
)
from human_robot_gym.wrappers.state_based_expert_imitation_reward_wrapper import (
    ReachHumanStateBasedExpertImitationRewardWrapper,
    PickPlaceHumanCartStateBasedExpertImitationRewardWrapper,
    CollaborativeLiftingCartStateBasedExpertImitationRewardWrapper,
)
from human_robot_gym.wrappers.dataset_collection_wrapper import DatasetCollectionWrapper
from human_robot_gym.wrappers.dataset_wrapper import DatasetObsNormWrapper, DatasetRSIWrapper

import human_robot_gym.robots  # noqa: F401


def get_controller_configs(config: TrainingConfig) -> List[Dict[str, Any]]:
    """Obtain the controller config for the robot from the paths specified in the config.

    We merge a robot specific config into the config for the controller (e.g. a failsafe controller).

    Args:
        config (TrainingConfig): The training config containing the paths to the controller and robot configs.

    Returns:
        List[Dict[str, Any]]: The controller config for the robot. Currently, only a single robot is supported;
            thus, the list contains only a single element.
    """
    controller_config_path = file_path_completion(config.robot.controller_config_path)
    robot_config_path = file_path_completion(config.robot.robot_config_path)

    controller_config = merge_configs(
        load_controller_config(custom_fpath=controller_config_path),
        load_controller_config(custom_fpath=robot_config_path),
    )

    return [controller_config]


def _compose_environment_kwargs(config: TrainingConfig, evaluation_mode: bool) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the environment.

    Args:
        config (TrainingConfig): The config object containing information about the environment
        evaluation_mode (bool): If `True`, use `run.eval_seed` instead of `environment.seed`. Defaults to `False`.

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments for the environment.
    """
    kwargs = OmegaConf.to_container(cfg=deepcopy(config.environment), resolve=True, throw_on_missing=True)
    del kwargs["env_id"]
    kwargs["robots"] = config.robot.name
    kwargs["controller_configs"] = get_controller_configs(config)
    if evaluation_mode:
        kwargs["seed"] = config.run.eval_seed

    return kwargs


def create_wrapped_env_from_config(config: TrainingConfig, evaluation_mode: bool = False) -> gym.Env:
    """Create a non-vectorized wrapped gym environment from a config.

    Args:
        config (Config): The config object containing information about the environment and optional wrappers
    """
    kwargs = _compose_environment_kwargs(config=config, evaluation_mode=evaluation_mode)

    if config.run.expert_obs_keys is None:
        env = make_gym_env(
            env_id=config.environment.env_id,
            env_kwargs=kwargs,
            obs_keys=config.run.obs_keys,
        )
    else:
        env = make_expert_obs_env(
            env_id=config.environment.env_id,
            env_kwargs=kwargs,
            obs_keys=config.run.obs_keys,
            expert_obs_keys=config.run.expert_obs_keys,
        )

    env = get_environment_wrap_fn(config=config)(env=env)

    return env


def create_data_collection_environment(config: DataCollectionConfig, start_episode: int = 0) -> gym.Env:
    """Create a wrapped gym environment for data collection from a config.

    Args:
        config (Config): The config object containing information about the environment and optional wrappers
        start_episode (int): The index of the first episode. Defaults to 0.
    """
    env = create_wrapped_env_from_config(config=config)

    env = DatasetCollectionWrapper(
        env=env,
        directory=file_path_completion(f"../datasets/{config.dataset_name}"),
        start_episode=start_episode,
        store_expert_observations=config.run.expert_obs_keys is not None,
        verbose=config.run.verbose,
    )

    return env


def _compose_expert_kwargs(config: TrainingConfig) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the expert.

    Args:
        config (Config): The config object containing information about the expert

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments for the expert.
    """
    kwargs = OmegaConf.to_container(cfg=deepcopy(config.expert), resolve=True, throw_on_missing=True)
    del kwargs["id"]
    del kwargs["obs_keys"]
    return kwargs


def create_expert(config: TrainingConfig, env: gym.Env) -> Expert:
    """Create an expert from a config.

    Args:
        config (Config): The config object containing information about the expert
        env (gym.Env): The environment the expert is defined for

    Returns:
        Expert: The expert specified in the config

    Raises:
        AssertionError: [No expert specified in config!]
        AssertionError: [Expert {`config.expert.id`} not registered!]
    """
    kwargs = _compose_expert_kwargs(config=config)
    assert config.expert is not None, "No expert specified in config!"
    assert config.expert.id in REGISTERED_EXPERTS, f"Expert {config.expert.id} not registered!"
    return REGISTERED_EXPERTS[config.expert.id](
        observation_space=env.observation_space,
        action_space=env.action_space,
        **kwargs
    )


def _compose_ik_position_delta_wrapper_kwargs(config: TrainingConfig) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the `IKPositionDeltaWrapper`.

    Args:
        config (Config): The config object containing information about the wrapper

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments for the `IKPositionDeltaWrapper`.
    """
    kwargs = OmegaConf.to_container(
        cfg=deepcopy(config.wrappers.ik_position_delta),
        resolve=True,
        throw_on_missing=True
    )
    kwargs["urdf_file"] = file_path_completion(kwargs["urdf_file"])
    action_limit = kwargs["action_limit"]
    del kwargs["action_limit"]
    kwargs["action_limits"] = np.array([
        [-action_limit, -action_limit, -action_limit],
        [action_limit, action_limit, action_limit]
    ])
    if kwargs["x_position_limits"] is not None:
        kwargs["x_position_limits"] = np.array(kwargs["x_position_limits"])

    return kwargs


def env_has_cartesian_action_space(config: TrainingConfig) -> bool:
    """Checks whether the wrapped environment has a Cartesian action space."""
    return hasattr(config.wrappers, "ik_position_delta") and config.wrappers.ik_position_delta is not None


def state_based_expert_imitation_reward_wrap_fn(
    config: TrainingConfig,
    env: gym.Env,
) -> gym.Env:
    """Wrap the environment in an `StateBasedExpertImitationRewardWrapper`.

    Which subclass is used depends on the expert specified in the config.

    Args:
        config (TrainingConfig): The config object containing information about the wrapper
        env (gym.Env): The environment to wrap

    Returns:
        gym.Env: The wrapped environment

    Raises:
        [AssertionError: No expert specified in config!]
        [NotImplementedError:
            State based expert imitation reward wrapper not implemented for expert {config.expert.id}!]
    """
    assert hasattr(config, "expert") and config.expert is not None, "No expert specified in config!"

    if config.expert.id in ["ReachHuman", "ReachHumanCart"]:
        return ReachHumanStateBasedExpertImitationRewardWrapper(
            env=env,
            **config.wrappers.state_based_expert_imitation_reward,
        )
    elif config.expert.id == "PickPlaceHumanCart":
        return PickPlaceHumanCartStateBasedExpertImitationRewardWrapper(
            env=env,
            **config.wrappers.state_based_expert_imitation_reward,
        )
    elif config.expert.id == "CollaborativeLiftingCart":
        return CollaborativeLiftingCartStateBasedExpertImitationRewardWrapper(
            env=env,
            **config.wrappers.state_based_expert_imitation_reward,
        )
    else:
        raise NotImplementedError(
            f"State based expert imitation reward wrapper not implemented for expert {config.expert.id}!"
        )


def _compose_action_based_expert_imitation_reward_wrapper_kwargs(config: TrainingConfig) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the `ActionBasedExpertImitationRewardWrapper`.

    Args:
        config (Config): The config object containing information about the wrapper

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments
            for the `ActionBasedExpertImitationRewardWrapper`.
    """
    kwargs = OmegaConf.to_container(
        cfg=deepcopy(config.wrappers.action_based_expert_imitation_reward),
        resolve=True,
        throw_on_missing=True,
    )

    del kwargs["rsi_prob"]
    del kwargs["dataset_name"]

    return kwargs


def action_based_expert_imitation_reward_wrap_fn(
    config: TrainingConfig,
    env: gym.Env,
) -> gym.Env:
    """Wrap the environment in an `ActionBasedExpertImitationRewardWrapper`.

    If the config specifies a `rsi_prob` > 0, the environment is also wrapped in a `DatasetRSIWrapper`.
    This step is omitted if the config specifies a state-based expert imitation reward wrapper,
    as the state-based wrapper already inherits from the dataset wrapper.

    Args:
        config (TrainingConfig): The config object containing information about the wrapper
        env (gym.Env): The environment to wrap

    Returns:
        gym.Env: The wrapped environment

    Raises:
        [AssertionError: No expert specified in config!]
        [NotImplementedError: Action based expert imitation reward wrapper not implemented for joint action space!]
    """
    assert hasattr(config, "expert") and config.expert is not None, "No expert specified in config!"

    if (
        config.wrappers.action_based_expert_imitation_reward.rsi_prob is not None and not (
            hasattr(config.wrappers, "state_based_expert_imitation_reward") and
            config.wrappers.state_based_expert_imitation_reward is not None
        )
    ):
        env = DatasetRSIWrapper(
            env=env,
            dataset_name=config.wrappers.action_based_expert_imitation_reward.dataset_name,
            rsi_prob=config.wrappers.action_based_expert_imitation_reward.rsi_prob,
        )

    expert = create_expert(config=config, env=env)
    kwargs = _compose_action_based_expert_imitation_reward_wrapper_kwargs(config=config)

    if env_has_cartesian_action_space(config=config):
        env = CartActionBasedExpertImitationRewardWrapper(
            env=env,
            expert=expert,
            **kwargs,
        )
    else:
        env = JointActionBasedExpertImitationRewardWrapper(
            env=env,
            expert=expert,
            **kwargs,
        )

    return env


def _compose_dataset_obs_norm_wrapper_kwargs(config: TrainingConfig) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the `DatasetObsNormWrapper`.

    Args:
        config (Config): The config object containing information about the wrapper

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments
            for the `DatasetObsNormWrapper`.
    """
    kwargs = OmegaConf.to_container(
        cfg=deepcopy(config.wrappers.dataset_obs_norm),
        resolve=True,
        throw_on_missing=True,
    )

    kwargs["mean"] = np.array(kwargs["mean"]) if kwargs["mean"] is not None else None
    kwargs["std"] = np.array(kwargs["std"]) if kwargs["std"] is not None else None

    return kwargs


def get_environment_wrap_fn(config: TrainingConfig) -> Callable[[gym.Env], gym.Env]:
    """Create a function that wraps the environment as specified in the config.

    Args:
        config (TrainingConfig): The config object containing information about the wrappers

    Returns:
        Callable[[gym.Env], gym.Env]: A function that wraps the environment as specified in the config.
    """
    def wrap_fn(env: gym.Env):
        # Collision prevention wrapper
        if hasattr(config.wrappers, "collision_prevention") and config.wrappers.collision_prevention is not None:
            env = CollisionPreventionWrapper(
                env=env,
                collision_check_fn=env.check_collision_action,
                **config.wrappers.collision_prevention,
            )

        # Inverse kinematics wrapper
        if env_has_cartesian_action_space(config=config):
            ikPositionDeltaKwargs = _compose_ik_position_delta_wrapper_kwargs(config=config)
            env = IKPositionDeltaWrapper(
                env=env,
                **ikPositionDeltaKwargs,
            )

        if (
            hasattr(config.wrappers, "state_based_expert_imitation_reward") and
            config.wrappers.state_based_expert_imitation_reward is not None
        ):
            env = state_based_expert_imitation_reward_wrap_fn(config=config, env=env)

        # Action based expert imitation reward wrapper
        if (
            hasattr(config.wrappers, "action_based_expert_imitation_reward") and
            config.wrappers.action_based_expert_imitation_reward is not None
        ):
            env = action_based_expert_imitation_reward_wrap_fn(config=config, env=env)

        # Dataset observation normalization wrapper
        if hasattr(config.wrappers, "dataset_obs_norm") and config.wrappers.dataset_obs_norm is not None:
            env = DatasetObsNormWrapper(
                env=env,
                **_compose_dataset_obs_norm_wrapper_kwargs(config),
            )

        # Visualization wrapper
        if hasattr(config.wrappers, "visualization") and config.wrappers.visualization is not None:
            env = VisualizationWrapper(env=env)

        return env

    return wrap_fn


def init_wandb(config: TrainingConfig) -> Run:
    """Initialize WandB.

    If a `id` is already specified in the run sub-config, this run will be resumed.
    Otherwise, a new run is created.

    Args:
        config (TrainingConfig): The config containing information about the run

    Returns:
        Run: The WandB run object

    Raises:
        ValueError: [Cannot load a model without an id]
    """
    if config.run.id is None and config.run.load_step is not None:
        raise ValueError("Cannot load a model without an id")

    run_id = config.run.id

    if config.run.id is not None and config.run.load_step is None:
        print("load_step not specified, will ignore id and create a new run")
        run_id = None

    resume = None if config.run.id is None else "must"

    config_dict = OmegaConf.to_container(
        cfg=config,
        resolve=True,
        throw_on_missing=False,
    )

    return wandb.init(
        project=config.wandb_run.project,
        entity=config.wandb_run.entity,
        group=config.wandb_run.group,
        name=config.wandb_run.name,
        tags=config.wandb_run.tags,
        config=config_dict,
        save_code=False,
        monitor_gym=True,
        sync_tensorboard=True,
        id=run_id,
        resume=resume,
    )
