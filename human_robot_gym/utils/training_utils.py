"""This file defines utility functions for setting up trainings from hydra config files.

Author:
    Felix Trost (FT)

Changelog:
    04.04.23 (FT) File created
"""
from typing import Any, Callable, Dict, List, Optional, Union
from copy import deepcopy
import os

from omegaconf import OmegaConf
import wandb
from wandb.sdk.wandb_run import Run

import numpy as np

import gym

from robosuite.controllers import load_controller_config

from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.off_policy_algorithm import OffPolicyAlgorithm
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import SAC, PPO, HerReplayBuffer

from human_robot_gym.demonstrations.experts import Expert, REGISTERED_EXPERTS

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.env_util import make_gym_env, make_vec_env, make_expert_obs_env
from human_robot_gym.utils.config_utils import TrainingConfig, DataCollectionConfig

from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper
from human_robot_gym.wrappers.action_based_expert_imitation_reward_wrapper import (
    CartActionBasedExpertImitationRewardWrapper
)
from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import custom_add, _custom_sample_transitions
from human_robot_gym.wrappers.tensorboard_callback import TensorboardCallback
from human_robot_gym.wrappers.dataset_collection_wrapper import DatasetCollectionWrapper
from human_robot_gym.wrappers.dataset_wrapper import DatasetObsNormWrapper, DatasetRSIWrapper

SB3_ALGORITHMS = {
    "SAC": SAC,
    "PPO": PPO,
}


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


def create_training_vec_env(config: TrainingConfig, evaluation_mode: bool = False) -> VecEnv:
    """Create an environment from a config and optionally wrap it in specified wrappers.

    If the config specifies more than one environment (`run.n_envs > 1`),
    they are run on different threads using a `SubprocVecEnv`. Otherwise, a `DummyVecEnv` is created.

    Args:
        config (Config): The config object containing information about the environment and optional wrappers
        evaluation_mode (bool): If `True`, use `run.eval_seed` instead of `environment.seed`. Defaults to `False`.
    """
    wrapper_class = get_environment_wrap_fn(config)

    # Get the environment keword arguments as a dict
    env_kwargs = _compose_environment_kwargs(config=config, evaluation_mode=evaluation_mode)

    env = make_vec_env(
        env_id=config.environment.env_id,
        type=config.run.env_type,
        obs_keys=config.run.obs_keys,
        expert_obs_keys=config.run.expert_obs_keys,
        n_envs=config.run.n_envs,
        seed=config.run.seed,
        start_index=config.run.start_index,
        monitor_dir=config.run.monitor_dir,
        wrapper_class=wrapper_class,
        env_kwargs=env_kwargs,
        vec_env_cls=DummyVecEnv if config.run.n_envs == 1 else SubprocVecEnv,
        vec_env_kwargs=config.run.vec_env_kwargs,
        monitor_kwargs=config.run.monitor_kwargs,
        wrapper_kwargs=None,
    )

    return env


def create_wrapped_env_from_config(config: TrainingConfig) -> gym.Env:
    """Create a non-vectorized wrapped gym environment from a config.

    Args:
        config (Config): The config object containing information about the environment and optional wrappers
    """
    kwargs = _compose_environment_kwargs(config=config, evaluation_mode=False)

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


def _compose_action_based_expert_imitation_reward_wrapper_kwargs(config: TrainingConfig) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the `CartActionBasedExpertImitationRewardWrapper`.

    Args:
        config (Config): The config object containing information about the wrapper

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments
            for the `CartActionBasedExpertImitationRewardWrapper`.
    """
    kwargs = OmegaConf.to_container(
        cfg=deepcopy(config.wrappers.action_based_expert_imitation_reward),
        resolve=True,
        throw_on_missing=True,
    )

    del kwargs["rsi_prob"]
    del kwargs["dataset_name"]

    return kwargs


def env_has_cartesian_action_space(config: TrainingConfig) -> bool:
    """Checks whether the wrapped environment has a Cartesian action space."""
    return hasattr(config.wrappers, "ik_position_delta") and config.wrappers.ik_position_delta is not None


def action_based_expert_imitation_reward_wrap_fn(
    config: TrainingConfig,
    env: gym.Env,
) -> gym.Env:
    """Wrap the environment in an `CartActionBasedExpertImitationRewardWrapper`.

    If the config specifies a `rsi_prob` > 0, the environment is also wrapped in a `DatasetRSIWrapper`.

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

    if config.wrappers.action_based_expert_imitation_reward.rsi_prob is not None:
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
        raise NotImplementedError(
            "Action based expert imitation reward wrapper not implemented for joint action space!"
        )

    return env


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
                **config.wrappers.dataset_obs_norm,
            )

        # Visualization wrapper
        if hasattr(config.wrappers, "visualization") and config.wrappers.visualization is not None:
            env = VisualizationWrapper(env=env)

        return env

    return wrap_fn


def _compose_algorithm_kwargs(
    config: TrainingConfig,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
    save_logs: bool = False,
) -> Dict[str, Any]:
    """Compose a dictionary of all configured keyword arguments for the algorithm.

    Args:
        config (TrainingConfig): The config object containing information about the algorithm
        env (VecEnv | None): The environment to train the algorithm on. Can be set to `None` for later initialization
        run_id (str | None): Can be set to override the run id specified in the run sub-config,
            as this value is usually set to `None` in training runs and generated at runtime.
        save_logs (bool): If `True`, save logs to the tensorboard log directory. Defaults to `False`.

    Returns:
        Dict[str, Any]: A dictionary of all configured keyword arguments for the algorithm.
    """
    if run_id is None:
        run_id = config.run.id

    kwargs = OmegaConf.to_container(cfg=deepcopy(config.algorithm), resolve=True, throw_on_missing=True)
    del kwargs["name"]

    kwargs["env"] = env
    kwargs["tensorboard_log"] = f"runs/{run_id}" if save_logs else None

    # Stable-baselines3 throws an error if train_freq is a list (expects tuple or int)
    if "train_freq" in kwargs and isinstance(kwargs["train_freq"], list):
        kwargs["train_freq"] = tuple(kwargs["train_freq"])

    # Support Hindsight Experience Replay (HER)
    if config.run.env_type == "goal_env":
        assert issubclass(SB3_ALGORITHMS[config.algorithm.name], OffPolicyAlgorithm), \
            "Only off-policy algorithms accepted for goal environments (HER)"

        assert config.run.n_envs == 1, "HER does not support vectorized environments!"

        HerReplayBuffer.add = custom_add
        HerReplayBuffer._sample_transitions = _custom_sample_transitions

        kwargs["replay_buffer_class"] = HerReplayBuffer

    return kwargs


def create_model(
    config: TrainingConfig,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
    save_logs: bool = False,
) -> BaseAlgorithm:
    """Create a new model according to the config for the given environment.

    If the env_type is set to 'goal_env' in the run sub-config,
    we use a `HerReplayBuffer` to support Hindsight Experience Replay.

    Args:
        config (TrainingConfig): The config object containing information about the model
        env (VecEnv | None): The environment to train the model on. Can be set to `None` for later initialization.
        run_id (str | None): Can be set to override the run id specified in the run sub-config,
            as this value is usually set to `None` in training runs and generated at runtime.
        save_logs (bool): Whether to save logs to tensorboard (or WandB)

    Returns:
        BaseAlgorithm: The model created according to the config for the given environment.

    Raises:
        AssertionError: [Only off-policy algorithms accepted for goal environments (HER)]
        AssertionError: [HER does not support vectorized environments!]
    """
    if config.run.verbose:
        print(f"Creating new model for run {run_id}")

    algorithm_kwargs = _compose_algorithm_kwargs(
        config=config,
        env=env,
        run_id=run_id,
        save_logs=save_logs,
    )

    return SB3_ALGORITHMS[config.algorithm.name](**algorithm_kwargs)


def load_model(
    config: TrainingConfig,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
    load_step: Optional[Union[int, str]] = None,
) -> BaseAlgorithm:
    """Load a model from disk.

    If the model uses an off-policy algorithm, the replay buffer is also loaded.

    Args:
        config (TrainingConfig): The config object containing information about the model
        env (VecEnv | None): The environment to use. Can be set to `None` for later initialization.
        run_id (str | None): The run id to load the model from.
            Can be set to override the value specified in the run sub-config
        load_step (int | str | None): The step to load the model from.
            Can be set to override the value specified in the run sub-config
            There has to exist a model file at `models/{run_id}/model_{load_step}.zip`.
            Generally, load_step should be set to a positive integer or `"final"`

    Returns:
        BaseAlgorithm: The loaded model

    Raises:
        ValueError: [load_step must be a positive integer or 'final']
    """
    if run_id is None:
        run_id = config.run.id

    if load_step is None:
        load_step = config.run.load_step

    if isinstance(load_step, int) and load_step < 0:
        raise ValueError("load_step must be a positive integer or 'final'")

    if config.run.verbose:
        print(f"Loading model from run {run_id} at episode {load_step}")

    if isinstance(load_step, int):
        model_path = f"models/{run_id}/model_{load_step:_}.zip"
    else:
        model_path = f"models/{run_id}/model_{load_step}.zip"

    model = SB3_ALGORITHMS[config.algorithm.name].load(
        model_path,
        env=env,
    )

    if env is not None:
        model.set_env(env)

    if isinstance(model, OffPolicyAlgorithm):
        model.load_replay_buffer(f"models/{run_id}/replay_buffer.pkl")

    return model


def get_model(
    config: TrainingConfig,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
    save_logs: bool = False,
) -> BaseAlgorithm:
    """Create a new model or load an existing model from disk,
        depending on whether a load episode is specified in the run sub-config.

    Args:
        config (TrainingConfig): The config object containing information about the model
        env (VecEnv | None): The environment to use. Can be set to `None` for later initialization.
        run_id (str | None): Can be set to override the run id specified in the run sub-config,
            as this value is usually set to `None` in training runs.
        save_logs (bool): Whether to save logs to tensorboard (or WandB)

    Returns:
        BaseAlgorithm: The model created according to the config for the given environment.
    """
    if config.run.load_step is None:
        return create_model(config=config, env=env, run_id=run_id, save_logs=save_logs)
    else:
        return load_model(config=config, env=env, run_id=run_id, load_step=config.run.load_step)


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


def run_debug_training(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training without storing any data to disk.

    This is useful for debugging purposes to avoid creating a lot of unnecessary files.

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    env = create_training_vec_env(config=config, evaluation_mode=False)
    model = create_model(config=config, env=env, run_id="~~debug~~", save_logs=False)
    model.learn(
        total_timesteps=config.run.n_steps,
        log_interval=1,
        reset_num_timesteps=False,
    )

    env.close()

    return model


def run_training_tensorboard(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training and store the logs to tensorboard.

    This avoids using WandB and stores logs only locally. Only stores the final model.
    Stores the model and the config in `models/{run_id}`.

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    run_id = "%05i" % np.random.randint(100_000) if config.run.id is None else config.run.id

    os.makedirs(f"models/{run_id}", exist_ok=True)
    with open(f"models/{run_id}/config.yaml", "w") as f:
        f.write(OmegaConf.to_yaml(config, resolve=True))

    env = create_training_vec_env(config=config, evaluation_mode=False)
    model = create_model(config=config, env=env, run_id=run_id, save_logs=True)
    model.learn(
        total_timesteps=config.run.n_steps,
        log_interval=1,
        reset_num_timesteps=config.run.load_step is None,
    )

    model.save(path=f"models/{run_id}/model_final")

    env.close()

    return model


def run_training_wandb(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training and store the logs to WandB.

    The WandB run is closed at the end of the training.
    Models are saved in regular intervals and at the end of the training.
    The frequency is specified in `config.run.save_freq`.
    Stores the model and the config in `models/{run_id}`.

    Link: https://wandb.ai

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    with init_wandb(config=config) as run:
        os.makedirs(f"models/{run.id}", exist_ok=True)
        with open(f"models/{run.id}/config.yaml", "w") as f:
            f.write(OmegaConf.to_yaml(config, resolve=True))

        env = create_training_vec_env(config=config, evaluation_mode=False)
        model = get_model(config=config, env=env, run_id=run.id, save_logs=True)
        callback = TensorboardCallback(
            eval_env=env,
            gradient_save_freq=100,
            model_save_path=f"models/{run.id}",
            verbose=2,
            save_freq=config.run.save_freq,
            model_file=f"models/{run.id}",
            start_episode=model._episode_num,
            additional_log_info_keys=config.run.log_info_keys,
            n_eval_episodes=0,
            deterministic=True,
            log_interval=config.run.log_interval,
        )

        model.learn(
            total_timesteps=config.run.n_steps,
            log_interval=1,
            reset_num_timesteps=config.run.load_step is None,
            callback=callback,
        )

        model.save(f"models/{run.id}/model_final")

        env.close()

        return model


def run_training(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training according to the config.

    Depending on the type specified in the run sub-config, logs are either
        -not stored at all (debug)
        -stored as tensorboard logs (tensorboard)
        -uploaded to WandB (wandb)

    Models are stored either
        -not at all (debug)
        -once, at the end of training (tensorboard)
        -in regular intervals and at the end of training (wandb)

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    if config.run.type == "tensorboard":
        return run_training_tensorboard(config=config)
    elif config.run.type == "wandb":
        return run_training_wandb(config=config)
    else:
        print("Performing debug run without storing to disk")
        return run_debug_training(config=config)


def evaluate_model_simple(config: TrainingConfig, model: BaseAlgorithm, eval_env: VecEnv):
    """Evaluate a model and print the reward mean and std.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
    (`config.run.eval_seed` instead of `config.run.seed`)

    Args:
        config (TrainingConfig): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
        eval_env (VecEnv): The environment to evaluate the model in
    """
    model.set_env(env=eval_env)

    mean_reward, std_reward = evaluate_policy(
        model=model,
        env=eval_env,
        n_eval_episodes=config.run.n_test_episodes,
        deterministic=True,
        return_episode_rewards=True,
    )

    print(f"Mean evaluation reward: {mean_reward} +/- {std_reward}")


def evaluate_model_wandb(config: TrainingConfig, model: BaseAlgorithm, eval_env: VecEnv):
    """Evaluate a model and upload the results to WandB.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
    (`config.run.eval_seed` instead of `config.run.seed`)

    Args:
        config (TrainingConfig): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
        env (VecEnv): The environment to use for evaluation.
    """
    with init_wandb(config=config):
        model.set_env(env=eval_env)

        callback = TensorboardCallback(
            eval_env=eval_env,
            verbose=2,
            additional_log_info_keys=config.run.log_info_keys,
            n_eval_episodes=config.run.n_test_episodes,
            deterministic=True,
        )

        model.learn(
            total_timesteps=0,
            log_interval=config.run.log_interval,
            callback=callback,
        )


def evaluate_model(config: TrainingConfig, model: BaseAlgorithm, eval_env: VecEnv):
    """Evaluate a model and either print the results to the console or upload them to WandB.

    Which method is used depends on the `type` specified in the run sub-config.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
    (`config.run.eval_seed` instead of `config.run.seed`)

    Args:
        config (TrainingConfig): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
        eval_env (VecEnv): The environment to evaluate the model in
    """
    if config.run.type == "wandb":
        evaluate_model_wandb(config=config, model=model, eval_env=eval_env)
    else:
        evaluate_model_simple(config=config, model=model, eval_env=eval_env)


def load_and_evaluate_model(config: TrainingConfig):
    """Load a model from disk and evaluate it.

    Results are either printed to the console or uploaded to WandB.
    Which method is used depends on the `type` specified in the run sub-config.

    The environment created for evaluation uses the evaluation seed
    specified in the config (`config.run.eval_seed`).

    Args:
        config (TrainingConfig): The config object containing information about the model to load
            and the evaluation environment
    """
    eval_env = create_training_vec_env(config=config, evaluation_mode=True)
    model = load_model(
        config=config,
        env=eval_env,
        run_id=config.run.id,
        load_step=config.run.load_step
    )

    evaluate_model(config=config, model=model, eval_env=eval_env)

    eval_env.close()


def train_and_evaluate(config: TrainingConfig):
    """Train a model and evaluate it.

    If the `test_only` flag is set in the run sub-config,
    the model is only loaded and not trained before evaluation.
    For evaluation a new environment is created, which uses the evaluation seed specified in the config
    (`config.run.eval_seed`).

    Args:
        config (TrainingConfig): The config object describing the environment, model, and training process
    """
    if config.run.test_only:
        load_and_evaluate_model(config=config)
    else:
        model = run_training(config=config)
        if config.run.verbose:
            print("Finished training, evaluating model...")
        eval_env = create_training_vec_env(config=config, evaluation_mode=True)
        evaluate_model(config=config, model=model, eval_env=eval_env)
