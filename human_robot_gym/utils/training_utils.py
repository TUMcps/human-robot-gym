"""This file defines utility functions for setting up trainings from hydra config files.

Author:
    Felix Trost (FT)

Changelog:
    04.04.23 (FT) File created
"""
from typing import Any, Callable, Dict, List, Optional, Union
from copy import deepcopy

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

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.env_util import make_vec_env
from human_robot_gym.utils.config_utils import Config

from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper
from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import custom_add, _custom_sample_transitions
from human_robot_gym.wrappers.tensorboard_callback import TensorboardCallback


SB3_ALGORITHMS = {
    "SAC": SAC,
    "PPO": PPO,
}


def get_controller_configs(config: Config) -> List[Dict[str, Any]]:
    """Obtain the controller config for the robot from the paths specified in the config.

    We merge a robot specific config into the config for the controller (e.g. a failsafe controller).

    Args:
        config (Config): The training config containing the paths to the controller and robot configs.

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


def create_environment(config: Config, evaluation_mode: bool = False) -> VecEnv:
    """Create an environment from a config and optionally wrap it in specified wrappers.

    The environment seed, robots and controller_configs arguments are overwritten:
        -seed: we take the seed from the training sub-config (eval_seed if evaluation_mode is True)
        -robots: we take the robot name from the robot sub-config
        -controller_configs: we assemble the controller configs using the robot sub-config

    If the config specifies more than one environment, they are run on different threads using a SubprocVecEnv.
    Otherwise, a DummyVecEnv is created.

    Args:
        config (Config): The config object containing information about the environment and optional wrappers
        evaluation_mode (bool): If True, the evaluation seed is used. Defaults to False.
    """
    wrapper_class = get_environment_wrap_fn(config)

    # Create copy of the config to avoid modifying the original config
    env_kwargs = deepcopy(config.environment.kwargs)

    # Add missing kwargs
    env_kwargs.seed = config.training.eval_seed if evaluation_mode else config.training.seed
    env_kwargs.robots = config.robot.name
    env_kwargs.controller_configs = get_controller_configs(config)

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
        env_kwargs=OmegaConf.to_container(cfg=env_kwargs, resolve=True, throw_on_missing=True),
        vec_env_cls=DummyVecEnv if config.training.n_envs == 1 else SubprocVecEnv,
        vec_env_kwargs=config.training.vec_env_kwargs,
        monitor_kwargs=config.training.monitor_kwargs,
        wrapper_kwargs=None,
    )

    return env


def get_environment_wrap_fn(config: Config) -> Callable[[gym.Env], gym.Env]:
    """Create a function that wraps the environment as specified in the config.

    Args:
        config (Config): The config object containing information about the wrappers

    Returns:
        Callable[[gym.Env], gym.Env]: A function that wraps the environment as specified in the config.
    """
    def wrap_fn(env: gym.Env):
        if (
            hasattr(config.wrappers, "collision_prevention") and
            config.wrappers.collision_prevention is not None and
            config.wrappers.collision_prevention.enabled
        ):
            env = CollisionPreventionWrapper(
                env=env,
                collision_check_fn=env.check_collision_action,
                **config.wrappers.collision_prevention.kwargs,
            )

        if (
            hasattr(config.wrappers, "visualization") and
            config.wrappers.visualization is not None and
            config.wrappers.visualization.enabled
        ):
            env = VisualizationWrapper(env=env)

        if (
            hasattr(config.wrappers, "ik_position_delta") and
            config.wrappers.ik_position_delta is not None and
            config.wrappers.ik_position_delta.enabled
        ):
            env = IKPositionDeltaWrapper(
                env=env,
                **config.wrappers.ik_position_delta.kwargs
            )

        return env

    return wrap_fn


def create_model(
    config: Config,
    save_logs: bool,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
) -> BaseAlgorithm:
    """Create a new model according to the config for the given environment.

    The model env, seed, verbose and tensorboard_log arguments are overwritten:
        -env: is not serializable, created with the parameters specified in the config
        -seed: we take the seed from the training sub-config
        -verbose: we take the verbose flag from the training sub-config
        -tensorboard_log: path generated according to run id

    Additionally, if the env_type is set to 'goal_env' in the training sub-config,
    we use a HerReplayBuffer to support Hindsight Experience Replay.

    Args:
        config (Config): The config object containing information about the model
        save_logs (bool): Whether to save logs to tensorboard (or WandB)
        env (VecEnv | None): The environment to train the model on. Can be set to None for later initialization.
        run_id (str | None): Can be set to override the run id specified in the training sub-config,
            as this value is usually set to None in training runs.

    Returns:
        BaseAlgorithm: The model created according to the config for the given environment.

    Raises:
        AssertionError: [Only off-policy algorithms accepted for goal environments (HER)]
        AssertionError: [HER does not support vectorized environments!]
    """
    if run_id is None:
        run_id = config.training.run_id

    if config.training.verbose:
        print(f"Creating new model for run {run_id}")

    kwargs = deepcopy(config.algorithm.kwargs)
    kwargs = OmegaConf.to_container(cfg=kwargs, resolve=True, throw_on_missing=False)
    kwargs["env"] = env
    kwargs["seed"] = config.training.seed
    kwargs["verbose"] = 1 if config.training.verbose else 0
    kwargs["tensorboard_log"] = f"runs/{run_id}" if save_logs else None

    # Stable-baselines3 throws an error if train_freq is a list (expects tuple or int)
    if "train_freq" in kwargs and isinstance(kwargs["train_freq"], list):
        kwargs["train_freq"] = tuple(kwargs["train_freq"])

    # Support Hindsight Experience Replay (HER)
    if config.training.env_type == "goal_env":
        assert issubclass(SB3_ALGORITHMS[config.algorithm.name], OffPolicyAlgorithm), \
            "Only off-policy algorithms accepted for goal environments (HER)"

        assert config.training.n_envs == 1, "HER does not support vectorized environments!"

        HerReplayBuffer.add = custom_add
        HerReplayBuffer._sample_transitions = _custom_sample_transitions

        kwargs["replay_buffer_class"] = HerReplayBuffer

    return SB3_ALGORITHMS[config.algorithm.name](**kwargs)


def load_model(
    config: Config,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None,
    load_episode: Optional[Union[int, str]] = None,
) -> BaseAlgorithm:
    """Load a model from disk.

    If the model uses an off-policy algorithm, the replay buffer is also loaded.

    Args:
        config (Config): The config object containing information about the model
        env (VecEnv | None): The environment to use. Can be set to None for later initialization.
        run_id (str | None): The run id to load the model from.
            Can be set to override the value specified in the training sub-config
        load_episode (int | str | None): The episode to load the model from.
            Can be set to override the value specified in the training sub-config
            There has to exist a model file at models/{run_id}/model_{load_episode}.zip
            Generally, load_episode should be set to a positive integer or 'final'

    Returns:
        BaseAlgorithm: The loaded model

    Raises:
        ValueError: [load_episode must be a positive integer or 'final']
    """
    if run_id is None:
        run_id = config.training.run_id

    if load_episode is None:
        load_episode = config.training.load_episode

    if isinstance(load_episode, int) and load_episode < 0:
        raise ValueError("load_episode must be a positive integer or 'final'")

    if config.training.verbose:
        print(f"Loading model from run {run_id} at episode {load_episode}")

    model = SB3_ALGORITHMS[config.algorithm.name].load(
        f"models/{run_id}/model_{load_episode}.zip",
        env=env,
    )

    if env is not None:
        model.set_env(env)

    if isinstance(model, OffPolicyAlgorithm):
        model.load_replay_buffer(f"models/{run_id}/replay_buffer.pkl")

    return model


def get_model(
    config: Config,
    save_logs: bool,
    env: Optional[VecEnv] = None,
    run_id: Optional[str] = None
) -> BaseAlgorithm:
    """Create a new model or load an existing model from disk,
        depending on whether a load episode is specified in the training sub-config.

    Args:
        config (Config): The config object containing information about the model
        save_logs (bool): Whether to save logs to tensorboard (or WandB)
        env (VecEnv | None): The environment to use. Can be set to None for later initialization.
        run_id (str | None): Can be set to override the run id specified in the training sub-config,
            as this value is usually set to None in training runs.

    Returns:
        BaseAlgorithm: The model created according to the config for the given environment.
    """
    if config.training.load_episode is None:
        return create_model(config=config, env=env, save_logs=save_logs, run_id=run_id)
    else:
        return load_model(config=config, env=env, run_id=run_id, load_episode=config.training.load_episode)


def init_wandb(config: Config) -> Run:
    """Initialize WandB.

    If a run_id is already specified in the training sub-config, this run will be resumed.
    Otherwise, a new run is created.

    Args:
        config (Config): The config containing information about the run

    Returns:
        Run: The WandB run object

    Raises:
        ValueError: [Cannot load a model without a run_id]
    """
    if config.training.run_id is None and config.training.load_episode is not None:
        raise ValueError("Cannot load a model without a run_id")

    run_id = config.training.run_id

    if config.training.run_id is not None and config.training.load_episode is None:
        print("load_episode not specified, will ignore run_id and create a new run")
        run_id = None

    resume = None if config.training.run_id is None else "must"

    config_dict = OmegaConf.to_container(
        cfg=config,
        resolve=True,
        throw_on_missing=False,
    )

    return wandb.init(
        project=config.wandb.project,
        entity=config.wandb.entity,
        group=config.wandb.group,
        name=config.wandb.name,
        tags=config.wandb.tags,
        config=config_dict,
        save_code=False,
        monitor_gym=True,
        sync_tensorboard=True,
        id=run_id,
        resume=resume,
    )


def run_debug_training(config: Config) -> BaseAlgorithm:
    """Run a training without storing any data to disk.

    This is useful for debugging purposes to avoid creating a lot of unnecessary files.

    Args:
        config (Config): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    env = create_environment(config=config)
    model = create_model(config=config, save_logs=False, env=env, run_id="~~debug~~")
    model.learn(
        total_timesteps=config.training.n_steps,
        log_interval=1,
        reset_num_timesteps=False,
    )

    env.close()

    return model


def run_training_tensorboard(config: Config) -> BaseAlgorithm:
    """Run a training and store the logs to tensorboard.

    This avoids using WandB and stores logs only locally. Only stores the final model.

    Args:
        config (Config): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    run_id = "%05i" % np.random.randint(100_000) if config.training.run_id is None else config.training.run_id
    env = create_environment(config=config)
    model = create_model(config=config, save_logs=True, env=env, run_id=run_id)
    model.learn(
        total_timesteps=config.training.n_steps,
        log_interval=1,
        reset_num_timesteps=config.training.load_episode is None,
    )

    model.save(f"models/{run_id}/model_final")

    env.close()

    return model


def run_training_wandb(config: Config) -> BaseAlgorithm:
    """Run a training and store the logs to WandB.

    The WandB run is closed at the end of the training.
    Models are saved in regular intervals and at the end of the training.
    The frequency is specified in @config.training.save_freq.

    Link: https://wandb.ai

    Args:
        config (Config): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    with init_wandb(config) as run:
        env = create_environment(config=config)
        model = get_model(config=config, save_logs=True, env=env, run_id=run.id)
        callback = TensorboardCallback(
            eval_env=env,
            gradient_save_freq=100,
            model_save_path=f"models/{run.id}",
            verbose=2,
            save_freq=config.training.save_freq,
            model_file=f"models/{run.id}",
            start_episode=model._episode_num,
            additional_log_info_keys=config.training.log_info_keys,
            n_eval_episodes=0,
            deterministic=True,
            log_interval=config.training.log_interval,
        )

        model.learn(
            total_timesteps=config.training.n_steps,
            log_interval=1,
            reset_num_timesteps=config.training.load_episode is None,
            callback=callback,
        )

        model.save(f"models/{run.id}/model_final")

        env.close()

        return model


def run_training(config: Config) -> BaseAlgorithm:
    """Run a training according to the config.

    Depending on the run_type specified in the training sub-config, logs are either
        -not stored at all (debug)
        -stored as tensorboard logs (tensorboard)
        -uploaded to WandB (wandb)

    Models are stored either
        -not at all (debug)
        -once, at the end of training (tensorboard)
        -in regular intervals and at the end of training (wandb)

    Args:
        config (Config): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    if config.training.run_type == "tensorboard":
        return run_training_tensorboard(config)
    elif config.training.run_type == "wandb":
        return run_training_wandb(config)
    else:
        print("Performing debug run without storing to disk")
        return run_debug_training(config)


def evaluate_model_simple(config: Config, model: BaseAlgorithm):
    """Evaluate a model and print the reward mean and std.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
        (config.training.eval_seed instead of config.training.seed)

    Args:
        config (Config): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
    """
    env = create_environment(config=config, evaluation_mode=True)
    model.set_env(env)

    mean_reward, std_reward = evaluate_policy(
        model=model,
        env=env,
        n_eval_episodes=config.training.n_test_episodes,
        deterministic=True,
        return_episode_rewards=True,
    )

    print(f"Mean evaluation reward: {mean_reward} +/- {std_reward}")

    env.close()


def evaluate_model_wandb(config: Config, model: BaseAlgorithm):
    """Evaluate a model and upload the results to WandB.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
        (config.training.eval_seed instead of config.training.seed)

    Args:
        config (Config): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
    """
    with init_wandb(config):
        env = create_environment(config=config, evaluation_mode=True)
        model.set_env(env)

        callback = TensorboardCallback(
            eval_env=env,
            verbose=2,
            additional_log_info_keys=config.training.log_info_keys,
            n_eval_episodes=config.training.n_test_episodes,
            deterministic=True,
        )

        model.learn(
            total_timesteps=0,
            log_interval=config.training.log_interval,
            callback=callback,
        )

        env.close()


def evaluate_model(config: Config, model: BaseAlgorithm):
    """Evaluate a model and either print the results to the console or upload them to WandB.

    Which method is used depends on the run_type specified in the training sub-config.

    Creates a new environment based on the information from the config.
    The environment is created in evaluation mode, which employs a different seed
        (config.training.eval_seed instead of config.training.seed)

    Args:
        config (Config): The config object containing information about the model
        model (BaseAlgorithm): The model to evaluate
    """
    if config.training.run_type == "wandb":
        evaluate_model_wandb(config=config, model=model)
    else:
        evaluate_model_simple(config=config, model=model)


def load_and_evaluate_model(config: Config):
    """Load a model from disk and evaluate it.

    Results are either printed to the console or uploaded to WandB.
    Which method is used depends on the run_type specified in the training sub-config.

    The environment created for evaluation uses the evaluation seed specified in the config (config.training.eval_seed).

    Args:
        config (Config): The config object containing information about the model to load and the evaluation environment
    """
    model = load_model(
        config=config,
        env=None,
        run_id=config.training.run_id,
        load_episode=config.training.load_episode
    )

    evaluate_model(config=config, model=model)


def train_and_evaluate(config: Config):
    """Train a model and evaluate it.

    If the test_only flag is set in the training sub-config, the model is only loaded and not trained before evaluation.
    For evaluation a new environment is created, which uses the evaluation seed specified in the config
        (config.training.eval_seed).

    Args:
        config (Config): The config object describing the environment, model, and training process
    """
    if config.training.test_only:
        load_and_evaluate_model(config)
    else:
        model = run_training(config)
        if config.training.verbose:
            print("Finished training, evaluating model...")
        evaluate_model(model, config)
