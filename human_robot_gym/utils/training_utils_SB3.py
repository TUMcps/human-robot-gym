"""This file defines utility functions for setting up trainings from hydra config files.

Author:
    Martin Winter (MW)

Changelog:
    16.07.23 MW moved all SB3 specific code from human_robot_gym/utils/training_utils.py to
    human_robot_gym/utils/training_utils_SB3.py
"""
from typing import Any, Dict, Optional, Union
from copy import deepcopy
import os

from omegaconf import OmegaConf

import numpy as np
import gym

from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.off_policy_algorithm import OffPolicyAlgorithm
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
from stable_baselines3 import SAC, PPO, HerReplayBuffer


from human_robot_gym.utils.env_util_SB3 import make_vec_env
from human_robot_gym.utils.config_utils import TrainingConfig
from human_robot_gym.utils.training_utils import _compose_environment_kwargs, get_environment_wrap_fn, init_wandb


from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import custom_add, _custom_sample_transitions
from human_robot_gym.callbacks.custom_wandb_callback import CustomWandbCallback
from human_robot_gym.callbacks.model_reset_callback import ModelResetCallback
from human_robot_gym.callbacks.logging_callback import LoggingCallback

SB3_ALGORITHMS = {
    "SAC": SAC,
    "PPO": PPO,
}


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


def _get_tb_log_path(
    config: TrainingConfig,
    save_logs: bool,
    run_id: Optional[str] = None,
) -> Optional[str]:
    """Get the path at which to save tensorboard logs.

    Args:
        config (TrainingConfig): The config object containing information about the run
        save_logs (bool): Whether to save logs to tensorboard (or WandB)
        run_id (str | None): Can be set to override the run id specified in the run sub-config,
            as this value is usually set to `None` in training runs and generated at runtime.

    Returns:
        Optional[str]: The path at which to save tensorboard logs. Returns `None` if `save_logs` is `False`.
    """
    if save_logs:
        if config.run.type == "wandb" or (config.run.type == "tensorboard" and all(
            [config.wandb_run.project, config.wandb_run.group, config.wandb_run.name]
        )):
            return os.path.join(
                "runs",
                config.wandb_run.project,
                config.wandb_run.group,
                config.wandb_run.name,
            )
        else:
            return f"runs/{run_id}"
    else:
        return None


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
    log_path = _get_tb_log_path(config=config, save_logs=save_logs, run_id=run_id)
    if log_path is not None:
        try:
            os.makedirs(log_path, exist_ok=False)
        except OSError:
            print(f"Tensorboard log directory {log_path} already exists!")
    kwargs["tensorboard_log"] = log_path

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
        if os.path.exists(f"models/{run_id}/replay_buffer"):
            if config.run.verbose:
                print(f"Loading replay buffer from run {run_id} at episode {load_step}")
            model.load_replay_buffer(f"models/{run_id}/replay_buffer.pkl")
        else:
            if config.run.verbose:
                print(f"Replay buffer not found for run {run_id} at episode {load_step}")

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


def create_callback(
    config: TrainingConfig,
    model: BaseAlgorithm,
    env: gym.Env,
    run_id: int
) -> BaseCallback:
    """Generate callbacks for the training run based on the config.

    Args:
        config (TrainingConfig): The config object containing information about the model
        model (BaseAlgorithm): The model to train
        env (gym.Env): The environment to train the model on
        run_id (int): The run id to use for this training run
    """
    callbacks = []

    if config.run.type == "wandb":
        callbacks.append(
            CustomWandbCallback(
                eval_env=env,
                gradient_save_freq=100,
                model_save_path=f"models/{run_id}",
                verbose=2,
                save_freq=config.run.save_freq,
                model_file=f"models/{run_id}",
                start_episode=model._episode_num,
                additional_log_info_keys=config.run.log_info_keys,
                n_eval_episodes=0,
                deterministic=True,
                log_interval=config.run.log_interval,
            )
        )
    elif config.run.type == "tensorboard":
        model_path = (
            f"models/{config.wandb_run.project}/{config.wandb_run.group}/{config.wandb_run.name}"
            if all([config.wandb_run.project, config.wandb_run.group, config.wandb_run.name])
            else f"models/{run_id}"
        )

        callbacks.append(
            LoggingCallback(
                verbose=2,
                save_freq=config.run.save_freq,
                model_path=model_path,
                start_episode=model._episode_num,
                additional_log_info_keys=config.run.log_info_keys,
                log_interval=config.run.log_interval,
            )
        )

    if hasattr(config.run, "resetting_interval") and config.run.resetting_interval is not None:
        callbacks.append(
            ModelResetCallback(
                n_steps_between_resets=config.run.resetting_interval,
                total_training_timesteps=config.run.n_steps,
                reset_fn=None,
                verbose=config.run.verbose,
            )
        )

    return CallbackList(callbacks)


def _run_training_with_id(config: TrainingConfig, run_id: Optional[int]) -> BaseAlgorithm:
    """Run a training with a given run id.

    Args:
        config (TrainingConfig): The config object containing information about the model
        run_id (int | None): The run id to use for this training run
            If `None`, create a debug run with id ~~debug~~

    Returns:
        BaseAlgorithm: The trained model
    """
    debug_run_id = "~~debug~~"

    if run_id is None:
        run_id = debug_run_id

    env = create_training_vec_env(config=config, evaluation_mode=False)
    model = create_model(config=config, env=env, run_id=run_id, save_logs=run_id != debug_run_id)
    callback = create_callback(config=config, model=model, env=env, run_id=run_id)

    model.learn(
        total_timesteps=config.run.n_steps,
        log_interval=None,
        reset_num_timesteps=config.run.load_step is None,
        callback=callback,
    )

    env.close()

    return model


def run_debug_training(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training without storing any data to disk.

    This is useful for debugging purposes to avoid creating a lot of unnecessary files.

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    return _run_training_with_id(config=config, run_id=None)


def run_training_tensorboard(config: TrainingConfig) -> BaseAlgorithm:
    """Run a training and store the logs to tensorboard.

    This avoids using WandB and stores logs only locally. Only stores the final model.
    Stores the model and the config in `models/{run_id}` per default.
    If the config provides a `wandb_run` sub-config, the model and config are stored in
    `models/{wandb_run.project}/{wandb_run.group}/{wandb_run.name}` instead.

    Args:
        config (TrainingConfig): The config object containing information about the model

    Returns:
        BaseAlgorithm: The trained model
    """
    run_id = "%05i" % np.random.randint(100_000) if config.run.id is None else config.run.id

    use_wandb_run_folders = all([config.wandb_run.project, config.wandb_run.group, config.wandb_run.name])

    model_path = f"models/{run_id}"
    if use_wandb_run_folders:
        model_path = f"models/{config.wandb_run.project}/{config.wandb_run.group}/{config.wandb_run.name}"

    os.makedirs(model_path, exist_ok=True)
    with open(f"{model_path}/config.yaml", "w") as f:
        f.write(OmegaConf.to_yaml(config, resolve=True))

    model = _run_training_with_id(config=config, run_id=run_id)

    model.save(f"{model_path}/model_final")

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

        model = _run_training_with_id(config=config, run_id=run.id)

        model_folder = f"models/{config.wandb_run.project}/{config.wandb_run.group}/{config.wandb_run.name}"
        if not os.path.exists(model_folder):
            os.makedirs(model_folder)

        model.save(f"{model_folder}/model_final")

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

        callback = CustomWandbCallback(
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
