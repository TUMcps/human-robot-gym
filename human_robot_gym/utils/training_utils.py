from typing import Any, Callable, Dict, List, Union
from copy import deepcopy

from omegaconf import OmegaConf
import wandb

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
    controller_config_path = file_path_completion(config.robot.controller_config_path)
    robot_config_path = file_path_completion(config.robot.robot_config_path)

    controller_config = merge_configs(
        load_controller_config(custom_fpath=controller_config_path),
        load_controller_config(custom_fpath=robot_config_path),
    )

    return controller_config


def create_environment(config: Config) -> VecEnv:
    wrapper_class = get_environment_wrap_fn(config)

    env_kwargs = deepcopy(config.environment.kwargs)

    # Add missing kwargs
    env_kwargs.seed = config.training.seed
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
    def wrap_fn(env):
        if config.wrappers.collision_prevention is not None and config.wrappers.collision_prevention.enabled:
            env = CollisionPreventionWrapper(
                env=env,
                collision_check_fn=env.check_collision_action,
                **config.wrappers.collision_prevention.kwargs,
            )

        if config.wrappers.visualization is not None and config.wrappers.visualization.enabled:
            env = VisualizationWrapper(env=env)

        if config.wrappers.ik_position_delta is not None and config.wrappers.ik_position_delta.enabled:
            env = IKPositionDeltaWrapper(
                env=env,
                **config.wrappers.ik_position_delta.kwargs
            )

        return env

    return wrap_fn


def create_model(env: VecEnv, config: Config, run_id: str, save_logs: bool) -> BaseAlgorithm:
    if config.training.verbose:
        print(f"Creating new model for run {run_id}")

    kwargs = deepcopy(config.algorithm.kwargs)
    kwargs = OmegaConf.to_container(cfg=kwargs, resolve=True, throw_on_missing=False)
    kwargs["env"] = env
    kwargs["seed"] = config.training.seed
    kwargs["verbose"] = 1 if config.training.verbose else 0
    kwargs["tensorboard_log"] = f"runs/{run_id}" if save_logs else None
    if "train_freq" in kwargs and isinstance(kwargs["train_freq"], list):
        kwargs["train_freq"] = tuple(kwargs["train_freq"])

    if config.training.env_type == "goal_env":
        assert issubclass(SB3_ALGORITHMS[config.algorithm.name], OffPolicyAlgorithm), \
            "Only off-policy algorithms accepted for goal environments!"

        assert config.training.n_envs == 1, "HER does not support vectorized environments!"

        HerReplayBuffer.add = custom_add
        HerReplayBuffer._sample_transitions = _custom_sample_transitions

        kwargs["replay_buffer_class"] = HerReplayBuffer

    return SB3_ALGORITHMS[config.algorithm.name](**kwargs)


def load_model(env: VecEnv, config: Config, run_id: str, load_episode: Union[int, str]) -> BaseAlgorithm:
    if isinstance(load_episode, int) and load_episode < 0:
        raise ValueError("load_episode must be a positive integer or 'final'")

    if config.training.verbose:
        print(f"Loading model from run {run_id} at episode {load_episode}")

    model = SB3_ALGORITHMS[config.algorithm.name].load(
        f"models/{run_id}/model_{load_episode}.zip",
        env=env,
    )

    model.set_env(env)

    if isinstance(model, OffPolicyAlgorithm):
        model.load_replay_buffer(f"models/{run_id}/replay_buffer.pkl")

    model.env.reset()

    return model


def get_model(env: VecEnv, config: Config, run_id: str, save_logs: bool) -> BaseAlgorithm:
    if config.training.load_episode is None:
        return create_model(env=env, config=config, run_id=run_id, save_logs=save_logs)
    else:
        return load_model(env=env, config=config, run_id=run_id, load_episode=config.training.load_episode)


def init_wandb(config: Config) -> Any:
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
    env = create_environment(config=config)
    model = create_model(env, config, "", save_logs=False)
    model.learn(
        total_timesteps=config.training.n_steps,
        log_interval=1,
        reset_num_timesteps=False,
    )

    env.close()

    return model


def run_training_tensorboard(config: Config) -> BaseAlgorithm:
    run_id = "%05i" % np.random.randint(100_000) if config.training.run_id is None else config.training.run_id
    env = create_environment(config=config)
    model = create_model(env, config, run_id, save_logs=True)
    model.learn(
        total_timesteps=config.training.n_steps,
        log_interval=1,
        reset_num_timesteps=config.training.load_episode is None,
    )

    env.close()

    return model


def run_training_wandb(config: Config) -> BaseAlgorithm:
    with init_wandb(config) as run:
        env = create_environment(config=config)
        model = get_model(env=env, config=config, run_id=run.id, save_logs=True)
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
    if config.training.run_type == "tensorboard":
        return run_training_tensorboard(config)
    elif config.training.run_type == "wandb":
        return run_training_wandb(config)
    else:
        print("Performing debug run without storing to disk")
        return run_debug_training(config)


def evaluate_model_simple(model: BaseAlgorithm, config: Config):
    mean_reward, std_reward = evaluate_policy(
        model=model,
        env=model.get_env(),
        n_eval_episodes=config.training.n_test_episodes,
        deterministic=True,
        return_episode_rewards=True,
    )

    print(f"Mean evaluation reward: {mean_reward} +/- {std_reward}")


def evaluate_model_wandb(model: BaseAlgorithm, config: Config):
    with init_wandb(config):
        callback = TensorboardCallback(
            eval_env=model.get_env(),
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


def evaluate_model(model: BaseAlgorithm, config: Config):
    if config.training.run_type == "wandb":
        evaluate_model_wandb(model, config)
    else:
        evaluate_model_simple(model, config)


def load_and_evaluate_model(config: Config):
    env = create_environment(config=config)
    model = load_model(
        env=env,
        config=config,
        run_id=config.training.run_id,
        load_episode=config.training.load_episode
    )

    evaluate_model(model, config)

    env.close()


def train_and_evaluate(config: Config):
    if config.training.test_only:
        load_and_evaluate_model(config)
    else:
        model = run_training(config)
        if config.training.verbose:
            print("Finished training, evaluating model...")
        model.set_env(create_environment(config=config))
        evaluate_model(model, config)
