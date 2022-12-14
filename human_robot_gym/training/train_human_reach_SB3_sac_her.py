#!/usr/bin/env python
"""This file describes the training functionality for a human reach env with SB3 SAC + HER.

Commandline args:
    config (str): Name of the config file.
    --wandb (optional): Use wandb to log training online.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import struct
import numpy as np
import argparse
import json
from datetime import datetime
from functools import partial
import gym

from stable_baselines3 import SAC, HerReplayBuffer
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv  # noqa: F401
from stable_baselines3.common.evaluation import evaluate_policy
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.env_util import make_vec_env
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper
from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import (
    custom_add,
    _custom_sample_transitions,
)
import human_robot_gym.robots  # noqa: F401
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401


def wrap_environment(
    env: gym.Env,
    use_collision_wrapper: bool = False,
    replace_type: int = 0,
    n_resamples: int = 20,
    has_renderer: bool = False,
) -> gym.Env:
    if use_collision_wrapper:
        env = CollisionPreventionWrapper(
            env=env,
            collision_check_fn=env.check_collision_action,
            replace_type=replace_type,
            n_resamples=n_resamples
        )
    if has_renderer:
        env = VisualizationWrapper(env)
    return env


# Command line arguments:
# Training config file
# Use wandb
def init_argparse() -> argparse.ArgumentParser:
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

    return parser


if __name__ == "__main__":
    parser = init_argparse()
    args = parser.parse_args()
    if not args.config:
        raise Exception("Config file is not specified.")
    training_config_file = args.config
    use_wandb = args.wandb

    callback = None
    if use_wandb:
        import wandb
        from human_robot_gym.wrappers.tensorboard_callback import TensorboardCallback

    # Training config
    training_config_path = file_path_completion(
        "training/config/" + training_config_file
    )
    try:
        with open(training_config_path) as f:
            training_config = json.load(f)
    except FileNotFoundError:
        print(
            "Error opening controller filepath at: {}. "
            "Please check filepath and try again.".format(training_config_path)
        )

    # Load robot and controller config files
    controller_config = dict()
    controller_conig_path = file_path_completion(
        training_config["robot"]["controller_config"]
    )
    robot_conig_path = file_path_completion(training_config["robot"]["robot_config"])
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    training_config["environment"]["controller_configs"] = [controller_config]

    env_kwargs = env_kwargs = {
        "robots": training_config["robot"]["name"],
        "robot_base_offset": training_config["environment"]["robot_base_offset"],
        "env_configuration": training_config["environment"]["env_configuration"],
        "controller_configs": training_config["environment"]["controller_configs"],
        "gripper_types": training_config["environment"]["gripper_types"],
        "initialization_noise": training_config["environment"]["initialization_noise"],
        "table_full_size": training_config["environment"]["table_full_size"],
        "table_friction": training_config["environment"]["table_friction"],
        "use_camera_obs": training_config["environment"]["use_camera_obs"],
        "use_object_obs": training_config["environment"]["use_object_obs"],
        "reward_scale": training_config["environment"]["reward_scale"],
        "reward_shaping": training_config["environment"]["reward_shaping"],
        "goal_dist": training_config["environment"]["goal_dist"],
        "collision_reward": training_config["environment"]["collision_reward"],
        "goal_reward": training_config["environment"]["goal_reward"],
        "has_renderer": False,
        "has_offscreen_renderer": training_config["environment"]["has_offscreen_renderer"],
        "render_camera": training_config["environment"]["render_camera"],
        "render_collision_mesh": training_config["environment"]["render_collision_mesh"],
        "render_visual_mesh": training_config["environment"]["render_visual_mesh"],
        "render_gpu_device_id": training_config["environment"]["render_gpu_device_id"],
        "control_freq": training_config["environment"]["control_freq"],
        "horizon": training_config["environment"]["horizon"],
        "ignore_done": training_config["environment"]["ignore_done"],
        "hard_reset": training_config["environment"]["hard_reset"],
        "camera_names": training_config["environment"]["camera_names"],
        "camera_heights": training_config["environment"]["camera_heights"],
        "camera_widths": training_config["environment"]["camera_widths"],
        "camera_depths": training_config["environment"]["camera_depths"],
        "camera_segmentations": training_config["environment"]["camera_segmentations"],
        "renderer": training_config["environment"]["renderer"],
        "renderer_config": training_config["environment"]["renderer_config"],
        "use_failsafe_controller": training_config["environment"]["use_failsafe_controller"],
        "visualize_failsafe_controller": training_config["environment"]["visualize_failsafe_controller"],
        "visualize_pinocchio": training_config["environment"]["visualize_pinocchio"],
        "control_sample_time": training_config["environment"]["control_sample_time"],
        "human_animation_names": training_config["environment"]["human_animation_names"],
        "base_human_pos_offset": training_config["environment"]["base_human_pos_offset"],
        "human_animation_freq": training_config["environment"]["human_animation_freq"],
        "safe_vel": training_config["environment"]["safe_vel"],
        "randomize_initial_pos": training_config["environment"]["randomize_initial_pos"],
        "self_collision_safety": training_config["environment"]["self_collision_safety"],
        "done_at_collision": training_config["environment"]["done_at_collision"],
        "done_at_success": training_config["environment"]["done_at_success"],
        "seed": training_config["training"]["seed"],
    }
    wrapper_cls = partial(wrap_environment,
                          use_collision_wrapper=True,
                          replace_type=training_config["environment"]["replace_type"],
                          has_renderer=training_config["environment"]["has_renderer"],)
    n_envs = training_config["training"]["n_envs"]
    assert n_envs == 1, "HER in SB3 is currently only supported for single environments."
    env = make_vec_env(
        env_id="ReachHuman",
        type="goal_env",
        obs_keys=training_config["environment"]["obs_keys"],
        n_envs=n_envs,
        env_kwargs=env_kwargs,
        vec_env_cls=DummyVecEnv if n_envs == 1 else SubprocVecEnv,
        wrapper_class=wrapper_cls,
    )

    now = datetime.now()
    load_episode = -1
    if "load_episode" in training_config["training"]:
        load_episode = training_config["training"]["load_episode"]
        if load_episode >= 0:
            if "run_id" in training_config["training"]:
                run_id = training_config["training"]["run_id"]
            else:
                print("Please provide a run_id in the config in the algorithm section.")

    last_time_steps = np.ndarray(0)

    # Monitor and log the training
    # env = Monitor(env, filename=outdir, info_keywords=("collision", "criticalCollision", "goalReached"))

    HerReplayBuffer.add = custom_add
    HerReplayBuffer._sample_transitions = _custom_sample_transitions
    start_episode = 0
    if load_episode == -1:
        # << Defining the "run" for weights and biases >>
        if use_wandb:
            run = wandb.init(
                project="safe_human_robot_rl",
                config=training_config,
                sync_tensorboard=True,
                monitor_gym=False,
                save_code=False,
            )
        else:
            run = struct
            run.id = int(np.random.rand(1) * 100000)
        # << Initialize the model >>
        if isinstance(training_config["algorithm"]["train_freq"], list):
            training_config["algorithm"]["train_freq"] = tuple(training_config["algorithm"]["train_freq"])
        model = SAC(
            "MultiInputPolicy",
            env,
            replay_buffer_class=HerReplayBuffer,
            verbose=1,
            seed=training_config["training"]["seed"],
            tensorboard_log=f"runs/{run.id}",
            device="auto",
            _init_setup_model=True,
            **training_config["algorithm"]
        )
    else:
        # << Defining the "run" for weights and biases >>
        if use_wandb:
            run = wandb.init(
                project="safe_human_robot_rl",
                config=training_config["algorithm"],
                sync_tensorboard=True,
                monitor_gym=False,
                save_code=False,
                resume="must",
                id=run_id,
            )
        # << Load the model >>
        model = SAC.load(
            "{}/model_{}".format(f"models/{run_id}", str(load_episode)), env=env
        )
        # load it into the loaded_model
        model.load_replay_buffer("{}/replay_buffer.pkl".format(f"models/{run_id}"))
        start_episode = model._episode_num
        model.set_env(env)
        model.env.reset()

    if use_wandb:
        callback = TensorboardCallback(
            eval_env=env,
            gradient_save_freq=100,
            model_save_path=f"models/{run.id}",
            verbose=2,
            save_freq=training_config["training"]["save_freq"],
            model_file=f"models/{run.id}",
            start_episode=start_episode,
            additional_log_info_keys=[
                "n_goal_reached",
                "collision",
                "collision_type",
                "n_collisions",
                "n_collisions_static",
                "n_collisions_robot",
                "n_collisions_human",
                "n_collisions_critical",
                "timeout",
                "failsafe_interventions",
                "action_resamples",
            ],
            n_eval_episodes=0,
            deterministic=True,
            log_interval=training_config["training"]["log_interval"],
        )
    # << Train the agent >>
    if not training_config["training"]["test_only"]:
        model.learn(
            total_timesteps=training_config["training"]["n_steps"],
            log_interval=1,
            reset_num_timesteps=(load_episode == -1),
            callback=callback
        )
        model.save(f"models/{run.id}/model_final")

    # << Evaluate the agent >>
    if use_wandb:
        callback = TensorboardCallback(
            eval_env=env,
            verbose=2,
            additional_log_info_keys=[
                "n_goal_reached",
                "collision",
                "collision_type",
                "n_collisions",
                "n_collisions_static",
                "n_collisions_robot",
                "n_collisions_human",
                "n_collisions_critical",
                "timeout",
                "failsafe_interventions",
                "action_resamples",
            ],
            n_eval_episodes=training_config["training"]["num_test_episodes"],
            deterministic=True,
        )
        # << Evaluate model >>
        model.learn(
            total_timesteps=0,
            log_interval=training_config["training"]["log_interval"],
            callback=callback,
        )
    else:
        mean_reward, std_reward = evaluate_policy(
            model=model,
            env=env,
            n_eval_episodes=training_config["training"]["num_test_episodes"],
            deterministic=True,
            return_episode_rewards=True
        )
        print("Mean evaluation reward: {} +/- {}".format(mean_reward, std_reward))
    # Close everything
    if use_wandb:
        run.finish()
    env.close()
