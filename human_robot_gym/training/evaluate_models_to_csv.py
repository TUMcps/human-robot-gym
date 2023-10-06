from typing import List, Optional, Union

import os
import shutil

import pandas as pd
import numpy as np

from scipy.stats import bootstrap

import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

from human_robot_gym.utils.config_utils import TrainingConfig
from human_robot_gym.utils.training_utils import create_training_vec_env, load_model, create_expert
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
import ray


def obtain_model(config: TrainingConfig, run_id: str, load_step: Optional[Union[int, str]] = None) -> str:
    model_path = os.path.join("models", run_id)

    if load_step is None:
        load_step = config.run.load_step

    if isinstance(load_step, int):
        model_path = os.path.join(model_path, f"model_{load_step:_}.zip")
    else:
        model_path = os.path.join(model_path, f"model_{load_step}.zip")

    return model_path

    if not os.path.exists(model_path):
        os.makedirs(os.path.join("models", run_id), exist_ok=True)
        print("Model directory not found locally. Looking at case...")
        command = f"scp trostf@case.cps.in.tum.de:/home/trostf/human-robot-gym/{model_path} models/{run_id}/"
        print(command)
        os.system(command)

    if not os.path.exists(model_path):
        print("Model directory not found locally or on case. Looking at glados...")
        command = f"scp trostf@glados.cps.in.tum.de:/home/trostf/human-robot-gym/{model_path} models/{run_id}/"
        print(command)
        os.system(command)

    if not os.path.exists(model_path):
        os.rmdir(os.path.join("models", run_id))
        raise ValueError(f"Model directory for run id {run_id} not found locally or on server!")

    return model_path


def eval_to_df(
    config: TrainingConfig,
    run_id: Optional[str],
    load_step: Optional[Union[int, str]] = None
) -> pd.DataFrame:
    env = create_training_vec_env(config, evaluation_mode=True)

    if load_step is None:
        load_step = config.run.load_step

    eval_expert = run_id is None

    if eval_expert:
        model = create_expert(config=config, env=env)
        expert_obs_wrappper = ExpertObsWrapper.get_from_wrapped_env(env.envs[0])
    else:
        try:
            model = load_model(
                config=config,  # Config needs to specify algorithm
                env=None,
                run_id=run_id,
                load_step=load_step
            )
        except Exception:
            print(f"Could not retrieve model for {run_id} at {load_step}")
            df = pd.DataFrame(dict(
                ep_rew=[],
                ep_len=[],
                success=[],
            ))

            for key in config.run.log_info_keys:
                df[key] = []

            return df

    different_obs = False
    if model.observation_space.shape != env.observation_space.shape:  # Fix for SIR
        different_obs = True
        env.observation_space = model.observation_space
        if os.path.exists(
            yaml_path := os.path.join("datasets", config.wrappers.dataset_obs_norm.dataset_name, "stats.yaml")
        ):
            dataset_stats = OmegaConf.load(yaml_path)
            if hasattr(dataset_stats, "ep_len_mean"):  # Compatibility
                mean_ep_len = dataset_stats.ep_len_mean
            elif hasattr(dataset_stats, "mean_ep_len"):
                mean_ep_len = dataset_stats.mean_ep_len
        elif os.path.exists(
            csv_path := os.path.join("datasets", config.wrappers.dataset_obs_norm.dataset_name, "stats.csv")
        ):
            dataset_stats = pd.read_csv(csv_path)
            mean_ep_len = dataset_stats.ep_len_mean[0]

    if not eval_expert:
        model.set_env(env)

    ep_returns = []
    ep_lengths = []
    successes = []
    ep_infos = {key: [] for key in config.run.log_info_keys}

    for _ in range(config.run.n_test_episodes):
        obs = env.reset()
        done = False
        ep_return = 0
        ep_length = 0

        step_index = 0

        while not done:
            if different_obs:
                time_value = min(step_index / mean_ep_len, 1)
                obs = np.concatenate([obs, time_value * np.ones((1, 1))], axis=1)

            if eval_expert:
                action = np.array([model(expert_obs_wrappper.current_expert_observation)])
            else:
                action, _ = model.predict(obs, deterministic=True)
            step_index += 1
            obs, reward, done, info = env.step(action)
            ep_return += reward[0]
            # print(reward)
            ep_length += 1

        successes.append(1 if info[0]["n_goal_reached"] > 0 else 0)
        for key in config.run.log_info_keys:
            ep_infos[key].append(info[0][key])

        ep_returns.append(ep_return)
        ep_lengths.append(ep_length)

    df = pd.DataFrame(dict(
        ep_rew=ep_returns,
        ep_len=ep_lengths,
        success=successes,
    ))

    for key in config.run.log_info_keys:
        df[key] = ep_infos[key]

    return df


def eval_to_csv(config: TrainingConfig, run_id: Optional[str], load_step: Optional[Union[int, str]] = None) -> str:
    df = eval_to_df(config, run_id, load_step)

    os.makedirs(os.path.join("csv", "evaluation", "raw"), exist_ok=True)

    if run_id is None:
        csv_path = os.path.join("csv", "evaluation", "raw", f"expert_{config.expert.id}.csv")
    else:
        if load_step is None:
            load_step = config.run.load_step

        if isinstance(load_step, int):
            csv_path = os.path.join("csv", "evaluation", "raw", f"{run_id}_{load_step:_}.csv")
        else:
            csv_path = os.path.join("csv", "evaluation", "raw", f"{run_id}_{load_step}.csv")
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    df.to_csv(csv_path, index=False)

    return csv_path


@ray.remote
def eval_to_csv_ray(config: TrainingConfig, run_id: Optional[str], load_step: Optional[Union[int, str]] = None) -> str:
    return eval_to_csv(config, run_id, load_step)


def combine_to_stats_df(config: TrainingConfig, run_ids: List[str], csv_paths: List[str]) -> pd.DataFrame:
    df = pd.concat([
        pd.read_csv(csv_path)
        for csv_path in csv_paths
    ])

    keys = ["ep_rew", "ep_len", "success"] + config.run.log_info_keys

    stats_df = pd.DataFrame()

    for key in keys:
        info_df = df.describe()[key]
        stats_df[f"{key}_mean"] = [info_df["mean"]]
        stats_df[f"{key}_std"] = [info_df["std"]]
        conf = bootstrap(
            df[key].values[np.newaxis, :],
            n_resamples=10000,
            confidence_level=0.95,
            statistic=np.mean,
        ).confidence_interval
        stats_df[f"{key}_025"] = [stats_df[f"{key}_mean"].values[0] if np.isnan(conf[0]) else conf[0]]
        stats_df[f"{key}_975"] = [stats_df[f"{key}_mean"].values[0] if np.isnan(conf[1]) else conf[1]]

        stats_df[f"{key}_median"] = [info_df["50%"]]
        stats_df[f"{key}_lower_quartile"] = [info_df["25%"]]
        stats_df[f"{key}_upper_quartile"] = [info_df["75%"]]

        iqr = info_df["75%"] - info_df["25%"]
        lower_whisker = df[key][df[key] > (info_df["25%"] - 1.5 * iqr)].min()
        lower_whisker = info_df["25%"] if np.isnan(lower_whisker) else lower_whisker
        upper_whisker = df[key][df[key] < (info_df["75%"] + 1.5 * iqr)].max()
        upper_whisker = info_df["75%"] if np.isnan(upper_whisker) else upper_whisker
        stats_df[f"{key}_lower_whisker"] = lower_whisker
        stats_df[f"{key}_upper_whisker"] = upper_whisker

    return stats_df


def evaluate_to_stats_df(config: TrainingConfig) -> pd.DataFrame:
    if config.run.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    run_ids = config.run.id
    if isinstance(run_ids, str):
        run_ids = [run_ids]

    load_steps = config.run.load_step
    if load_steps == "all":
        load_steps = [
            load_step + config.run.save_freq for load_step in range(0, config.run.n_steps, config.run.save_freq)
        ]
    else:
        load_steps = [load_steps]

    if run_ids is None:
        print("No run ids specified. Running evaluation using expert policy.")
        csv_paths = [eval_to_csv(config, None)]
    else:
        for run_id in run_ids:
            for load_step in load_steps:
                obtain_model(config, run_id, load_step)

    print(f"Evaluating models {run_ids} at steps {load_steps}")

    csv_paths = []
    if run_ids is None:
        csv_paths.append([eval_to_csv(config, None)])
    else:
        # csv_paths = [[] for _ in range(len(load_steps))]
        # for run_id in run_ids:
        #     results = [eval_to_csv_ray.remote(config, run_id, load_step) for load_step in load_steps]
        #     # paths = [eval_to_csv(config, run_id, load_step) for load_step in load_steps]
        #     # paths = ray.get(results)
        #     for i, path in enumerate(paths):
        #         csv_paths[i].append(path)

        results = []
        for load_step in load_steps:
            results.append([eval_to_csv_ray.remote(config, run_id, load_step) for run_id in run_ids])

        for result in results:
            csv_paths.append(ray.get(result))

    print("Got data, now obtaining stats")

    dfs = [combine_to_stats_df(config=config, run_ids=run_ids, csv_paths=paths) for paths in csv_paths]
    if len(dfs) > 1:
        df = pd.concat(dfs)
        df["step"] = load_steps
        return df
    else:
        return dfs[0]


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: TrainingConfig):
    df = evaluate_to_stats_df(config)

    assert config.group_name is not None
    stats_csv_folder = os.path.join("csv", "evaluation", "stats", config.group_name)
    if os.path.exists(stats_csv_folder):
        print(f"Stats csv folder {stats_csv_folder} already exists! Overwrite? (y/[n])")
        if False:
            raise ValueError("Stats csv folder already exists!")
        else:
            shutil.rmtree(stats_csv_folder)

    os.makedirs(stats_csv_folder, exist_ok=False)
    df.to_csv(os.path.join(stats_csv_folder, "stats.csv"), index=False)


if __name__ == "__main__":
    main()
