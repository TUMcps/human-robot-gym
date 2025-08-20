"""This file implements an evaluation script for evaluating stable-baselines3 agents and experts
in human-robot-gym environments.

We use hydra configuration files. These are located in the `human_robot_gym/training/config` folder.
To specify the config file to use for evaluation, use the `--config-name` flag:

```
python human_robot_gym/training/evaluate_models_to_csv.py --config-name human_reach_ppo_parallel
```

For more information about hydra, see https://hydra.cc/docs/intro/
or refer to the human-robot-gym documentation: https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/training.html

This script can be used to evaluate multiple models at different load steps in parallel.
A common use case is to evaluate multiple models trained with different seeds and calculate statistics over all.
To do this, pass a list of strings to `config.run.id`:

```
python human_robot_gym/training/evaluate_models_to_csv.py --config-name human_reach_ppo_parallel \
    config.run.id=[run_id_0,run_id_1,run_id_2]
```

If `config.run.id` is `None`, the expert policy specified in the config is evaluated
instead of stable-baselines3 agents.

The `config.run.load_step` parameter can be used to specify which snapshots of the agent to evaluate.
The models should be saved in the form of `{run_id}/model_{load_step:_}.zip` in the `models` folder. For example, if
`config.run.load_step` is set to 100000, and `config.run.id` to `abcxyz`,
the model at `models/abcxyz/model_100_000.zip` is evaluated.
Please note that the run id may also contain slashes (e.g. `abc/xyz`), in which case the model is loaded from
`models/abc/xyz/model_100_000.zip`.

If `config.run.load_step` is set to `"all"`, models from load step 0 to `config.run.n_steps` are evaluated,
with a step size of `config.run.save_freq`.
For example, if `config.run.n_steps` is 1e6 and `config.run.save_freq` is 1e5, models at load steps
0, 1e5, 2e5, ..., 1e6 are evaluated.

The raw evaluation results are saved to a csv file in the `csv/evaluation/raw` folder.
Specifying a value for `config.group_name` is required, as statistics are saved at
`csv/evaluation/stats/{config.group_name}/stats.csv`.

Author:
    Felix Trost (FT)

Changelog:
    06.10.23 FT Added docstrings
"""
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
from human_robot_gym.utils.training_utils import create_expert, create_wrapped_env_from_config
from human_robot_gym.utils.training_utils_SB3 import load_model
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
import ray


def evaluate_to_df(
    config: TrainingConfig,
    evaluate_expert: bool,
    run_id: Optional[str] = None,
    load_step: Optional[Union[int, str]] = None,
) -> pd.DataFrame:
    """Evaluate an agent on an evaluation environment and return a dataframe with the results.

    If `evaluate_expert` is `True`, the expert policy specified in the config is evaluated. Otherwise, a model
    is loaded from disk.

    Args:
        config (TrainingConfig): The config to use for evaluation.
        evaluate_expert (bool): Whether to evaluate the expert policy. Defaults to `False`.
        run_id (Optional[str]): The run id of the model to evaluate. If `None`, the model specified in
            `config.run.id` is evaluated. Only used if `evaluate_expert` is `False`. Defaults to `None`.
        load_step (Optional[Union[int, str]], optional): The load step of the model to evaluate.
            If `None`, the model specified in `config.run.load_step` is evaluated.
            Only used if `evaluate_expert` is `False`. Defaults to `None`.

    Returns:
        pd.DataFrame: A dataframe with the evaluation results.
    """
    env = create_wrapped_env_from_config(config=config, evaluation_mode=True)

    if run_id is None:
        run_id = config.run.id
    if load_step is None:
        load_step = config.run.load_step

    if evaluate_expert:
        model = create_expert(config=config, env=env)
        expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env=env)
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

    # If model and env have different observation spaces, assume it's because of the time value
    # added to the observation space in the SIR wrapper
    different_obs = model.observation_space.shape != env.observation_space.shape
    if different_obs:
        assert len(model.observation_space.shape) == len(env.observation_space.shape)
        assert model.observation_space.shape[0] == env.observation_space.shape[0] + 1

        env.observation_space = model.observation_space
        assert os.path.exists(
            csv_path := os.path.join("datasets", config.wrappers.dataset_obs_norm.dataset_name, "stats.csv")
        )

        mean_ep_len = pd.read_csv(csv_path).ep_len_mean[0]

    if not evaluate_expert:
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
                obs = np.append(obs, time_value)

            if evaluate_expert:
                action = np.array(model(expert_obs_wrapper.current_expert_observation))
            else:
                action, _ = model.predict(obs, deterministic=True)
            step_index += 1
            obs, reward, done, info = env.step(action)
            ep_return += reward
            ep_length += 1

        successes.append(1 if info["n_goal_reached"] > 0 else 0)
        for key in config.run.log_info_keys:
            ep_infos[key].append(info[key])

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


def get_eval_raw_data_folder_path(config: TrainingConfig) -> str:
    """Return the relative path to the folder in which the raw evaluation csv files should be saved.

    Args:
        config (TrainingConfig): the config specifying the subfolder for the raw csv files.

    Returns:
        str: the path to the raw csv folder
    """
    return os.path.join("csv", "evaluation", config.group_name, "raw")


def get_eval_raw_data_csv_file_path(
    config: TrainingConfig,
    evaluate_expert: bool,
    run_id: Optional[str],
    load_step: Optional[Union[int, str]],
) -> str:
    """Return the path to the file where the raw evaluation csv files should be saved according to the config.

    Args:
        config (TrainingConfig): the config specifying the save location of the raw csv files
        evaluate_expert (bool): whether the expert or a trained agent is evaluated
        run_id (str | None): enables overriding the run id specified in `config.run.id`
        load_step (str | int | None): enables overriding `config.run.load_step`

    Returns:
        str: the path to the raw csv file
    """
    raw_csv_folder_path = get_eval_raw_data_folder_path(config=config)
    if evaluate_expert:
        return os.path.join(raw_csv_folder_path, f"expert_{config.expert.id}.csv")
    else:
        if run_id is None:
            run_id = config.run.id
        if load_step is None:
            load_step = config.run.load_step

        if isinstance(load_step, int):
            return os.path.join(raw_csv_folder_path, f"{run_id}_{load_step:_}.csv")
        else:
            return os.path.join(raw_csv_folder_path, f"{run_id}_{load_step}.csv")


def get_eval_stats_data_folder_path(config: TrainingConfig) -> str:
    """Return the relative path to the folder in which the evaluation statistics csv files should be saved.

    Args:
        config (TrainingConfig): the config specifying the subfolder for the stats csv files.

    Returns:
        str: the path to the raw csv folder
    """
    return os.path.join("csv", "evaluation", config.group_name)


def eval_to_csv(
    config: TrainingConfig,
    evaluate_expert: bool,
    run_id: Optional[str] = None,
    load_step: Optional[Union[int, str]] = None,
) -> str:
    """Evaluate an agent on an evaluation environment and save the results to a csv file.

    If `evaluate_expert` is `True`, the expert policy specified in the config is evaluated. Otherwise, a model
    is loaded from disk.

    Args:
        config (TrainingConfig): The config to use for evaluation.
        evaluate_expert (bool): Whether to evaluate the expert policy. Defaults to `False`.
        run_id (Optional[str]): The run id of the model to evaluate. If `None`, the model specified in
            `config.run.id` is evaluated. Only used if `evaluate_expert` is `False`. Defaults to `None`.
        load_step (Optional[Union[int, str]], optional): The load step of the model to evaluate.
            If `None`, the model specified in `config.run.load_step` is evaluated.
            Only used if `evaluate_expert` is `False`. Defaults to `None`.

    Returns:
        str: The path to the csv file containing the evaluation results.
    """
    df = evaluate_to_df(config=config, evaluate_expert=evaluate_expert, run_id=run_id, load_step=load_step)

    csv_file_path = get_eval_raw_data_csv_file_path(
        config=config,
        evaluate_expert=evaluate_expert,
        run_id=run_id,
        load_step=load_step,
    )

    os.makedirs(os.path.dirname(csv_file_path), exist_ok=True)
    df.to_csv(csv_file_path, index=False)
    return csv_file_path


@ray.remote
def eval_to_csv_ray(
    config: TrainingConfig,
    evaluate_expert: bool,
    run_id: Optional[str] = None,
    load_step: Optional[Union[int, str]] = None,
) -> str:
    """Evaluate an agent on an evaluation environment and save the results to a csv file.

    If `evaluate_expert` is `True`, the expert policy specified in the config is evaluated. Otherwise, a model
    is loaded from disk.

    This function is a wrapper around `eval_to_csv` that can be parallelized with Ray.

    Args:
        config (TrainingConfig): The config to use for evaluation.
        evaluate_expert (bool): Whether to evaluate the expert policy. Defaults to `False`.
        run_id (Optional[str]): The run id of the model to evaluate. If `None`, the model specified in
            `config.run.id` is evaluated. Only used if `evaluate_expert` is `False`. Defaults to `None`.
        load_step (Optional[Union[int, str]], optional): The load step of the model to evaluate.
            If `None`, the model specified in `config.run.load_step` is evaluated.
            Only used if `evaluate_expert` is `False`. Defaults to `None`.

    Returns:
        str: The path to the csv file containing the evaluation results.
    """
    return eval_to_csv(config=config, evaluate_expert=evaluate_expert, run_id=run_id, load_step=load_step)


def combine_to_stats_df(
    config: TrainingConfig,
    csv_paths: List[str],
    replace_nan: bool = True,
    bootstrap_resamples: int = 10000,
) -> pd.DataFrame:
    """Combine the evaluation results from a list of csv files into a single dataframe containing statistics.

    Statistics obtained:
        - mean
        - std
        - 0.95 confidence intervals (bootstrap)
        - median
        - lower quartile
        - upper quartile
        - lower whisker
        - upper whisker

    Args:
        config (TrainingConfig): The config used for evaluation.
        csv_paths (List[str]): A list of csv files containing the evaluation results.
        replace_nan (bool, optional): Whether to replace NaN values for the confidence bounds
            with the mean of the column.
        bootstrap_resamples (int): The number of bootstrap resamples to use for the confidence bounds.

    Returns:
        pd.DataFrame: A dataframe containing statistics of the evaluation results.
    """
    df = pd.concat([pd.read_csv(csv_path) for csv_path in csv_paths])

    keys = ["ep_rew", "ep_len", "success"] + config.run.log_info_keys

    stats_df = pd.DataFrame()

    for key in keys:
        info_df = df.describe()[key]
        stats_df[f"{key}_mean"] = [info_df["mean"]]
        stats_df[f"{key}_std"] = [info_df["std"]]
        conf = bootstrap(
            df[key].values[np.newaxis, :],
            n_resamples=bootstrap_resamples,
            confidence_level=0.95,
            statistic=np.mean,
        ).confidence_interval
        stats_df[f"{key}_025"] = [stats_df[f"{key}_mean"].values[0] if np.isnan(conf[0]) and replace_nan else conf[0]]
        stats_df[f"{key}_975"] = [stats_df[f"{key}_mean"].values[0] if np.isnan(conf[1]) and replace_nan else conf[1]]

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


def get_evaluation_run_ids(config: TrainingConfig) -> Optional[List[str]]:
    """Get the run ids to evaluate."""
    run_ids = config.run.id
    if hasattr(run_ids, "__iter__"):
        return [str(run_id) for run_id in run_ids]
    elif run_ids is None:
        return None
    else:
        return [str(run_ids)]


def get_evaluation_load_steps(config: TrainingConfig) -> List[Union[str, int]]:
    """Get the steps at which model snapshots should be evaluated."""
    load_steps = config.run.load_step
    if load_steps == "all":
        return [
            load_step + config.run.save_freq
            for load_step in range(0, config.run.n_steps, config.run.save_freq)
        ]
    else:
        return [load_steps]


def evaluate_to_stats_df(
    config: TrainingConfig,
    max_parallel_runs: Optional[int] = None,
) -> pd.DataFrame:
    """Evaluate an agent on an evaluation environment and return the statistics inside a dataframe.

    Can be used to evaluate multiple models at different load steps in parallel.
    if `config.run.id` is a list of strings, all models are evaluated and statistics are calculated over all.
    if `config.run.id` is `None`, the expert policy specified in the config is evaluated.

    if `config.run.load_step` is set to `"all"`, models from load step 0 to `config.run.n_steps` are evaluated,
    with a step size of `config.run.save_freq`.
    For example, if `config.run.n_steps` is 1e6 and `config.run.save_freq` is 1e5, models at load steps
    0, 1e5, 2e5, ..., 1e6 are evaluated.

    Statistics obtained:
        - mean
        - std
        - 0.95 confidence intervals (bootstrap)
        - median
        - lower quartile
        - upper quartile
        - lower whisker
        - upper whisker
    of the following metrics:
        - ep_rew
        - ep_len
        - success
        - all metrics specified in `config.run.log_info_keys`

    Args:
        config (TrainingConfig): The config to use for evaluation.
        max_parallel_runs (Optional[int], optional): The maximum number of parallel runs to use for evaluation.
            If `None`, the number of parallel runs is set to the number of run ids times the number of load steps
            (i.e. all runs are executed in parallel). Defaults to `None`.

    Returns:
        pd.DataFrame: A dataframe containing statistics of the evaluation results.
    """
    if config.run.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    # If a list of strings is passed to config.run.id, evaluate all and calculate statistics over all
    run_ids = get_evaluation_run_ids(config=config)

    # If run_ids is None, evaluate the expert policy
    evaluate_expert = run_ids is None

    load_steps = get_evaluation_load_steps(config=config)

    if config.run.verbose:
        print(f"Evaluating models {run_ids} at steps {load_steps}")

    # List of lists: shape (len(load_steps), len(run_ids))
    csv_paths = []
    if evaluate_expert:
        if config.run.verbose:
            print(f"Evaluating expert {config.expert.id}")
        csv_paths.append([eval_to_csv(config=config, evaluate_expert=True)])
    else:
        if max_parallel_runs is None:
            max_parallel_runs = len(run_ids) * len(load_steps)
        max_parallel_runs = min(max_parallel_runs, len(run_ids) * len(load_steps))
        assert max_parallel_runs > 0

        ray.init(num_cpus=max_parallel_runs)
        print(f"Evaluating models with up to {max_parallel_runs} parallel runs")

        results = []
        results = [
            [
                eval_to_csv_ray.remote(
                    config=config,
                    evaluate_expert=False,
                    run_id=run_id,
                    load_step=load_step,
                )
                for run_id in run_ids
            ]
            for load_step in load_steps
        ]

        for result in results:
            csv_paths.append(ray.get(result))

    print("Got data, now obtaining stats")

    dfs = [combine_to_stats_df(config=config, csv_paths=paths) for paths in csv_paths]
    if len(dfs) > 1:  # Multiple load steps
        df = pd.concat(
            [
                pd.DataFrame({"step": load_steps}),
                pd.concat(dfs, ignore_index=True),
            ],
            axis=1,
        )
        return df
    else:
        df = pd.concat(
            [
                pd.DataFrame({"step": [load_steps[0]]}),
                dfs[0],
            ],
            axis=1,
        )
        return df


def evaluate_to_csv(config: TrainingConfig, max_parallel_runs: Optional[int] = None) -> str:
    """Evaluate an agent on an evaluation environment and save statistics to a csv file.

    Can be used to evaluate multiple models at different load steps in parallel.
    if `config.run.id` is a list of strings, all models are evaluated and statistics are calculated over all.
    if `config.run.id` is `None`, the expert policy specified in the config is evaluated.

    if `config.run.load_step` is set to `"all"`, models from load step 0 to `config.run.n_steps` are evaluated,
    with a step size of `config.run.save_freq`.
    For example, if `config.run.n_steps` is 1e6 and `config.run.save_freq` is 1e5, models at load steps
    0, 1e5, 2e5, ..., 1e6 are evaluated.

    Statistics obtained:
        - mean
        - std
        - 0.95 confidence intervals (bootstrap)
        - median
        - lower quartile
        - upper quartile
        - lower whisker
        - upper whisker
    of the following metrics:
        - ep_rew
        - ep_len
        - success
        - all metrics specified in `config.run.log_info_keys`

    Args:
        config (TrainingConfig): The config to use for evaluation.
        max_parallel_runs (Optional[int], optional): The maximum number of parallel runs to use for evaluation.
            If `None`, the number of parallel runs is set to the number of run ids times the number of load steps
            (i.e. all runs are executed in parallel). Defaults to `None`.

    Returns:
        str: The path to the csv file containing the evaluation results.
    """
    assert hasattr(config, "group_name") and config.group_name is not None
    df = evaluate_to_stats_df(config=config, max_parallel_runs=max_parallel_runs)

    stats_csv_folder = get_eval_stats_data_folder_path(config=config)
    stats_csv_file_path = os.path.join(stats_csv_folder, "stats.csv")
    if os.path.exists(stats_csv_folder):
        print(f"Stats csv folder {stats_csv_folder} already exists! Overwriting...")
        shutil.rmtree(stats_csv_folder)

    os.makedirs(stats_csv_folder, exist_ok=False)
    df.to_csv(stats_csv_file_path, index=False)

    return stats_csv_file_path


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: TrainingConfig):
    if config.run.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    max_parallel_runs = 50  # By default, evaluate 50 models in parallel
    if hasattr(config, "max_parallel_runs") and config.max_parallel_runs is not None:
        max_parallel_runs = config.max_parallel_runs

    evaluate_to_csv(config, max_parallel_runs=max_parallel_runs)


if __name__ == "__main__":
    main()
