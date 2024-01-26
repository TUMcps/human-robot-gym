"""This file contains a script to generate datasets with expert demonstrations.

We use hydra config files to configure the data collection. The config files are located in the
`human_robot_gym/training/config` folder. To specify the config file to use for data collection,
use the `--config-name` flag:

```
python human_robot_gym/training/create_expert_dataset.py --config-name pick_place_data_collection
```

For more information about hydra, see https://hydra.cc/docs/intro/
or refer to the human-robot-gym documentation: https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/training.html

Statistics of the dataset are stored in a csv file in the dataset folder. The csv file contains the following
statistics:
- Success rate: The percentage of episodes in which the goal was reached.
- Episode length: The mean, standard deviation, and confidence intervals of the episode lengths.
- Episode return: The mean, standard deviation, and confidence intervals of the episode returns.
- Info keys: The mean, standard deviation, and confidence intervals of the values
    of the logged info keys. These are specified in `config.run.log_info_keys`.

Author:
    Felix Trost (FT)

Changelog:
    15.06.23 FT File creation
    06.10.23 FT Store statistics in csv file
"""
from typing import Any, Dict, Optional, List, Tuple
import os
import threading

import numpy as np
import pandas as pd
from scipy.stats import bootstrap

import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
from human_robot_gym.utils.config_utils import DataCollectionConfig
from human_robot_gym.utils.training_utils import create_data_collection_environment, create_expert
from human_robot_gym.utils.mjcf_utils import file_path_completion


def extract_stats(
    episode_lengths: np.ndarray,
    episode_returns: np.ndarray,
    n_successes: int,
    episode_infos: Dict[str, List[Any]],
    bootstrap_resamples: int = 10000,
) -> pd.DataFrame:
    """Create a pandas dataframe containing statistics of the dataset.

    Args:
        episode_lengths (np.ndarray): Array containing the length of each episode.
        episode_returns (np.ndarray): Array containing the return of each episode.
        n_successes (int): Number of episodes in which the goal was reached.
        episode_infos (Dict[str, List[Any]]): Dictionary containing the values of the logged info keys for each episode.
        bootstrap_resamples (int): Number of bootstrap resamples to use for computing the confidence intervals.

    Returns:
        pd.DataFrame: Dataframe containing the statistics.
    """
    n_episodes = len(episode_lengths)

    success_rate_confidence_interval = bootstrap(
        data=np.array([1 for _ in range(n_successes)] + [0 for _ in range(n_episodes - n_successes)])[np.newaxis, :],
        n_resamples=bootstrap_resamples,
        confidence_level=0.95,
        statistic=np.mean,
    ).confidence_interval

    ep_len_confidence_interval = bootstrap(
        data=episode_lengths[np.newaxis, :],
        n_resamples=bootstrap_resamples,
        confidence_level=0.95,
        statistic=np.mean,
    ).confidence_interval

    ep_rew_confidence_interval = bootstrap(
        data=episode_returns[np.newaxis, :],
        n_resamples=bootstrap_resamples,
        confidence_level=0.95,
        statistic=np.mean,
    ).confidence_interval

    stats = pd.DataFrame(
        {
            "success_mean": [n_successes / n_episodes],
            "success_025": [success_rate_confidence_interval[0]],
            "success_975": [success_rate_confidence_interval[1]],

            "ep_len_mean": [np.mean(episode_lengths)],
            "ep_len_std": [np.std(episode_lengths)],
            "ep_len_025": [ep_len_confidence_interval[0]],
            "ep_len_975": [ep_len_confidence_interval[1]],

            "ep_rew_mean": [np.mean(episode_returns)],
            "ep_rew_std": [np.std(episode_returns)],
            "ep_rew_025": [ep_rew_confidence_interval[0]],
            "ep_rew_975": [ep_rew_confidence_interval[1]],
        }
    )

    for key, value in episode_infos.items():
        stats[f"{key}_mean"] = [np.mean(value)]
        stats[f"{key}_std"] = [np.std(value)]
        info_key_confidence_interval = bootstrap(
            data=np.array(value)[np.newaxis, :],
            n_resamples=bootstrap_resamples,
            confidence_level=0.95,
            statistic=np.mean,
        ).confidence_interval
        stats[f"{key}_025"] = [info_key_confidence_interval[0]]
        stats[f"{key}_975"] = [info_key_confidence_interval[1]]

    return stats


def print_stats(
    stats: pd.DataFrame,
):
    """Print the statistics of the dataset.

    Args:
        stats (pd.DataFrame): Dataframe containing the statistics.
    """
    stats_str = f"Success rate: {stats.success_mean.values[0]}%\n" \
                f"Episode length: {stats.ep_len_mean.values[0]} ({stats.ep_len_025[0]}, {stats.ep_len_975[0]})\n" \
                f"Episode return: {stats.ep_rew_mean.values[0]} ({stats.ep_rew_025[0]}, {stats.ep_rew_975[0]})\n" \

    print(stats_str)


def save_stats(
    stats: pd.DataFrame,
    dataset_name: str,
):
    """Save the statistics of the dataset to a csv file."""
    stats.to_csv(os.path.join(file_path_completion(f"../datasets/{dataset_name}"), "stats.csv"), index=False)


def save_obs_stats(
    observations: np.ndarray,
    dataset_name: str,
):
    # Account for the case when an observation value is constant across all observations
    # as that might provoke division-by-zero errors when performing observation normalization
    std = np.std(observations, axis=0)
    std[std == 0] = 1

    pd.DataFrame(
        {
            "mean": np.mean(observations, axis=0),
            "std": np.std(observations, axis=0),
        }
    ).to_csv(os.path.join(file_path_completion(f"../datasets/{dataset_name}"), "observations.csv"), index=False)


def collect_data(
    config: DataCollectionConfig,
    start_ep_idx: int = 0,
    end_ep_idx: Optional[int] = None,
    verbose: bool = True,
    save_stats_to_file: bool = True,
    save_observation_to_file: bool = True,
) -> Tuple[int, List[int], List[float], Dict[str, List[Any]], List[np.ndarray]]:
    """Collect data from the environment. The environment is seeded with `config.environment.seed`.

    Args:
        config (DataCollectionConfig): Data collection configuration
        start_ep_idx (int): The index of the first episode. Defaults to `0`.
        end_ep_idx (Optional[int]): The index of the last episode. Defaults to `None`.
            In this case, it is equal to `config.n_episodes + start_ep_idx`.
        verbose (bool): Whether to print out debug information about the data collection. Defaults to `True`.
        save_stats_to_file (bool): Whether to save the statistics of the dataset to a csv file. Defaults to `True`.
        save_observation_to_file (bool):
            Whether to save the observations of the dataset to a csv file. Defaults to `True`.

    Returns:
        Tuple[int, List[int], List[float], Dict[str, List[Any]], List[np.ndarray]]: Tuple containing:
            - The number of successful episodes
            - A list of the episode lengths
            - A list of the episode returns
            - A dictionary containing the values of the logged info keys for each episode
            - A list of the observations
    """
    if end_ep_idx is None:
        end_ep_idx = config.n_episodes + start_ep_idx

    env = create_data_collection_environment(config=config)
    expert = create_expert(config=config, env=env)
    expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env=env)

    n_successes = 0
    episode_lengths = []
    episode_returns = []
    episode_infos = {key: [] for key in config.run.log_info_keys}
    observations = []

    env.seed(config.environment.seed)

    for _ in range(start_ep_idx, end_ep_idx):
        obs = env.reset()
        observations.append(obs)

        done = False
        ret = 0
        i = 0
        while not done:
            i += 1
            obs, reward, done, info = env.step(expert(expert_obs_wrapper.current_expert_observation))
            observations.append(obs)
            ret += reward

        if info["n_goal_reached"] > 0:
            n_successes += 1

        for key in config.run.log_info_keys:
            episode_infos[key].append(info[key])
        episode_lengths.append(i)
        episode_returns.append(ret)

    env.close()

    stats = extract_stats(
        episode_lengths=np.array(episode_lengths),
        episode_returns=np.array(episode_returns),
        n_successes=n_successes,
        episode_infos=episode_infos,
    )

    if verbose:
        print_stats(stats=stats)

    if save_stats_to_file:
        save_stats(stats=stats, dataset_name=config.dataset_name)

    if save_observation_to_file:
        save_obs_stats(observations=np.array(observations), dataset_name=config.dataset_name)

    return n_successes, episode_lengths, episode_returns, episode_infos, observations


def collect_data_threaded(
    config: DataCollectionConfig,
    verbose: bool = True,
    save_stats_to_file: bool = True,
    save_observation_to_file: bool = True,
):
    """Collect data in parallel using multiple threads.

    Since robosuite uses the global np random number generator this method does not support seeding
    due to the fact that the random number generator is not thread safe. Sorry.

    Args:
        config (DataCollectionConfig): Data collection configuration
        verbose (bool): Whether to print out debug information about the data collection. Defaults to `True`.
        save_stats_to_file (bool): Whether to save the statistics of the dataset to a csv file. Defaults to `True`.
        save_observation_to_file (bool):
            Whether to save the observations of the dataset to a csv file. Defaults to `True`.
    """
    n_threads = config.n_threads if config.n_threads is not None else 1
    n_successes_total = [None for _ in range(n_threads)]
    ep_lengths_total = [None for _ in range(n_threads)]
    ep_returns_total = [None for _ in range(n_threads)]
    ep_infos_total = [None for _ in range(n_threads)]
    observations_total = [None for _ in range(n_threads)]

    class EpCollectionThread(threading.Thread):
        def __init__(self, thread_index: int, config: DataCollectionConfig, start_ep_idx: int, end_ep_idx: int):
            super().__init__()
            self._thread_index = thread_index
            self._config = config
            self._start_ep_idx = start_ep_idx
            self._end_ep_idx = end_ep_idx

        def run(self):
            (
                n_successes_in_thread,
                ep_lengths_in_thread,
                ep_returns_in_thread,
                ep_infos_in_thread,
                observations_in_thread,
            ) = collect_data(
                config=self._config,
                start_ep_idx=self._start_ep_idx,
                end_ep_idx=self._end_ep_idx,
                verbose=False,
                save_stats_to_file=False,
                save_observation_to_file=False,
            )

            n_successes_total[self._thread_index] = n_successes_in_thread
            ep_lengths_total[self._thread_index] = ep_lengths_in_thread
            ep_returns_total[self._thread_index] = ep_returns_in_thread
            ep_infos_total[self._thread_index] = ep_infos_in_thread
            observations_total[self._thread_index] = observations_in_thread

    threads = [None for _ in range(n_threads)]
    for i in range(n_threads):
        threads[i] = EpCollectionThread(
            i,
            config,
            round(config.n_episodes * i / n_threads),
            round(config.n_episodes * (i + 1) / n_threads)
        )

        threads[i].start()

    for t in threads:
        t.join()

    stats = extract_stats(
        episode_lengths=np.concatenate(ep_lengths_total),
        episode_returns=np.concatenate(ep_returns_total),
        n_successes=sum(n_successes_total),
        episode_infos={
            key: np.concatenate([info[key] for info in ep_infos_total]) for key in config.run.log_info_keys
        },
    )

    if verbose:
        print_stats(stats=stats)

    if save_stats_to_file:
        save_stats(stats=stats, dataset_name=config.dataset_name)

    if save_observation_to_file:
        save_obs_stats(observations=np.concatenate(observations_total), dataset_name=config.dataset_name)


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: DataCollectionConfig):
    if config.run.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    directory = file_path_completion(f"../datasets/{config.dataset_name}")
    if os.path.exists(directory):
        print(f"Dataset path '{directory}' already exists, would you like to overwrite it? ([y]/n)")
        if input() == "n":
            raise ValueError("Dataset path already exists, please change the directory name.")
        os.system(f"rm -r {directory}")

    os.makedirs(directory)

    with open(os.path.join(directory, "config.yaml"), "w") as f:
        f.write(OmegaConf.to_yaml(cfg=config, resolve=True))

    if config.n_threads == 1:
        collect_data(
            config=config,
            verbose=config.run.verbose,
            save_stats_to_file=True,
            save_observation_to_file=True,
        )
    else:
        collect_data_threaded(
            config=config,
            verbose=config.run.verbose,
            save_stats_to_file=True,
            save_observation_to_file=True,
        )

    print("Done.")


if __name__ == '__main__':
    main()
