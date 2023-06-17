"""This file contains a script to generate datasets with expert demonstrations.

We use hydra config files to configure the data collection. The config files are located in the
`human_robot_gym/training/config` folder. To specify the config file to use for data collection,
use the `--config-name` flag:

```
python human_robot_gym/training/create_expert_dataset.py --config-name pick_place_data_collection
```

For more information about hydra, see https://hydra.cc/docs/intro/
or refer to the human-robot-gym documentation: https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/training.html

Author:
    Felix Trost (FT)

Changelog:
    15.06.23 FT File creation
"""

from typing import Optional, Tuple
import os
import threading

import numpy as np

import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
from human_robot_gym.utils.config_utils import DataCollectionConfig
from human_robot_gym.utils.training_utils import create_data_collection_environment, create_expert
from human_robot_gym.utils.mjcf_utils import file_path_completion


def collect_data(
    config: DataCollectionConfig,
    start_ep_idx: int = 0,
    end_ep_idx: Optional[int] = None,
    verbose: bool = False,
) -> Tuple[int, int]:
    """Collect data from the environment. The environment is seeded with config.environment.seed.

    Args:
        config (DataCollectionConfig): Data collection configuration
        start_ep_idx (int): The index of the first episode. Defaults to 0.
        end_ep_idx (Optional[int]): The index of the last episode. Defaults to None.
            In this case, it is equal to `config.n_episodes + start_ep_idx`.
        verbose (bool): Whether to print out debug information about the data collection. Defaults to False.
    """
    if end_ep_idx is None:
        end_ep_idx = config.n_episodes + start_ep_idx

    env = create_data_collection_environment(config=config)
    expert = create_expert(config=config, env=env)
    expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env=env)

    n_successes = 0
    episode_lengths = []

    env.seed(config.environment.seed)

    for _ in range(start_ep_idx, end_ep_idx):
        env.reset()
        done = False
        i = 0
        while not done:
            i += 1
            _, _, done, info = env.step(expert(expert_obs_wrapper.current_expert_observation))

        if info["n_goal_reached"] > 0:
            n_successes += 1

        episode_lengths.append(i)

    if verbose:
        print(f"Success rate: {n_successes / config.n_episodes * 100}%")

        print(f"Average episode length: {np.mean(episode_lengths)}")
        print(f"Episode length std: {np.std(episode_lengths)}")

    env.close()

    return n_successes, episode_lengths


def collect_data_threaded(config: DataCollectionConfig, verbose: bool = False):
    """Collect data in parallel using multiple threads.

    Since robosuite uses the global np random number generator this method does not support seeding
    due to the fact that the random number generator is not thread safe. Sorry.

    Args:
        config (DataCollectionConfig): Data collection configuration
    """
    n_threads = config.n_threads if config.n_threads is not None else 1
    n_successes_total = [None for _ in range(n_threads)]
    ep_lengths_total = [None for _ in range(n_threads)]

    class EpCollectionThread(threading.Thread):
        def __init__(self, thread_index: int, config: DataCollectionConfig, start_ep_idx: int, end_ep_idx: int):
            super().__init__()
            self._thread_index = thread_index
            self._config = config
            self._start_ep_idx = start_ep_idx
            self._end_ep_idx = end_ep_idx

        def run(self):
            n_successes_in_thread, ep_lengths_in_thread = collect_data(
                config=self._config,
                start_ep_idx=self._start_ep_idx,
                end_ep_idx=self._end_ep_idx,
                verbose=False,
            )

            n_successes_total[self._thread_index] = n_successes_in_thread
            ep_lengths_total[self._thread_index] = ep_lengths_in_thread

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

    if verbose:
        print(f"Success rate: {sum(n_successes_total) / config.n_episodes * 100}%")

        ep_lengths_total = sum(ep_lengths_total, [])
        print(f"Average episode length: {np.mean(ep_lengths_total)}")
        print(f"Episode length std: {np.std(ep_lengths_total)}")


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: DataCollectionConfig):
    if config.training.verbose:
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
        collect_data(config=config, verbose=config.training.verbose)
    else:
        collect_data_threaded(config=config, verbose=config.training.verbose)

    print("Done.")


if __name__ == '__main__':
    main()
