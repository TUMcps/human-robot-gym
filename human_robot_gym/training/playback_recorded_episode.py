"""This file contains a script to playback episodes from datasets collected with the `DatasetCollectionWrapper`.

We use hydra config files. The config files are located in the `human_robot_gym/training/config` folder.

To specify the config file to use for playback, use the `--config-name` flag:

```
python human_robot_gym/training/playback_recorded_episode.py --config-name pick_place_data_playback
```

For more information about hydra, see https://hydra.cc/docs/intro/
or refer to the human-robot-gym documentation: https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/training.html

Author:
    Felix Trost (FT)

Changelog:
    15.06.23 FT File creation
"""
import os

import numpy as np

import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

import human_robot_gym.robots  # noqa: F401
from human_robot_gym.utils.training_utils import create_wrapped_env_from_config
from human_robot_gym.utils.mjcf_utils import file_path_completion
from human_robot_gym.utils.config_utils import DataCollectionConfig


def load_ep_data(config: DataCollectionConfig):
    """Load the xml file and the state dictionary of an episode.

    Args:
        config (DataCollectionConfig): Data collection config
    """
    load_episode = config.load_episode_index

    if load_episode is None:
        print("Select which episode to load:")
        load_episode = int(input())

    episode_dir = "%06i" % load_episode
    dataset_path = file_path_completion(f"../datasets/{config.dataset_name}/ep_{episode_dir}")
    xml_path = os.path.join(dataset_path, "model.xml")
    states_path = os.path.join(dataset_path, "state.npz")

    xml_file = None

    with open(xml_path, "r") as f:
        xml_file = f.read()

    dic = np.load(states_path, allow_pickle=True)

    return xml_file, dic


def playback_trajectory(config: DataCollectionConfig):
    """Playback a trajectory from a dataset.

    Loops the episode by playing it forwards and backwards.

    Args:
        config (DataCollectionConfig): Data collection config
    """
    env = create_wrapped_env_from_config(config)

    xml_file, dic = load_ep_data(config)
    env.reset()
    env.unwrapped.reset_from_xml_string(xml_file)

    while True:
        for state in list(dic["states"]) + list(dic["states"][::-1]):
            env.unwrapped.set_environment_state(state)
            env.unwrapped.render()


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: DataCollectionConfig):
    if config.run.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    playback_trajectory(config=config)


if __name__ == '__main__':
    main()
