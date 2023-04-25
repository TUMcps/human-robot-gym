"""This file implements a training script for training stable-baselines3 agents in human-robot-gym environments.

We use hydra to configure the training. The config files are located in the `human_robot_gym/training/config` folder.
To specify the config file to use for training, use the `--config-name` flag:

```
python human_robot_gym/training/train_SB3.py --config-name human_reach_ppo_parallel
```

For more information about hydra, see https://hydra.cc/docs/intro/
or refer to the human-robot-gym documentation: https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/training.html

Author:
    Felix Trost (FT)

Changelog:
    25.04.23 FT File creation
"""
import torch
import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

from human_robot_gym.utils.config_utils import Config
from human_robot_gym.utils.training_utils import train_and_evaluate
import human_robot_gym.robots  # noqa: F401


@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: Config):
    if config.training.verbose:
        print(OmegaConf.to_yaml(cfg=config, resolve=True))

    train_and_evaluate(config)


if __name__ == "__main__":
    torch.set_num_threads(1)

    main()
