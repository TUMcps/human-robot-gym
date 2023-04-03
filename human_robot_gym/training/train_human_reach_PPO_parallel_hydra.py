import hydra
from omegaconf import OmegaConf

import robosuite  # noqa: F401

from human_robot_gym.utils.config_utils import Config
from human_robot_gym.utils.training_utils import train
import human_robot_gym.robots  # noqa: F401


@hydra.main(version_base=None, config_path="config", config_name="human_reach_ppo_parallel")
def main(config: Config):
    if config.training.verbose:
        print(OmegaConf.to_yaml(config))

    train(config)


if __name__ == "__main__":
    main()
