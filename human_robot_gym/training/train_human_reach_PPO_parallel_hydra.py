import hydra
from omegaconf import OmegaConf

from human_robot_gym.utils.config_utils import Config
from human_robot_gym.utils.training_utils import train


@hydra.main(base_version=None, config_path="config", config_name="human_reach_ppo_parallel")
def main(config: Config):
    if config.training.verbose:
        print(OmegaConf.to_yaml(config))

    train(config)


if __name__ == "__main__":
    main()
