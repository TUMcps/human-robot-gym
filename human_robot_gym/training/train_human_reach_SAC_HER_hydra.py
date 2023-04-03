import hydra
from omegaconf import OmegaConf

from stable_baselines3 import HerReplayBuffer

from human_robot_gym.utils.config_utils import Config
from human_robot_gym.utils.training_utils import train
from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import custom_add, _custom_sample_transitions


@hydra.main(base_version=None, config_path="config", config_name="human_reach_sac_her")
def main(config: Config):
    if config.training.verbose:
        print(OmegaConf.to_yaml(config))

    HerReplayBuffer.add = custom_add
    HerReplayBuffer._sample_transitions = _custom_sample_transitions

    train(config)


if __name__ == "__main__":
    main()
