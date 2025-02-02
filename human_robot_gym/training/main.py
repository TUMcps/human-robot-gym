import hydra
from human_robot_gym.utils.config_utils import DataAugmentationConfig

@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: DataAugmentationConfig):
    pass