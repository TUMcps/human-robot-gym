import hydra
from omegaconf import OmegaConf
from human_robot_gym.utils.config_utils import DataAugmentationConfig
from human_robot_gym.augmentation_pipeline.trans_rot_aug import TranslationXYRotationAug
from human_robot_gym.utils.mjcf_utils import xml_path_completion
import sys
import os
from human_robot_gym.utils.logging_utils import setup_logger
from human_robot_gym.utils.augmentation_utils import set_random_seed

@hydra.main(version_base=None, config_path="config", config_name=None)
def main(config: DataAugmentationConfig):
    
    set_random_seed(config['seed'])

    # setup the logger to help with message debugging etc.
    logger = setup_logger("AugmentationLogger", 'debug.log') 

    # get the absolute path of the folder where
    #  all the demos are located - this folder 
    # contains folders (which are essentialy a task descriptor, and inside of those, 
    # the actual animation data is there)
    human_animation_parent_path = xml_path_completion('human/animations/human-robot-animations/')
    
    # get the absolute path of the folder where we would 
    # like to save the augmentations (whatever format they may be - 
    # for some we may just save quaternion jsons, for some 
    # we may need to make an entire new dict, i.e. pickle file)
    aug_parent_folder = os.path.join(xml_path_completion(f'human/animations/human-robot-animations/augmented_data/{config["aug_output_folder_name"]}'))
    
    # full filepath of metadata file
    aug_md_filepath = os.path.join(aug_parent_folder, config['aug_metadata_file_name'] + '.json')

    if not os.path.exists(aug_parent_folder):
        logger.warning(f'{aug_parent_folder} does not exist, hence making it!')
        os.makedirs(aug_parent_folder)


    transrot  = TranslationXYRotationAug(translation_ranges = config['translation_ranges'], rotation_ranges = config['rotation_ranges'], aug_factor=config['aug_factor'], translation_aug = config['translation_aug'], rotation_aug = config['rotation_aug'], human_animation_names = config['human_animation_names'],
    human_animation_parent_path = human_animation_parent_path,
    augmentations_output_folder = aug_parent_folder, aug_md_filepath=aug_md_filepath)
    
    transrot.generate_demos()

if __name__ == '__main__':
    main() 