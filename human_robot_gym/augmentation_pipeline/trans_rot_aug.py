from typing import List, Any, Union
import os 
import random
import json

class TranslationXYRotationAug:
    def __init__(
        self, translation_ranges: List[List[float]] = None, rotation_range: List = None, num_trans_aug: int = None, num_rot_aug: int = None, human_demo_names: List[str] = None, human_demo_parent_path: str = os.path.join("/home/mb230/projects/human-robot-gym/human_robot_gym/models/assets/human/animations/human-robot-animations"),
        augmentations_output_folder: str = "./aug_human_demo/"
    ):
         # using an absolute for the human demo path now, but later depending on which folder the file is run from, this can be changed to a relative path, but leaving it as an absolute path for now
        """The init function
        Args:
            translation_ranges (List[List[float]], optional): Range in specific units to translate the base of the human. Defaults to None.
            rotation_range (List, optional): The angle (in some specific units) to rotate the human. Defaults to None.
        """
        self.num_trans_aug = num_trans_aug
        self.num_rot_aug = num_rot_aug
        assert len(translation_ranges) == 2, "We are using only augmentations along the X and Y plane so the size of the list must be 2!"
        self.x_trans_range = translation_ranges[0]
        self.y_trans_range = translation_ranges[1]
        self.rotation_range = rotation_range

        self.augmentations_output_folder = augmentations_output_folder
        # make augmentations output path, if it does not exist
        if not os.path.exists(self.augmentations_output_folder):
            os.makedirs(self.augmentations_output_folder, exist_ok=True)
        
        # get the paths of the human demos from the parent human path and the demo names - the human joints are controlled by the 
        self.demo_paths = {"infos": [], "motion_pkl": []}
        self.human_demo_names = human_demo_names

        for demo_name in self.human_demo_names:
            self.demo_paths["infos"].append(os.path.join(human_demo_parent_path, demo_name + "_info.json"))
            self.demo_paths['motion_pkl'].append(os.path.join(human_demo_parent_path, demo_name + ".pkl"))
        
        self.num_demos = len(human_demo_names)
        print(f"Number of original human demos is {self.num_demos}")

    def generate_demos(self):
        """Function to perform data augmentation on the existing human demos
        """
        aug_count = 0

        # TODO - this loop needs to be figured out, how we want to apply the rotation and translation augmentation
        for _ in range(self.num_rot_aug):
            for _ in range(self.num_trans_aug):
                # choose a random demonstration to perform augmentation - 
                demo_idx = random.randint(0, self.num_demos - 1)
                demo_json = self.demo_paths['infos'][demo_idx]
                demo_name = self.human_demo_names[demo_idx]

                with open(demo_json, "r") as file:
                    json_data = json.load(file)

                sample_x_trans = random.uniform(*self.x_trans_range)
                sample_y_trans = random.uniform(*self.y_trans_range)
                new_json = json_data.copy()

                # add the augmentation offsets to the original offsets
                new_json["position_offset"][0] = json_data["position_offset"][0] + sample_x_trans
                new_json["position_offset"][1] = json_data["position_offset"][1] + sample_y_trans

                # TODO - change orientation of the human by modifying the orientation quaternion - have not done this yet

                new_json_path = os.path.join(self.augmentations_output_folder, demo_name + f"_info_aug{aug_count}.json")

                # make json dir - this is solely needed to handle the way the HR Gym work takes in demo names, it takes in one-level above folder and the original filename
                json_dir_to_make = os.path.dirname(new_json_path)
                if not os.path.exists(json_dir_to_make):
                    os.makedirs(json_dir_to_make, exist_ok=True)

                with open(new_json_path, "w") as file:
                    json.dump(new_json, file, indent=4)
                
                # copy the original pickle data in addition to saving the json data
                aug_count = aug_count + 1



# code below to test the class        
if __name__ == '__main__':
    human_animation_names= ["CMU/62_01"]
    x = TranslationXYRotationAug(translation_ranges = [[10,10],[10,10]], num_trans_aug = 3, num_rot_aug = 1, human_demo_names = human_animation_names)
    x.generate_demos()

