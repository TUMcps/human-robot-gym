from typing import List, Any, Union
import os 
import random
import json
import sys
import warnings


class TranslationXYRotationAug:
    def __init__(
        self, translation_ranges: Union[List[List[float]], None] = None, rotation_ranges: Union[List[List[float]], None] = None, aug_factor: Union[int, None] = 1 , 
        translation_aug: bool = None, rotation_aug: bool = None, human_animation_names: List[str] = None, human_animation_parent_path: str = os.path.join("/home/mb230/projects/human-robot-gym/human_robot_gym/models/assets/human/animations/human-robot-animations"),
        augmentations_output_folder: str = "./aug_human_demo/",
        aug_md_filepath: str = None, # filename of the metadata (explained in config)
        mode = 'offline',):
        # using an absolute for the human demo path now, but later depending on which folder the file is run from, this can be changed to a relative path, but leaving it as an absolute path for now
        """The init function
        Args:
            translation_ranges (List[List[float]], optional): Range in specific units to translate the base of the human. Defaults to None.
            rotation_range (List, optional): The angle (in some specific units) to rotate the human. Defaults to None.
        Brief description of the coordinates and what coordinate corresponds to what - 
        # from the perspective of default opening scene reference frame when playing demo, here are the descriptions of the coordinates (vice versa can be trivially inferred):
        For position offset the following are the coordinate descriptions:
        # 0th - higher value moves towards front of screen
        # 1st - higher value moves up (i.e above the floor plane)
        # 2nd - higher value moves towards left (computer users left)
        
        For the orientation quaternion here are the descriptions:
        Format of the quaternion is (x, y, z, w) - where (x, y, z) indicate the vector in 3 space, while w indicates the rotation angle about the axis defined by the position vector.
        Please look at the scipy docs here https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html#scipy.spatial.transform.Rotation to get a better sense, the rotation transform is directly taken from here in the original HR Gym implementation
        """
        # the below variables define the number of augmentations that we want to perform
        self.translation_aug = translation_aug
        self.rotation_aug = rotation_aug
        self.aug_factor = aug_factor

        # check the validity of the arguments passed
        self.check_argument_validity(translation_ranges, translation_aug, rotation_ranges, rotation_aug)

        self.translation_ranges = translation_ranges
        self.rotation_ranges = rotation_ranges

        self.augmentations_output_folder = augmentations_output_folder
        # make augmentations output path and the augmentation metadata path, if it does not exist
        if not os.path.exists(self.augmentations_output_folder):
            os.makedirs(self.augmentations_output_folder, exist_ok=True)
        
        self.aug_md_filepath = aug_md_filepath

        # get the paths of the human demos from the parent human path and the demo names - the human joints are controlled by the 
        self.demo_paths = {"infos": [], "motion_pkl": []}
        self.human_animation_names = human_animation_names
        
        for demo_name in self.human_animation_names:
            self.demo_paths["infos"].append(os.path.join(human_animation_parent_path, demo_name + "_info.json"))
            self.demo_paths['motion_pkl'].append(os.path.join(human_animation_parent_path, demo_name + ".pkl"))
        

        # make another dict, containing 2 lists to store all the pickle files, including the original and augmented ones
        self.all_demo_paths = self.demo_paths.copy()

        self.num_demos = len(human_animation_names)
        print(f"Number of original human demos is {self.num_demos}")

    def check_argument_validity(self, translation_ranges, translation_aug, rotation_ranges, rotation_aug):
        if self.aug_factor is not None:
            assert self.aug_factor >= 1, f"Augmentation factor must be >=1, got {self.aug_factor}"
        else:
            warnings.warn("Augmentation Factor is None, hence not doing any augmentations, for purposes of streamlining the implementation, setting the augmentation factor = 1")
            self.aug_factor = 1
        
        if translation_ranges is None:
            assert translation_aug == False, f"Translation ranges are None and hence translation_aug has to be false, it is {translation_aug}. Please enter a valid value for translation range parameter!"
        else:           
            assert len(translation_ranges) == 3, f"Translation ranges should be a list of 3 since we are working in 3-Dimensional space, Currently is {len(translation_ranges)}"

        if rotation_ranges is None:
            assert rotation_aug == False, f"Rotation ranges are None and hence rotation_aug has to be false, it is {rotation_aug}. Please enter a valid value for translation range parameter!"
        else:
            assert len(rotation_ranges) == 4, f"Rotation is a 4-Dimensional Quaternion and hence the size of the ranges must be = 4, Currently is {len(rotation_ranges)}"

        if translation_ranges == None and rotation_ranges ==  None:
            assert translation_aug == False and rotation_aug == False, f"Both tranlation and rotation ranges are None, and hence both translation and rotation aug indicator needs to be False! Found {translation_aug} and {rotation_aug}"
            warnings.warn(f"Since translation and rotation ranges and indicators all are indicative of no augmentation overriding augmentation factor parameter! Current value {self.aug_factor}! Setting it = 1")
            self.aug_factor = 1 # setting augmentation factor to 1 if no rotation and translation is being done, irrespective of what the current value is.

    def perturb_list(self, input_vals: List[float], perturb_ranges: List[List[float]], sampling_mode: str = 'uniform', op_mode: str = 'add'):
        """
        Args:
            input_vals (List[float]): Input list of values to be perturbed
            perturb_ranges (List[List[float]]): Ranges that we specify from which we want to sample the values of perturbation
            sampling_mode (str, optional): The distribution according to which we want to sample the value of perturbation. Defaults to 'uniform'.
        """
        output_vals = []
        assert len(input_vals) == len(perturb_ranges), f"The lengths of the values to be perturbed and the ranges of perturbation should be the same to prevent ambiguity, got {len(input_vals)} and {len(perturb_ranges)} respectively!"
        for input_val, perturb_range in zip(input_vals, perturb_ranges):
            assert len(perturb_range) == 2, f"The range of perturbation must contain only 2 real numbers since it is a range! Got {len(perturb_range)}"
            if sampling_mode == 'uniform':
                if op_mode == 'add':
                    output_val = input_val + random.uniform(*perturb_range)
                elif op_mode == 'multiply' or op_mode == 'mult':
                    output_val = input_val *  random.uniform(*perturb_range)
                else:
                    raise NotImplementedError(f"Operation mode {op_mode} is not implemented yet!")
            else:
                raise NotImplementedError(f"Sampling mode {sampling_mode} is not implemented yet!")
            output_vals.append(output_val)
        return output_vals


    def calculate_num_augmentation(self):
        # calculate the total number of augmentations that we want to perform, based on the above quantities - augmentation factor takes precedece over the rotation and translation quantities
        num_augmentations = int(self.aug_factor * len(self.human_animation_names)) - len(self.human_animation_names)
        return num_augmentations

    def generate_demos(self):
        """Function to perform data augmentation on the existing human demos
        """
        aug_count = 0
        n_aug = self.calculate_num_augmentation()
        print(f"Number of Augmentations to Generate are {n_aug}")
        for aug_num in range(n_aug):
            demo_idx = random.randint(0, self.num_demos - 1)
            demo_json = self.demo_paths['infos'][demo_idx]
            demo_name = self.human_animation_names[demo_idx]
            demo_pkl_path = self.demo_paths['motion_pkl'][demo_idx]

            with open(demo_json, "r") as file:
                json_data = json.load(file)
            new_offsets =  self.perturb_list(input_vals = json_data['position_offset'], perturb_ranges = self.translation_ranges)
            new_orientation = self.perturb_list(input_vals = json_data['orientation_quat'], perturb_ranges = self.rotation_ranges)
            new_json = json_data.copy()

            # add the augmentation offsets to the original offsets
            # please see the coordinate descriptions in the config file 
            new_json["position_offset"] = new_offsets
            new_json["orientation_quat"] = new_orientation 
            # do rotation augmentation, i.e add some random small values to the original orientation quaternion values


            # Major TODO - check if this augmentation is valid, that is, after doing the perturbations to the config file of a human demo, see if the demo is actually realisticially possible, i.e. check for collisions with static objects etc. How to do this? What would be an efficient way to checking this? The most brute force way would be just to run the simulation and see if any collisions are happenning. Are there any other ways? Maybe check the human coordinates time series and the coordinates of the static objects in the world frame and see if after augmentation, in some geometric sense there is any overlap or something like that? Do we need to build and run some lightweight simulation for this? i.e just for the purposes of mesh collision checking etc. Check this out.

            new_json_path = os.path.join(self.augmentations_output_folder, demo_name + f"_info_aug{aug_num}.json")

            # make json dir - this is solely needed to handle the way the HR Gym work takes in demo names, it takes in one-level above folder and the original filename
            json_dir_to_make = os.path.dirname(new_json_path)
            if not os.path.exists(json_dir_to_make):
                os.makedirs(json_dir_to_make, exist_ok=True)

            with open(new_json_path, "w") as file:
                json.dump(new_json, file, indent=4)
            
            self.all_demo_paths['motion_pkl'].append(demo_pkl_path)
            self.all_demo_paths['infos'].append(new_json_path)
        
            aug_count = aug_count + 1
    
        with open(self.aug_md_filepath, "w") as f:
            json.dump(self.all_demo_paths, f, indent=4)  
        
        print(f"Number of human animations after performing augmentation is {len(self.all_demo_paths['infos'])}")




# code below to test the class        
if __name__ == '__main__':
    human_animation_names= ["CMU/62_01"]
    x = TranslationXYRotationAug(translation_ranges = [[10,10],[10,10]], num_trans_aug = 3, num_rot_aug = 1, human_animation_names = human_animation_names)
    x.generate_demos()



    # def generate_demos(self):
    #     """Function to perform data augmentation on the existing human demos
    #     """
    #     aug_count = 0
    #     n_aug = self.calculate_num_augmentation()
        
    #     # TODO - this loop needs to be figured out, how we want to apply the rotation and translation augmentation
    #     for _ in range(self.num_rot_aug):

    #         # TODO - change orientation of the human by modifying the orientation quaternion - have not done this yet
    #         # Rotation Data Augmentation is not implemented yet, this is a TODO, so doing translation augmentation only.

    #         for _ in range(self.num_trans_aug):
    #             # choose a random demonstration to perform augmentation - 
    #             demo_idx = random.randint(0, self.num_demos - 1)
    #             demo_json = self.demo_paths['infos'][demo_idx]
    #             demo_name = self.human_animation_names[demo_idx]
    #             demo_pkl_path = self.demo_paths['motion_pkl'][demo_idx]

    #             with open(demo_json, "r") as file:
    #                 json_data = json.load(file)

    #             sample_x_trans = random.uniform(*self.x_trans_range)
    #             sample_y_trans = random.uniform(*self.y_trans_range)
    #             new_json = json_data.copy()

    #             # add the augmentation offsets to the original offsets
    #             # please see the coordinate descriptions in the config file 
    #             new_json["position_offset"][0] = json_data["position_offset"][0] + sample_x_trans
    #             new_json["position_offset"][2] = json_data["position_offset"][2] + sample_y_trans

    #             new_json_path = os.path.join(self.augmentations_output_folder, demo_name + f"_info_aug{aug_count}.json")

    #             # make json dir - this is solely needed to handle the way the HR Gym work takes in demo names, it takes in one-level above folder and the original filename
    #             json_dir_to_make = os.path.dirname(new_json_path)
    #             if not os.path.exists(json_dir_to_make):
    #                 os.makedirs(json_dir_to_make, exist_ok=True)

    #             with open(new_json_path, "w") as file:
    #                 json.dump(new_json, file, indent=4)
                
    #             self.all_demo_paths['motion_pkl'].append(demo_pkl_path)
    #             self.all_demo_paths['infos'].append(new_json_path)
            
    #             aug_count = aug_count + 1
    
    #     with open(self.aug_md_filepath, "w") as f:
    #         json.dump(self.all_demo_paths, f, indent=4)  
        
    #     print(f"Number of human animations after performing augmentation is {len(self.all_demo_paths['infos'])}")


