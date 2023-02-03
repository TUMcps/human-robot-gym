"""This file describes utility functions for manipulating MJCF XML models.

Owner:
    Jakob Thumm (JT)

Contributors:
    Jonathan Kuelz (JK)

Changelog:
    2.5.22 JT Formatted docstrings
"""
import os
import human_robot_gym
import robosuite

from typing import List

from scipy.spatial.transform import Rotation
from copy import deepcopy


def find_robot_assets_folder(robot_name: str) -> str:
    """Find the correct robot assets folder.

    Search the human-robot-gym (first) and robosuite (second) models.assets package
    for the given robot name and returns the folder if found.

    Args:
        robot_name (str): name of the robot (must match folder name)

    Returns:
        str: Full (absolute) file path if path found. Otherwise, returns None.
    """
    robot_asset_folder_part = "robots/" + robot_name.lower() + "/"
    robot_asset_folder_hrg = os.path.join(
        human_robot_gym.models.assets_root, robot_asset_folder_part
    )
    robot_asset_folder_robosuite = os.path.join(
        robosuite.models.assets_root, robot_asset_folder_part
    )
    if os.path.isdir(robot_asset_folder_hrg):
        return robot_asset_folder_hrg
    elif os.path.isdir(robot_asset_folder_robosuite):
        return robot_asset_folder_robosuite
    else:
        print("Couldn't find robot assets folder!")
        return None


def file_path_completion(file_path: str) -> str:
    """Take in a local file path and return a full path from human robot gym root.

    if @file_path is absolute, do nothing
    if @file_path is not absolute, load file that is shipped by the package

    Args:
        file_path (str): local file path

    Returns:
        str: Full (absolute) file path
    """
    if file_path.startswith("/"):
        full_path = file_path
    else:
        full_path = os.path.join(human_robot_gym.human_robot_gym_root, file_path)
    return full_path


def xml_path_completion(xml_path: str) -> str:
    """Take in a local xml path and returns a full path.

    if @xml_path is absolute, do nothing
    if @xml_path is not absolute, load xml that is shipped by the package

    Args:
        xml_path (str): local xml path

    Returns:
        str: Full (absolute) xml path
    """
    if xml_path.startswith("/"):
        full_path = xml_path
    else:
        full_path = os.path.join(human_robot_gym.models.assets_root, xml_path)
    return full_path


def rot_to_quat(rot: Rotation) -> List:
    """Convert a scipy rotation to a mujoco conform quaternion (w, x, y, z)."""
    quat = rot.as_quat()
    return [quat[3], quat[0], quat[1], quat[2]]


def quat_to_rot(quat: List) -> Rotation:
    """Convert a mujoco conform quaternion (w, x, y, z) to a scipy rotation."""
    scipy_quat = [quat[1], quat[2], quat[3], quat[0]]
    return Rotation.from_quat(scipy_quat)


def merge_configs(config1: dict, config2: dict) -> dict:
    """Merge two dictionaries with the following strategy.

    1) Use config1
    2) If config2 has a key but config1 doesn't, use config2's (key, value) pair
    3) If config2[KEY] is not none and config1[KEY]==none, use config2[KEY]

    Args:
        config1 (dict): The main config dictionary
        config2 (dict): The secondary config dictionary

    Returns:
        Merged config (dict)
    """
    config = deepcopy(config1)
    for key in config2:
        if key not in config or (key in config and config[key] is None):
            config[key] = config2[key]
    return config
