"""This file defines a converter from BVH to pickle animations.

Load in a human animation BVH file and transform it to an usable human animation.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import argparse
import os
import numpy as np
import pickle

from bvh import Bvh
from scipy.spatial.transform import Rotation


if __name__ == "__main__":
    # << Load in arguments >>
    parser = argparse.ArgumentParser(
        description="Convert a BVH motion capture file to a usable animation and save the animation as a pickle file."
    )
    parser.add_argument("path", help="Path to bvh file. Has to end in .bvh", type=str)
    parser.add_argument(
        "--save_path",
        "-s",
        help="Path to save the pickle file. Has to end in .pkl",
        type=str,
    )
    args = parser.parse_args()
    path = args.path
    file_name, file_extension = os.path.splitext(path)
    assert file_extension == ".bvh", "BVH file has to end in .bvh"
    assert os.path.isfile(path), "BVH file does not exist!"
    save_path = args.save_path
    if save_path is not None:
        _, file_extension = os.path.splitext(save_path)
        assert file_extension == ".pkl", "File to save pkl has to end in .pkl"
    else:
        save_path = file_name + ".pkl"

    # << Load BVH file >>
    with open(path) as f:
        mocap = Bvh(f.read())
    mocap.get_joint_channels_index("hip")
    mocap.get_joint("hip")
    mujoco_to_mocap_names = {
        "L_Hip": "lThigh",
        "R_Hip": "rThigh",
        "Torso": "abdomen",
        "L_Knee": "lShin",
        "R_Knee": "rShin",
        "Spine": None,
        "L_Ankle": "lFoot",
        "R_Ankle": "rFoot",
        "Chest": "chest",
        "L_Toe": None,
        "R_Toe": None,
        "Neck": "neck",
        "L_Thorax": "lCollar",
        "R_Thorax": "rCollar",
        "Head": "head",
        "L_Shoulder": "lShldr",
        "R_Shoulder": "rShldr",
        "L_Elbow": "lForeArm",
        "R_Elbow": "rForeArm",
        "L_Wrist": "lHand",
        "R_Wrist": "rHand",
        "L_Hand": None,
        "R_Hand": None,
    }
    frames = np.asarray(mocap.frames, dtype=np.float64, order="C")
    data = {}
    channel_dict = {}
    base_idx = mocap.get_joint_channels_index("hip")
    for (i, channel_name) in enumerate(mocap.joint_channels("hip")):
        channel_dict[channel_name] = base_idx + i

    data["Pelvis_pos_x"] = frames[:, channel_dict["Xposition"]] / 100
    data["Pelvis_pos_y"] = frames[:, channel_dict["Yposition"]] / 100
    data["Pelvis_pos_z"] = frames[:, channel_dict["Zposition"]] / 100
    rot = Rotation.from_euler(
        "ZYX",
        np.swapaxes(
            np.array(
                [
                    frames[:, channel_dict["Zrotation"]],
                    frames[:, channel_dict["Yrotation"]],
                    frames[:, channel_dict["Xrotation"]],
                ]
            ),
            0,
            1,
        ),
        degrees=True,
    )
    data["Pelvis_quat"] = rot.as_quat()

    for joint_name in mujoco_to_mocap_names:
        if mujoco_to_mocap_names[joint_name] is not None:
            mocap_name = mujoco_to_mocap_names[joint_name]
            base_idx = mocap.get_joint_channels_index(mocap_name)
            channel_dict = {}
            for (i, channel_name) in enumerate(mocap.joint_channels(mocap_name)):
                channel_dict[channel_name] = base_idx + i
            data[joint_name + "_x"] = np.clip(
                np.radians(frames[:, channel_dict["Xrotation"]]), -1.56, 1.56
            )
            data[joint_name + "_y"] = np.clip(
                np.radians(frames[:, channel_dict["Yrotation"]]), -1.56, 1.56
            )
            data[joint_name + "_z"] = np.clip(
                np.radians(frames[:, channel_dict["Zrotation"]]), -1.56, 1.56
            )
        else:
            data[joint_name + "_x"] = np.zeros(data["Pelvis_pos_x"].shape)
            data[joint_name + "_y"] = np.zeros(data["Pelvis_pos_x"].shape)
            data[joint_name + "_z"] = np.zeros(data["Pelvis_pos_x"].shape)

    output = open(save_path, "wb")
    pickle.dump(data, output)
    output.close()
