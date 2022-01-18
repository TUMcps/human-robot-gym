import pickle
import numpy as np

from bvh import Bvh
from human_robot_gym.utils.mjcf_utils import xml_path_completion

if __name__ == "__main__":
    path = xml_path_completion('human/animations/62_01_no_hand.bvh')
    path = "/home/jakob/OneDrive/Promotion/Work/CMU_motion_cap/cmuconvert-daz-60-75/61/test.bvh"
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
        "R_Hand": None
    }
    frames = np.asarray(mocap.frames, dtype=np.float64, order='C')
    data = {}
    channel_dict = {}
    base_idx = mocap.get_joint_channels_index("hip")
    for (i, channel_name) in enumerate(mocap.joint_channels("hip")):
        channel_dict[channel_name] = base_idx + i

    data["Pelvis_pos_x"] = frames[:, channel_dict["Xposition"]]/100
    data["Pelvis_pos_y"] = frames[:, channel_dict["Yposition"]]/100
    data["Pelvis_pos_z"] = frames[:, channel_dict["Zposition"]]/100
    data["Pelvis_rot_x"] = np.clip(np.radians(frames[:, channel_dict["Xrotation"]]), -1.56, 1.56)
    data["Pelvis_rot_y"] = np.clip(np.radians(frames[:, channel_dict["Yrotation"]]), -1.56, 1.56)
    data["Pelvis_rot_z"] = np.clip(np.radians(frames[:, channel_dict["Zrotation"]]), -1.56, 1.56)
    for joint_name in mujoco_to_mocap_names:
        if mujoco_to_mocap_names[joint_name] is not None:
            mocap_name = mujoco_to_mocap_names[joint_name]
            base_idx = mocap.get_joint_channels_index(mocap_name)
            channel_dict = {}
            for (i, channel_name) in enumerate(mocap.joint_channels(mocap_name)):
                channel_dict[channel_name] = base_idx + i
            data[joint_name + "_x"] = np.clip(np.radians(frames[:, channel_dict["Xrotation"]]), -1.56, 1.56)
            data[joint_name + "_y"] = np.clip(np.radians(frames[:, channel_dict["Yrotation"]]), -1.56, 1.56)
            data[joint_name + "_z"] = np.clip(np.radians(frames[:, channel_dict["Zrotation"]]), -1.56, 1.56)
        else:
            data[joint_name + "_x"] = np.zeros(data["Pelvis_pos_x"].shape)
            data[joint_name + "_y"] = np.zeros(data["Pelvis_pos_x"].shape)
            data[joint_name + "_z"] = np.zeros(data["Pelvis_pos_x"].shape)
    
    output = open(xml_path_completion('human/animations/test.pkl'), 'wb')
    pickle.dump(data, output)
    output.close()