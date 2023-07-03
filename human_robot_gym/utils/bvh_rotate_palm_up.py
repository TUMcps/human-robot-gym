"""This file can be used to rotate a hand in a human animation file to point upwards.

By default, wrist rotations of animations generated using the vicon tracker system is
disregarded. However, in some cases (e.g. handover tasks) it may be required to have
a palm of the human facing up when reaching over the table.

This script applies a constant offset to the wrist x joint angle to manually account for that.

Author:
    Felix Trost (FT)

Changelog:
    03.07.23 FT File creation
"""
import argparse
import os
import numpy as np

from bvh import Bvh


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Rotate a hand in a human animation file to point upwards."
    )

    parser.add_argument(
        "path",
        help="Path to bvh file. Has to end in .bvh",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--save_path",
        "-s",
        help="Path to save the pickle file. Has to end in .pkl",
        type=str,
        required=True,
    )

    parser.add_argument(
        "--joint",
        "-j",
        help="Joint to apply the offset to. Has to be one of 'L_Wrist' or 'R_Wrist'",
        type=str,
        choices=["L_Wrist", "R_Wrist"],
        required=True,
    )

    parser.add_argument(
        "--offset",
        "-o",
        help="Offset to apply to the wrist x joint angle in degrees",
        type=float,
        default=120.0,
        required=False,
    )

    args = parser.parse_args()

    load_path = args.path
    save_path = args.save_path

    assert os.path.isfile(load_path), "BVH file does not exist!"
    assert not os.path.exists(save_path), "Save path already exists!"

    load_file_name, load_file_extension = os.path.splitext(load_path)
    assert os.path.splitext(save_path)[1] == load_file_extension == ".bvh", "BVH file has to end in .bvh"

    with open(load_path) as f:
        mocap = Bvh(f.read())

    mocap.get_joint_channels_index(f"{args.joint}_")
