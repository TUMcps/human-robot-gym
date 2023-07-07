r"""This file can be used to rotate a hand in a human animation file to point upwards.

By default, wrist rotations of animations generated using the vicon tracker system is
disregarded. However, in some cases (e.g. handover tasks) it may be required to have
a palm of the human facing up when reaching over the table.

This script applies a constant offset to the wrist x joint angle to manually account for that.

For generating the animations for the robot-human handover task, the following commands were used:

```
python human_robot_gym/utils/bvh_rotate_palm_up.py \
    path/to/input/file.bvh -s path/to/output/file.bvh -j left_hand -o 100
```

```
python human_robot_gym/utils/bvh_rotate_palm_up.py \
    path/to/input/file.bvh -s path/to/output/file.bvh -j right_hand -o 100
```

Author:
    Felix Trost (FT)

Changelog:
    03.07.23 FT File creation
"""
import argparse
import os

import bvhtoolbox


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Rotate a hand in a human animation file to point upwards."
    )

    parser.add_argument(
        "path",
        help="Path to bvh file. Has to end in .bvh",
        type=str,
    )

    parser.add_argument(
        "--save-path",
        "-s",
        help="Path to save the pickle file. Has to end in .pkl",
        type=str,
        required=False,
    )

    parser.add_argument(
        "--in-place",
        "-i",
        help="Whether to save the file in place or not",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "--joint",
        "-j",
        help="Joint to apply the offset to. Has to be one of 'left_hand' or 'right_hand'",
        type=str,
        choices=["left_hand", "right_hand"],
        required=True,
    )

    parser.add_argument(
        "--angle-offset",
        "-a",
        help="Offset to apply to the wrist x joint angle in degrees",
        type=float,
        default=120.0,
        required=False,
    )

    args = parser.parse_args()

    load_path = args.path

    if args.in_place:
        save_path = load_path
    else:
        save_path = args.save_path

    assert os.path.isfile(load_path), "BVH file does not exist!"
    assert args.in_place or args.save_path is not None, "Please specify a save path or use the --in-place flag!"
    assert args.in_place or not os.path.isfile(save_path), "Save path already exists while not storing in-place!"

    load_file_name, load_file_extension = os.path.splitext(load_path)
    assert os.path.splitext(save_path)[1] == load_file_extension == ".bvh", "BVH file has to end in .bvh"

    with open(load_path) as f:
        mocap = bvhtoolbox.BvhTree(f.read())

    channel_index = mocap.get_joint_channels_index(
        args.joint
    ) + mocap.get_joint_channel_index(args.joint, "Xrotation")

    frames = bvhtoolbox.get_motion_data(mocap)
    frames[:, channel_index] += args.angle_offset

    bvhtoolbox.set_motion_data(mocap, frames)

    mocap.write_file(save_path)

    print("Done.")
