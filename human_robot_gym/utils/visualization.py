"""This file describes visualization function for the pinoccio manipulator model.

Owner:
    Jakob Thumm (JT)

Contributors:
    Jonathan Kuelz (JK)

Changelog:
    2.5.22 JT Formatted docstrings
"""
from pathlib import Path

import meshcat
import meshcat.geometry
import numpy as np
from pinocchio.visualize import MeshcatVisualizer

import human_robot_gym.utils.errors
import human_robot_gym.utils.spatial


def place_arrow(
    viz: MeshcatVisualizer,
    name: str,
    material: meshcat.geometry.Material = meshcat.geometry.MeshBasicMaterial(),
    scale: float = 1.0,
    pose: np.ndarray = human_robot_gym.utils.spatial.NEUTRAL_HOMOGENEOUS,
    axis: str = "z",
) -> None:
    """Create a composed meshcat geometry object that looks like an arrow.

    Args:
        viz: Visualizer instance to draw into
        name: Unique object name for the visualizer
        material: If provided, this defines the appearance (e.g. color)
        scale: Arrow length in meters will be 0.1 * scale
        pose: Defines orientation and placement of the arrow. 4x4 homogeneous matrix
        axis: Axis alignment of the arrow. The default is alignment to the z-axis in the "pose" coordinate frame.
    """
    human_robot_gym.utils.errors.assert_is_homogeneous_transformation(pose)
    if axis == "x":
        rot = human_robot_gym.utils.spatial.rotZ(-np.pi / 2)
    elif axis == "y":
        rot = human_robot_gym.utils.spatial.NEUTRAL_HOMOGENEOUS
    elif axis == "z":
        rot = human_robot_gym.utils.spatial.rotX(np.pi / 2)
    else:
        raise ValueError("Invalid argument for axis: {}".format(axis))

    length = 0.1 * scale
    base_length = length * 3 / 5
    base_width = length / 7
    head_length = length * 2 / 5
    rmax_head = 1.5 * length / 5
    base = meshcat.geometry.Cylinder(base_length, base_width)
    head = meshcat.geometry.Cylinder(head_length, radiusTop=0, radiusBottom=rmax_head)

    viz.viewer[name + "_arr_body"].set_object(base, material)
    base_transform = (
        pose
        @ rot
        @ human_robot_gym.utils.spatial.homogeneous(
            translation=[0, -0.5 * base_length - head_length, 0]
        )
    )
    viz.viewer[name + "_arr_body"].set_transform(base_transform)

    viz.viewer[name + "_arr_head"].set_object(head, material)
    head_transform = (
        pose
        @ rot
        @ human_robot_gym.utils.spatial.homogeneous(
            translation=[0, -0.5 * head_length, 0]
        )
    )
    viz.viewer[name + "_arr_head"].set_transform(head_transform)


def drawable_coordinate_system(
    transformation: np.ndarray, scale: float = 1.0
) -> meshcat.geometry:
    """Return a visualization of the coordinate system.

    A visual representation of the origin of a coordinate system, drawn as three
    lines in red, green, and blue along the x, y, and z axes. The `scale` parameter
    controls the length of the three lines.
    Returns an `Object` which can be passed to `set_object()`
    Other than meshcat.geometry.triad, this allows drawing the triad in any coordinate system, which is
    defined by <original coordinate system @ transformation>.

    Args:
        transformation: 4x4 homogeneous transformation
        scale: Length of the drawn vectors for the coordinate system main axes
    Returns:
        A meshcat object that can be visualize via viewer[your_name].set_object(this)
    """
    human_robot_gym.utils.errors.assert_is_homogeneous_transformation(transformation)
    p0 = (transformation @ human_robot_gym.utils.spatial.NEUTRAL_HOMOGENEOUS)[:3, 3]
    x = (
        transformation
        @ human_robot_gym.utils.spatial.homogeneous(translation=[scale, 0, 0])
    )[:3, 3]
    y = (
        transformation
        @ human_robot_gym.utils.spatial.homogeneous(translation=[0, scale, 0])
    )[:3, 3]
    z = (
        transformation
        @ human_robot_gym.utils.spatial.homogeneous(translation=[0, 0, scale])
    )[:3, 3]
    return meshcat.geometry.LineSegments(
        geometry=meshcat.geometry.PointsGeometry(
            position=np.array([p0, x, p0, y, p0, z], dtype=np.float32).T,
            color=np.array(
                [[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]
            )
            .astype(np.float32)
            .T,
        ),
        material=meshcat.geometry.LineBasicMaterial(vertexColors=True),
    )


def movie(robot, q: np.ndarray, dt: float, save_as: Path = None):
    """Create a movie clip of the pinocchio robot.

    Args:
        q (np.ndarray): Joint configuration.
        dt (float): Time step length of video frames.
        save_as (Path): Path to file.
    """
    import cv2

    if q.shape[1] != robot.njoints:
        raise ValueError(
            "The provided configurations must be of shape time steps x dof"
        )

    robot.update_configuration(q[0, :])
    viz = robot.visualize()
    img = viz.play(q.T, dt, capture=True)

    """https://stackoverflow.com/questions/52414148/turn-pil-images-into-video-on-linux"""
    if save_as is not None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video = cv2.VideoWriter(str(save_as), fourcc, 10, (1280, 910))
        for image in img:
            video.write(image[:, :, :3])
        video.release()
