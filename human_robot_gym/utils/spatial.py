"""This file describes spatial utility functions for the pinoccio manipulator model.

Owner:
    Jakob Thumm (JT)

Contributors:
    Jonathan Kuelz (JK)

Changelog:
    2.5.22 JT Formatted docstrings
"""
from typing import Collection, List, Tuple, Union

import hppfcl
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

import human_robot_gym.utils.errors as err

NO_TRANSLATION = np.zeros([3], float)
NO_ROTATION = np.eye(3)
NEUTRAL_HOMOGENEOUS = np.asarray(
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],  # == np.eye(4)
    dtype=float,
)


# ----- Projections Start -----
def cartesian2cylindrical(cart: np.ndarray) -> np.ndarray:
    """Convert a (3,) array in cartesian coordinates [x, y, z] to cylindrical coordinates [r, phi, z].

    https://en.wikipedia.org/wiki/Polar_coordinate_system
    """
    if not cart.squeeze().shape == (3,):
        raise ValueError("Expected (3,) array, got {}".format(cart.shape))
    x, y, z = cart
    r = (x**2 + y**2) ** 0.5
    if r == 0:
        phi = 0
    elif y >= 0:
        phi = np.arccos(x / r)
    else:
        phi = -np.arccos(x / r)
    return np.array([r, phi, z])


def cylindrical2cartesian(cyl: np.ndarray) -> np.ndarray:
    """Convert a (3,) array in cylindrical coordinates [r, phi, z] to cartesian coordinates [x, y, z]."""
    if not cyl.squeeze().shape == (3,):
        raise ValueError("Expected (3,) array, got {}".format(cyl.shape))
    r, phi, z = cyl
    return np.array([r * np.cos(phi), r * np.sin(phi), z], float)


def cartesian2spherical(cart: np.ndarray) -> np.ndarray:
    """Convert a (3,) array in cartesian coordinates [x, y, z] to spherical coordinates [r, theta, phi].

    Convention: r, theta, phi ~ U, S, E (ISO)
    https://en.wikipedia.org/wiki/Spherical_coordinate_system
    """
    if not cart.squeeze().shape == (3,):
        raise ValueError("Expected (3,) array, got {}".format(cart.shape))
    x, y, z = cart
    r = (x**2 + y**2 + z**2) ** 0.5
    if r == 0:
        return np.array([0, 0, 0], float)

    theta = np.arccos(z / r)
    if x == 0:
        if y == 0:
            phi = 0
        elif y > 0:
            phi = np.pi / 2
        else:
            phi = -np.pi / 2
    elif x > 0:
        phi = np.arctan(y / x)
    else:
        if y >= 0:
            phi = np.arctan(y / x) + np.pi
        else:
            phi = np.arctan(y / x) - np.pi
    return np.array([r, theta, phi], float)


def spherical2cartesian(spher: np.ndarray) -> np.ndarray:
    """Convert a (3,) array in spherical coordinates [r, theta, phi] to cartesian coordinates [x, y, z]."""
    if not spher.squeeze().shape == (3,):
        raise ValueError("Expected (3,) array, got {}".format(spher.shape))
    r, theta, phi = spher
    return np.array(
        [
            r * np.cos(phi) * np.sin(theta),
            r * np.sin(phi) * np.sin(theta),
            r * np.cos(theta),
        ],
        float,
    )


def rot_mat2axis_angle(rot_mat: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to the corresponding axis-angle representation.

    Args:
        rot_mat: 3x3 rotation matrix
    Returns:
        4x1 array of axis-angle representation (n_x, n_y, n_z, theta_R), where n_x, n_y, n_z ~ unit vector
    """
    axis_angle = Rotation.from_matrix(rot_mat).as_rotvec()
    if all(axis_angle == 0.0):
        return np.zeros(4, float)
    axis_angle_4d = np.concatenate(
        (
            axis_angle / np.linalg.norm(axis_angle),
            np.array([(np.pi + np.linalg.norm(axis_angle)) % (2 * np.pi) - np.pi]),
        )
    )
    return axis_angle_4d


def axis_angle2rot_mat(axis_angle: np.ndarray) -> np.ndarray:
    """Convert a axis-angle representation to the corresponding rotation matrix."""
    rot_vec = axis_angle[:3] * axis_angle[3]
    return Rotation.from_rotvec(rot_vec).as_matrix()


# ----- Projections End -----


def clone_collision_object(co: hppfcl.CollisionObject) -> hppfcl.CollisionObject:
    """Return a copy of a hppfcl collision object."""
    return hppfcl.CollisionObject(co.collisionGeometry().clone(), co.getTransform())


def euler(rotation: Collection[float], seq: str):
    """Wrap the scipy euler method.

    Args:
        rotation: Rotation angles in radian
        seq: Any ordering of {xyz}[intrinsic] or {XYZ}[extrinsic] axes.
    Returns:
        3x3 rotation matrix
    """
    return Rotation.from_euler(seq, rotation).as_matrix()


def frame2geom(frame_id: int, geom_model: pin.GeometryModel):
    """Return all geometry objects where porent frame is frame_id."""
    return [geom for geom in geom_model.geometryObjects if geom.parentFrame == frame_id]


def homogeneous(
    translation: Union[
        List[float], Tuple[float, float, float], np.ndarray
    ] = NO_TRANSLATION,
    rotation: np.ndarray = NO_ROTATION,
) -> np.ndarray:
    """Return a homogeneous matrix for translation, then rotation.

    Args:
        translation: Translation in the homogeneous matrix. 3x1
        rotation: 3x3 rotation matrix. Nested List works as well

    Returns:
        np.ndarray (4x4): homogenous transformation
    """
    T = np.eye(4)
    T[:3, 3] = np.asarray(translation)
    T[:3, :3] = rotation
    return T


def inv_homogeneous(T: np.ndarray) -> np.ndarray:
    """Inverse a homogeneous transformation."""
    if T.shape != (4, 4):
        raise err.UnexpectedSpatialShapeError(
            "Homogeneous transformation must be of shape 4x4"
        )
    return homogeneous(-np.transpose(T[:3, :3]) @ T[:3, 3], np.transpose(T[:3, :3]))


def random_rotation() -> np.ndarray:
    """Return a random 3x3 rotation matrix."""
    return Rotation.random().as_matrix()


def random_homogeneous() -> np.ndarray:
    """Return a random 4x4 homogeneous transformation matrix."""
    p = np.random.random((3,))
    return homogeneous(translation=p, rotation=random_rotation())


def rot2D(angle: float) -> np.ndarray:
    """Return the 2D rotation matrix for an angle in radian.

    Args:
        angle (float): Angel in radian

    Returns:
        np.ndarray (2x2): 2D rotation
    """
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def rotX(alpha: float) -> np.ndarray:
    """Return the homogenous transformation matrix of an x-Rotation with angle alpha."""
    R = np.eye(4)
    R[1:3, 1:3] = rot2D(alpha)
    return R


def rotY(beta: float) -> np.ndarray:
    """Return the homogenous transformation matrix of an y-Rotation with angle beta."""
    R = np.eye(4)
    R[::2, ::2] = rot2D(beta).T
    return R


def rotZ(gamma: float) -> np.ndarray:
    """Return the homogenous transformation matrix of an z-Rotation with angle gamma."""
    R = np.eye(4)
    R[:2, :2] = rot2D(gamma)
    return R


def skew(vec3: np.ndarray) -> np.ndarray:
    """Return a skew matrix."""
    if vec3.shape != (3,):
        raise err.UnexpectedSpatialShapeError("Invalid shape for skew symmetric matrix")
    return np.array(
        [[0, -vec3[2], vec3[1]], [vec3[2], 0, -vec3[0]], [-vec3[1], vec3[0], 0]]
    )


def xyz_rpy_to_homogeneous(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """Transform a URDF-Like position description of xyz and roll pitch yaw to a homogeneous transform.

    Uses EXTRINSIC euler rotation (https://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/)
    """
    rot = Rotation.from_euler("xyz", rpy).as_matrix()
    return homogeneous(xyz, rot)
