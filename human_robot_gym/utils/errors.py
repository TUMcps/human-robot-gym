"""This file describes custom error messages for robotics applications.

Owner:
    Jakob Thumm (JT)

Contributors:
    Jonathan Kuelz (JK)

Changelog:
    2.5.22 JT Formatted docstrings
"""
import numpy as np
import numpy.testing as np_test


class DuplicateValueError(Exception):
    """Raise whenever a duplicate value is added to a container (e.g. a list, set) that allows unique values only."""

    pass


class InvalidAssemblyError(Exception):
    """Raise when an assembly of modules should be created that's not valid."""

    pass


class UniqueValueError(ValueError):
    """Raise whenever a value, such as an ID or name should be unique, but isn't."""

    pass


class UnexpectedSpatialShapeError(ValueError):
    """Raise whenever a spatial input (point, rotation, transformation) has the wrong input shape."""

    pass


def assert_has_3d_point(p: np.ndarray):
    """Check whether the input can be interpreted as a point in cartesian space.

    Raises:
        UnexpectedSpatialShapeError
    """
    s = p.shape
    if s not in [(3,), (1, 3), (3, 1), (4, 4)]:
        raise UnexpectedSpatialShapeError(
            f"Array of shape {s} cannot be interpreted as 3D point."
        )


def assert_is_3d_point(p: np.ndarray):
    """Assert on points in cartesian space.

    Raises:
        UnexpectedSpatialShapeError
    """
    s = p.shape
    if s not in [(3,), (3, 1)]:
        raise UnexpectedSpatialShapeError(
            f"Array of shape {s} is not a point in 3D space."
        )


def assert_is_rotation_matrix(R: np.ndarray):
    """Check whether a 3x3 matrix is a valid rotation (orthonormal).

    Raises
        AssertionError
    """
    if R.shape != (3, 3):
        raise UnexpectedSpatialShapeError(
            f"Rotation matrix must be of shape (3, 3). Given: {R.shape}"
        )
    np_test.assert_allclose(
        R @ R.T, np.eye(3), err_msg="Rotation matrix is not orthogonal", atol=1e-9
    )
    np_test.assert_almost_equal(
        np.linalg.det(R),
        1.0,
        err_msg="Rotation matrix determinant != 1 (not volume preserving)",
    )


def assert_is_homogeneous_transformation(T: np.ndarray):
    """Check for shape, valid rotation matrix and last row.

    Raises:
        UnexpectedSpatialShapeError, AssertionError
    """
    if T.shape != (4, 4):
        raise UnexpectedSpatialShapeError(
            f"Array of shape {T.shape} cannot be interpreted as homogeneous transformation."
        )
    assert_is_rotation_matrix(T[:3, :3])
    np_test.assert_allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9)
