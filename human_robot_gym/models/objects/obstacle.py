"""This file describes obstacles that can be used by Pinocchio for collision detection.

Owner:
    Jakob Thumm (JT)

Contributors:
    Jonathan Kuelz (JK)

Changelog:
    2.5.22 JT Formatted docstrings
"""

from abc import ABC, abstractmethod
import itertools
from copy import copy, deepcopy
from pathlib import Path
import warnings
from typing import Dict, List

from hppfcl import CollisionObject, Transform3f
from hppfcl import hppfcl
import numpy as np
from numpy import pi
import meshcat.geometry
import pinocchio as pin

from human_robot_gym.utils.spatial import (
    homogeneous,
    rotX,
    skew,
    NO_TRANSLATION,
    NO_ROTATION,
    NEUTRAL_HOMOGENEOUS,
)


class ObstacleBase(ABC):
    """A wrapper class to integrate hppfcl obstacles in custom Environments.

    Args:
        name (str): Name of the obstacle.
        collision_objects (list(CollisionObject)): An obstacle may consist of multiple collision objects.
    """

    def __init__(
        self,
        name: str,
        collision_objects: List[CollisionObject] = None,
    ):  # noqa: D107
        self.name = name
        if collision_objects is None:
            self.collision_objects: List[CollisionObject] = []
        else:
            self.collision_objects: List[CollisionObject] = collision_objects
        if isinstance(self.collision_objects, CollisionObject):
            self.collision_objects: List[CollisionObject] = [self.collision_objects]

    def __copy__(self):
        """Deep copy the object.

        If another behavior is not desired explicitly,
        shallow copy can lead to unexpected behaviour due to the hppfcl objects.
        """
        warnings.warn(
            "{}'s copy behavior defaults to deepcopy. Explicit call to deepcopy recommended!".format(
                self.__class__
            )
        )
        return self.__deepcopy__()

    @abstractmethod
    def __deepcopy__(self, memodict={}):
        """Deep copy the object."""
        return self.__copy__()

    @property
    @abstractmethod
    def as_dict(self) -> Dict[str, any]:
        """Interface to write a crok-ready dictionary."""
        pass

    @abstractmethod
    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize the object."""
        pass

    def set_transform(
        self,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):
        """Set the transformation of the object.

        Args:
            translation (np.ndarray): translation (x, y, z) of the object.
            rotation (np.ndarray): rotation matrix (3x3) of the object.
        """
        self.T = homogeneous(translation, rotation)
        transform = Transform3f(rotation, translation)
        for collision_object in self.collision_objects:
            collision_object.setTransform(transform)


class Box(ObstacleBase):
    """Box object with side lengths [x, y, z].

    Args:
        name (str): Name of the box.
        x (float): Side length 1
        y (float): Side length 2
        z (float): Side length 3
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
    """

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):  # noqa: D107
        self.lengths = np.asarray([x, y, z])
        self.T = homogeneous(translation, rotation)
        box = hppfcl.Box(x, y, z)
        transform = Transform3f(rotation, translation)
        collision_object = CollisionObject(box, transform)
        super().__init__(name, collision_object)

    def __deepcopy__(self, memodict={}):
        """Return a copy of this object."""
        return self.__class__(self.name, *self.lengths, self.T[:3, 3], self.T[:3, :3])

    @property
    def as_dict(self) -> Dict[str, any]:
        """Return the object as a dictionary."""
        return {"pose": self.T.tolist(), "box": self.lengths.tolist()}

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize the box."""
        viz_box = meshcat.geometry.Box(self.lengths)
        viz.viewer[self.name].set_object(viz_box)
        viz.viewer[self.name].set_transform(self.T)


class Cylinder(ObstacleBase):
    """Cylinder object with radius r and length z.

    Args:
        name (str): Name of the cylinder.
        r (float): Radius of the cylinder.
        z (float): Length of the cylinder.
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
    """

    def __init__(
        self,
        name: str,
        r: float,
        z: float,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):  # noqa: D107
        # Expands a cylinder in z-direction centered in the origin
        self.r: float = float(r)
        self.z: float = float(z)
        self.T = homogeneous(translation, rotation)
        cylinder = hppfcl.Cylinder(r, z)
        transform = Transform3f(rotation, translation)
        collision_object = CollisionObject(cylinder, transform)
        super().__init__(name, collision_object)

    def __deepcopy__(self, memodict={}):
        """Return a copy of this cylinder."""
        return Cylinder(self.name, self.r, self.z, self.T[:3, 3], self.T[:3, :3])

    @property
    def as_dict(self) -> Dict[str, any]:
        """Return this cylinder as a dictionary."""
        return {"pose": self.T.tolist(), "cylinder": [self.z, self.r]}

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize this cylinder."""
        viz_cylinder = meshcat.geometry.Cylinder(height=self.z, radius=self.r)
        viz.viewer[self.name].set_object(viz_cylinder)
        viz.viewer[self.name].set_transform(self.T @ rotX(pi / 2))


class MeshObstacle(ObstacleBase):
    """Object defined by an outer mesh.

    Args:
        name (str): Name of the box.
        mesh: Collision mesh.
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
        mesh_file (str): Optional path to the mesh file as information.
            This provides no functionality at the moment.
        scale (np.ndarry): Scale of the mesh object in each dimension (x, y, z).
    """

    def __init__(
        self,
        name: str,
        mesh: hppfcl.BVHModelOBBRSS,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
        mesh_file: str = "",
        scale: np.ndarray = np.array([1, 1, 1], float),
    ):  # noqa: D107
        self.T = homogeneous(translation, rotation)
        self.mesh_geometry = mesh
        transform = Transform3f(rotation, translation)
        collision_object = CollisionObject(mesh, transform)
        self.mesh_file: str = mesh_file
        self.scale: np.ndarray = scale
        super().__init__(name, collision_object)

    def __deepcopy__(self, memodict={}):
        """Return a copy this mesh object."""
        return self.__class__(
            self.name, self.mesh_geometry.clone(), self.T[:3, 3], self.T[:3, :3]
        )

    @property
    def as_dict(self) -> Dict[str, any]:
        """Return this mesh object as a dictionary."""
        if len(self.mesh_file) == 0:
            raise ValueError(
                "Cannot store mesh obstacle if original filepath is unknown"
            )
        return {
            "pose": self.T.tolist(),
            "file": self.mesh_file,
            "scale": self.scale.tolist(),
        }

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize this mesh object."""
        viz_mesh = pin.visualize.meshcat_visualizer.loadMesh(self.mesh_geometry)
        viz.viewer[self.name].set_object(viz_mesh)
        viz.viewer[self.name].set_transform(self.T)


class Plane(ObstacleBase):
    """Plane object with normal vector n and thickness d.

    Args:
        name (str): Name of the box.
        n (np.ndarry): Normal vector on the plane.
        d (float): Thickness of the plane.
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
    """

    def __init__(
        self,
        name: str,
        n: np.ndarray = np.asarray([0, 0, 1]),
        d: float = 0.0,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):  # noqa: D107
        self.name = name
        self.n = n / np.linalg.norm(n)
        self.d = d
        self.T = homogeneous(translation, rotation)
        plane = hppfcl.Plane(self.n, d)
        transform = Transform3f(rotation, translation)
        collision_object = CollisionObject(plane, transform)
        super().__init__(name, collision_object)

    def __deepcopy__(self, memodict={}):
        """Return a copy of this plane."""
        return self.__class__(self.name, self.n, self.d, self.T[:3, 3], self.T[:3, :3])

    @property
    def as_dict(self) -> Dict[str, any]:
        """Return this plane as a dictionary."""
        raise NotImplementedError("Plane is not a standard geometry")

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize this plane.

        Meshcat cannot visualize an infinite plane, so instead visualize it as very narrow box.
        """
        # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        z_normal = np.array([0, 0, 1])
        if (self.n == -z_normal).all():
            R = rotX(pi)
        elif (self.n == z_normal).all():
            R = np.eye(4)
        else:  # Cross product won't fail for these
            v = np.cross(z_normal, self.n)
            v_skew = skew(v)
            r3 = np.eye(3) + v_skew + v_skew**2 * (1 - z_normal @ self.n)
            R = homogeneous(rotation=r3)

        viz_plane = meshcat.geometry.Box(np.array([10, 10, 0.01]))
        viz.viewer[self.name].set_object(viz_plane)
        viz.viewer[self.name].set_transform(
            R @ homogeneous(translation=self.d) @ self.T
        )


class Sphere(ObstacleBase):
    """Sphere object with radius r.

    Args:
        name (str): Name of the sphere.
        r (float): Radius of the sphere.
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
    """

    def __init__(
        self,
        name: str,
        r: float,
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):  # noqa: D107
        self.r: float = float(r)
        self.T = homogeneous(translation, rotation)
        sphere = hppfcl.Sphere(r)
        transform = Transform3f(rotation, translation)
        collision_object = CollisionObject(sphere, transform)
        super().__init__(name, collision_object)

    def __deepcopy__(self, memodict={}):
        """Return a copy of this sphere."""
        return self.__class__(self.name, self.r, self.T[:3, 3], self.T[:3, :3])

    @property
    def as_dict(self) -> Dict[str, any]:
        """Return this sphere as a dictionary."""
        return {"pose": self.T.tolist(), "sphere": self.r}

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize this sphere."""
        viz_sphere = meshcat.geometry.Sphere(self.r)
        viz.viewer[self.name].set_object(viz_sphere)
        viz.viewer[self.name].set_transform(self.T)


class ComposedObstacle(ObstacleBase):
    """Obstacle composed of multiple obstacles.

    Args:
        name (str): Name of the composed obstacle.
        obstacle (List[Obstacles]): List of obstacles this obstacle is composed of.
        translation (np.ndarray): translation (x, y, z) of the object.
        rotation (np.ndarray): rotation matrix (3x3) of the object.
    """

    def __init__(
        self,
        name: str,
        obstacles: List[ObstacleBase],
        translation: np.ndarray = NO_TRANSLATION,
        rotation: np.ndarray = NO_ROTATION,
    ):  # noqa: D107
        self.T = homogeneous(translation, rotation)
        transform3f = hppfcl.Transform3f(rotation, translation)
        self._children = obstacles
        for i, child in enumerate(self._children):
            child.T = child.T @ self.T
            for co in child.collision_objects:
                co.setTransform(co.getTransform() * transform3f)
        super().__init__(
            name,
            list(itertools.chain(*[child.collision_objects for child in obstacles])),
        )

    def __copy__(self):
        """Shallow copy all children IF they have a shallow copy.

        For obstacles, copy defaults to deepcopy.
        """
        return self.__class__(
            self.name, [copy(o) for o in self._children], self.T[:3, 3], self.T[:3, :3]
        )

    def __deepcopy__(self, memodict={}):
        """Return a copy of this composed obstacle."""
        return self.__class__(
            self.name,
            [deepcopy(o, memodict) for o in self._children],
            self.T[:3, 3],
            self.T[:3, :3],
        )

    @property
    def as_dict(self) -> List[Dict[str, any]]:
        """Return this composed obstacle as a dictionary."""
        return [obst.as_dict for obst in self._children]

    def visualize(self, viz: pin.visualize.MeshcatVisualizer):
        """Visualize this composed obstacle."""
        for child in self._children:
            child.visualize(viz)


def crok2obstacle(
    geometry: Dict[str, any], package_dir: Path, name: str, pose: np.ndarray = None
) -> ObstacleBase:
    """Take a crok geometry specification and return the according Obstacle.

    Args:
        geometry: Geometry description as dictionary (e.g. as found in crok robot collision)
        package_dir: If a mesh is given, it is given relative to a package directory that must be specified
        name: The obstacle name
        pose: The pose of the obstacle in the world frame
    Returns:
        The obstacle class instance that matches the specification
    """
    if "pose" in geometry and (pose is None):
        # Override default pause as it was provided with the geometry dict
        pose = np.asarray(geometry["pose"])
    elif pose is None:
        pose = NEUTRAL_HOMOGENEOUS

    translation = pose[:3, 3]
    rotation = pose[:3, :3]
    if "file" in geometry:
        scale = np.asarray(geometry.get("scale", [1.0, 1.0, 1.0]), dtype=float)
        if scale.size == 1:
            scale = np.ones((3,), dtype=float) * scale
        try:
            relative_path = geometry["file"]
            if relative_path.startswith(
                "/"
            ):  # Commonly used in urdf-like formats, but incompatible with pathlib
                relative_path = relative_path[1:]
            filepath = str(package_dir.joinpath(relative_path))
        except AttributeError:
            raise ValueError(
                "You must provide a package dir when a mesh file is given."
            )

        mesh_loader = hppfcl.MeshLoader()
        mesh = mesh_loader.load(filepath, scale)
        return MeshObstacle(
            name,
            mesh,
            translation=translation,
            rotation=rotation,
            mesh_file=relative_path,
            scale=scale,
        )
    elif "cylinder" in geometry:
        z, r = geometry["cylinder"]
        return Cylinder(name, r, z, translation=translation, rotation=rotation)
    elif "R_cyl" in geometry and "h_cyl" in geometry:
        z, r = geometry["h_cyl"], geometry["R_cyl"]
        return Cylinder(name, r, z, translation=translation, rotation=rotation)
    elif "box" in geometry:
        (
            x,
            y,
            z,
        ) = geometry["box"]
        return Box(name, x, y, z, translation=translation, rotation=rotation)
    else:
        raise NotImplementedError("Unknown geometry {}".format(geometry))


def hppfcl2obstacle(name: str, fcl: hppfcl.CollisionObject) -> ObstacleBase:
    """Transform a generic hppfcl collision object to an according Obstacle instance.

    (This allows plotting, composing a scenario of multiple obstacles, ...)

    Args:
        name (str): The name that shall be given to the Obstacle.
        fcl (FCL Collision Object): The hppfcl collision object.
    Returns:
        An instance of the according Obstacle class, based on the kind of collision object handed over.
    """
    translation = fcl.getTranslation()
    rotation = fcl.getRotation()
    geom = fcl.collisionGeometry()
    if isinstance(geom, hppfcl.Box):
        return Box(
            name,
            *[2 * hs for hs in geom.halfSide],
            translation=translation,
            rotation=rotation
        )
    elif isinstance(geom, hppfcl.Cylinder):
        return Cylinder(
            name,
            geom.radius,
            2 * geom.halfLength,
            translation=translation,
            rotation=rotation,
        )
    elif isinstance(geom, hppfcl.Sphere):
        return Sphere(name, geom.radius, translation=translation, rotation=rotation)
    elif isinstance(geom, hppfcl.Plane):
        return Plane(name, geom.n, geom.d, translation=translation, rotation=rotation)
    elif isinstance(geom, hppfcl.BVHModelOBBRSS):
        return MeshObstacle(name, geom, translation=translation, rotation=rotation)
    else:
        raise NotImplementedError()
