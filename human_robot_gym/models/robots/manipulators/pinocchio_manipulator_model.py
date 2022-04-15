from collections import OrderedDict
from tokenize import Double
from typing import Collection, Dict, List, Tuple, Union
import numpy as np
import itertools

import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer
from hppfcl import CollisionRequest, CollisionResult, collide, CollisionObject
from hppfcl import hppfcl
from robosuite.models.robots.manipulators import ManipulatorModel

from human_robot_gym.utils.pinocchio_utils import q_pin
import human_robot_gym.utils.errors as err
from human_robot_gym.utils import spatial
from human_robot_gym.utils.visualization import drawable_coordinate_system


class PinocchioManipulatorModel(ManipulatorModel):
    """
    Base class for all manipulator models (robot arm(s) with gripper(s)) when Pinocchio should be used.

    Args:
        fname (str): Path to relevant xml file from which to create this robot instance
        urdf_file (str): Path to relevant urdf file from which to create the pinocchio instance
        package_dirs (str): Path to package where mesh files lie (as defined in urdf file)
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, fname, urdf_file, package_dirs, idn=0):
        # Always run super init first
        super().__init__(fname, idn=idn)
        self.placement = np.eye(4)
        ## Setup robot representation in pinocchio. This is used for
        # - Collision checking
        # - Jacobian / Joint velocities
        #robot_assets_folder = find_robot_assets_folder(self.robots[0].name)
        self.pin_model, self.collision, self.visual = pin.buildModelsFromUrdf(
            urdf_file, 
            package_dirs=package_dirs, 
            geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL])
        self.pin_data = self.pin_model.createData()
        self.pin_local_world_aligned_frame = pin.ReferenceFrame(2)
        self.configuration = pin.neutral(self.pin_model)
        ## Visualization only for debugging and testing.
        #self.visualize_pinocchio_robot(self.pin_model, self.collision, self.visual, q)
        ## Collisions
        self._init_collision_pairs()
        self.collision_data = self.collision.createData()
        self.visual_data = self.visual.createData()
        self._update_geometry_placement()
        self._remove_home_collisions()

    def _update_geometry_placement(self):
        """Updates the collision (for collision detection) and visual (for plotting) geometry placements."""
        self._update_collision_placement()
        #self._update_visual_placement()

    def move(self, displacement: Union[np.ndarray, pin.SE3]):
        """Moves frames, joints and geometries."""
        if isinstance(displacement, np.ndarray):
            displacement = pin.SE3(displacement)
        for i, frame in enumerate(self.pin_model.frames):
            if frame.type == pin.FrameType.JOINT:
                break
            cos = spatial.frame2geom(i, self.collision)
            vos = spatial.frame2geom(i, self.visual)
            for co in cos:
                co.placement = displacement * co.placement
            for vo in vos:
                vo.placement = displacement * vo.placement
            frame.placement = displacement * frame.placement
        self.pin_model.jointPlacements[0] = displacement * self.pin_model.jointPlacements[0]
        try:
            self.pin_model.jointPlacements[1] = displacement * self.pin_model.jointPlacements[1]
        except IndexError:
            assert self.dof == 0, "Could not find the first joint, but there seems to be one"
        self.update_configuration(self.configuration)
        self._base_placement = self.placement @ displacement.homogeneous

    def set_base_placement(self, pose: np.ndarray) -> None:
        """Moves the base of the pinocchio model to desired position"""
        err.assert_is_homogeneous_transformation(pose)
        self.move(pose @ spatial.inv_homogeneous(self.placement))

    def update_configuration(self,
                             q: np.ndarray,
                             dq: np.ndarray = None,
                             ddq: np.ndarray = None,
                             *,
                             frames: bool = False,
                             geometry: bool = False):
        """
        Update the robot configuration by updating the joint parameters.
        :param q: The new configuration in joint space iven as numpy array of joint values.
        :param dq: Joint velocities, optional
        :param ddq: Joint acceleration, optional
        :param frames: If true, frame placements are also updated
        :param geometry: If true, geometry (visual and collision) placements are updated
        :return: None
        """
        self.configuration = q
        if dq is not None:
            self.velocities = dq
            if ddq is None:
                pin.forwardKinematics(self.pin_model, self.pin_data, self.configuration, dq)
            else:
                self.accelerations = ddq
                pin.forwardKinematics(self.pin_model, self.pin_data, self.configuration, dq, ddq)
        else:
            pin.forwardKinematics(self.pin_model, self.pin_data, self.configuration)
        if frames:
            pin.updateFramePlacements(self.pin_model, self.pin_data)
        if geometry:
            self._update_geometry_placement()


    # ---------- Collision Methods ----------

    def _init_collision_pairs(self):
        """
        Adds all collision pairs to the robot.
        """
        self.collision.addAllCollisionPairs()

    def check_collision(self, q, collision_object):
        """
        Check collision between robot in configuration q and an object.
        """
        q = q_pin(q)
        if q is not None:
            self.update_configuration(q)
        self._update_collision_placement()
        request = CollisionRequest()
        result = CollisionResult()
        # Iterate over all robot collision objects
        robot_coll_geometries = [rco.geometry for rco in self.collision_objects]
        for (robot_coll_geo, robot_coll_trans) in zip(robot_coll_geometries, self.collision_data.oMg):
            # Iterate over all sub-collision objects in the given object
            for collision_part in collision_object.collision_objects:
                if collide(robot_coll_geo, 
                          robot_coll_trans, 
                          collision_part.collisionGeometry(),
                          collision_part.getTransform(), 
                          request, 
                          result):
                    return True
        return False

    def has_self_collision(self, q: np.ndarray = None, safety_dist: Double = 0.0) -> bool:
        """
        Returns true if there are any self collisions in the current robot configuration
        If q is provided, the robot configuration will be changed. If not, this defaults to the current configuration.
        """
        q = q_pin(q) if q is not None else None
        if q is not None:
            self.update_configuration(q)
        self._update_collision_placement()
        pin.computeDistances(self.pin_model, self.pin_data, self.collision, self.collision_data, self.configuration)
        for dist in self.collision_data.distanceResults:
            if dist.min_distance <= safety_dist:
                return True
        return False

    def _remove_home_collisions(self):
        """
        Removes all collision pairs that are in collision while the robot is in its home configuration.
        This can be replaced by manually defining collision pairs to be checked or reading an SRDF file that defines the
        collision pairs that can be ignored.
        Keep in mind that this method leads to never detecting collisions that already exist in
        the home/zero configuration of the robot - so it should eventually be replaced by something more sophisticated.
        """
        # TODO: Implement more precise methods, e.g. SRDF input
        # print("WARNING: REMOVING ALL COLLISIIONS IN ZERO POSITION")
        self._update_collision_placement()
        pin.computeCollisions(self.pin_model, self.pin_data, self.collision, self.collision_data, self.configuration, False)
        active = np.zeros((self.collision.ngeoms, self.collision.ngeoms), bool)
        for k in range(len(self.collision.collisionPairs)):
            cr = self.collision_data.collisionResults[k]
            cp = self.collision.collisionPairs[k]
            if cr.isCollision():
                active[cp.first, cp.second] = False
            else:
                active[cp.first, cp.second] = True

        self.collision.setCollisionPairs(active)
        self.collision_data = self.collision.createData()
        assert not self.has_self_collision(), "Self collisions should have been removed by now..."

    def _update_collision_placement(self):
        """Updates the placement of collision geometry objects"""
        pin.updateGeometryPlacements(self.pin_model, self.pin_data, self.collision, self.collision_data)

    # ---------------------------- Visualize Gepetto -------------------------------------
    def visualize_gepetto(self, q):
        """
        Visualize pinocchio robot for debugging purposes.
        This requires to start the gepetto viewer first using `gepetto-gui`.

        Args:
            q (np.array): Joint angles to visualize
        """
        q = q_pin(q)
        viz = GepettoVisualizer(self.pin_model, self.collision, self.visual)
        # Initialize the viewer.
        try:
            viz.initViewer()
        except ImportError as err:
            print("Error while initializing the viewer. It seems you should install gepetto-viewer")
            print(err)
        try:
            viz.loadViewerModel("pinocchio")
            viz.display(q_pin(q))
        except AttributeError as err:
            print("Error while loading the viewer model. It seems you should start gepetto-viewer")
            print(err)

    def get_joint_vel(self, q, dq):
        """
        Returns the cartesian velocity and angular velocity of each joint in 
          (x, y, z, \omega_x, \omega_y, \omega_z) in world coordinates.

        Args:
            q (np.array): Joint configuration
            dq (np.array): Joint velocity
        """
        q = q_pin(q); dq = q_pin(dq)
        # Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame. 
        # The result is accessible through data.J. 
        # This function computes also the forwardKinematics of the model. 
        pin.computeJointJacobians(self.pin_model, self.pin_data, q)
        # Check velocity
        v_joints = []
        # +2: 1 for pinocchio base joint, 1 for fake end joint
        for i in range(1, self.dof + 2):
            joint_jacobian = pin.getJointJacobian(self.pin_model, self.pin_data, i, self.pin_local_world_aligned_frame)
            v_joints.append(np.matmul(joint_jacobian, dq))
        return v_joints

    @property
    def collision_pairs(self) -> np.ndarray:
        """
        Uses the collision_objects property and returns all pairs that are in collision
        :return: An nx2 numpy array of collision pair inidices, where n is the number of pairs in collision
        """
        cp = np.asarray(
            [[cp.first, cp.second] for cp in self.collision.collisionPairs],
            dtype=int
        )
        if cp.size == 0:
            cp = np.empty([0, 2])
        return cp

    @property
    def collision_objects(self) -> Tuple[CollisionObject]:
        """
        Returns all collision objects of the robot (internally wrapped by pinocchio) as hppfcl collision objects.
        :returns: A list of hppfcl collision objects that can be used for fast collision checking.
        """
        return tuple(self.collision.geometryObjects)


    # ---------- Visualization ----------
    def _update_visual_placement(self):
        """Updates the placement of visual geometry objects"""
        pin.updateGeometryPlacements(self.pin_model, self.pin_data, self.visual, self.visual_data)

    def plot(self,
             visualizer: pin.visualize.BaseVisualizer = None,
             coordinate_systems: Union[str, None] = None
             ) -> pin.visualize.MeshcatVisualizer:
        """
        Displays the robot in its current environment using a Meshcat Visualizer in the browser
        :param visualizer: A meshcat visualizer instance. If none, a new will be created
        :param coordinate_systems: Can be None, 'tcp', 'joints' or 'full' - the specified coordinates will be drawn
          (full means all frames, as in fk)
        :return: The meshcat visualizer instance used for plotting the robot
        """
        if coordinate_systems not in (None, 'joints', 'full', 'tcp'):
            raise ValueError("Invalid Value for coordinate system argument: {}".format(coordinate_systems))

        if visualizer is None:
            visualizer = pin.visualize.MeshcatVisualizer(self.pin_model, self.collision, self.visual,
                                                         copy_models=False,
                                                         data=self.pin_data,
                                                         collision_data=self.collision_data,
                                                         visual_data=self.visual_data)
        elif visualizer.model is not self.pin_model:
            # Copy everything, s.t. robot updates are transferred to the visualizer!
            visualizer.model = self.pin_model
            visualizer.collision_model = self.collision
            visualizer.visual_model = self.visual
            visualizer.data = self.pin_data
            visualizer.collision_data = self.collision_data
            visualizer.visual_data = self.visual_data

        if not hasattr(visualizer, 'viewer'):
            visualizer.initViewer()
        try:
            visualizer.loadViewerModel(rootNodeName=self.name)
        except AttributeError as exe:
            if "'GepettoVisualizer'" in str(exe):
                print(exe)
                raise ConnectionError("You might have forgotten to start gepetto (gepetto-gui)")
            else:
                raise exe

        visualizer.display(self.configuration)
        if coordinate_systems is not None:
            if coordinate_systems == 'tcp':
                visualizer.viewer['tcp'].set_object(drawable_coordinate_system(self.fk(kind='tcp'), .3))
            else:
                names = self.joints if coordinate_systems == 'joints' else (fr.name for fr in self.pin_model.frames)
                for transform, name in zip(self.fk(kind=coordinate_systems), names):
                    visualizer.viewer[name].set_object(drawable_coordinate_system(transform, .3))

        return visualizer

    def plot_self_collisions(self,
                             visualizer: pin.visualize.MeshcatVisualizer = None) -> pin.visualize.MeshcatVisualizer:
        """
        Computes all collisions (of pre-defined collision pairs) and visualizes them in a different color for every
        collision pair in contact.
        :param visualizer: A pinnocchio meshcat visualizer (only tested for this one)
        """
        # Compute collisions already updates geometry placement
        pin.computeCollisions(self.pin_model, self.pin_data, self.collision, self.collision_data, self.configuration, False)

        collides = list()
        for k in range(len(self.collision.collisionPairs)):
            cr = self.collision_data.collisionResults[k]
            first, second = map(int, self.collision_pairs[k])
            if cr.isCollision():
                collides.append([self.collision.geometryObjects[first], self.collision.geometryObjects[second]])
                print("collision pair detected:", first, ",", second, "- collision:",
                      "Yes" if cr.isCollision() else "No")

        if visualizer is None:
            visualizer = pin.visualize.MeshcatVisualizer(self.pin_model, self.collision, self.visual,
                                                         copy_models=False,
                                                         data=self.pin_data,
                                                         collision_data=self.collision_data,
                                                         visual_data=self.visual_data)
        elif visualizer.model != self.pin_model:
            visualizer.model = self.pin_model
            visualizer.collision_model = self.collision
            visualizer.visual_model = self.visual
            visualizer.rebuildData()

        if not hasattr(visualizer, 'viewer'):
            visualizer.initViewer()
        visualizer.loadViewerModel(rootNodeName=self.name)

        # red = np.array([1, 0.2, 0.15, 1])
        collision_colors = [np.hstack([np.array(clr), np.ones(1)]) for clr in colors.BASE_COLORS.values()]
        visualizer.displayCollisions(True)
        visualizer.displayVisuals(False)
        visualizer.display(self.configuration)
        for i, cp in enumerate(collides):
            clr = collision_colors[i % len(collision_colors)]
            cp[0].meshColor = clr
            cp[1].meshColor = clr
            visualizer.loadViewerGeometryObject(cp[0], pin.GeometryType.COLLISION)
            visualizer.loadViewerGeometryObject(cp[1], pin.GeometryType.COLLISION)

        return visualizer

    def visualize(self,
                  visualizer: pin.visualize.BaseVisualizer = None,
                  coordinate_systems: Union[str, None] = None
                  ) -> pin.visualize.MeshcatVisualizer:
        """An alias for plot"""
        return self.plot(visualizer, coordinate_systems)
