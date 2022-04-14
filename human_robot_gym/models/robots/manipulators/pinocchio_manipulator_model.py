from collections import OrderedDict

import numpy as np

import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer

from robosuite.models.robots.manipulators import ManipulatorModel

from human_robot_gym.utils.pinocchio_utils import q_pin

#from robosuite.utils.mjcf_utils import find_elements, string_to_array


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
        ## Setup robot representation in pinocchio. This is used for
        # - Collision checking
        # - Jacobian / Joint velocities
        #robot_assets_folder = find_robot_assets_folder(self.robots[0].name)
        self.pin_model, self.pin_collision_model, self.pin_visual_model = pin.buildModelsFromUrdf(
            urdf_file, 
            package_dirs=package_dirs, 
            geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL])
        self.pin_data = self.pin_model.createData()
        self.pin_local_world_aligned_frame = pin.ReferenceFrame(2)
        ## Visualization only for debugging and testing.
        #self.visualize_pinocchio_robot(self.pin_model, self.pin_collision_model, self.pin_visual_model, q)

    def visualize_pinocchio_robot(self, q):
        """
        Visualize pinocchio robot for debugging purposes.
        This requires to start the gepetto viewer first using `gepetto-gui`.

        Args:
            q (np.array): Joint angles to visualize
        """
        viz = GepettoVisualizer(self.pin_model, self.pin_collision_model, self.pin_visual_model)
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
        # Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame. 
        # The result is accessible through data.J. 
        # This function computes also the forwardKinematics of the model. 
        pin.computeJointJacobians(self.pin_model, self.pin_data, q_pin(q))
        # Check velocity
        v_joints = []
        # +2: 1 for pinocchio base joint, 1 for fake end joint
        for i in range(1, self.dof + 2):
            joint_jacobian = pin.getJointJacobian(self.pin_model, self.pin_data, i, self.pin_local_world_aligned_frame)
            v_joints.append(np.matmul(joint_jacobian, q_pin(dq)))
        return v_joints