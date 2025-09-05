"""CBF-based safety controller for human-robot interaction.

This controller implements Control Barrier Functions (CBFs) for ensuring safe robot operation
in the vicinity of humans using optimization-based safety filtering.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2025-09-05 JT Refactored from CBFFailsafeController
"""

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import casadi as cs
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    cs = None

from human_robot_gym.controllers.failsafe_controller.failsafe_controller.safety_controller_base import SafetyController
from human_robot_gym.controllers.failsafe_controller.cbf_utils.class_k import ClassKFunction, MonotonicFunction
from human_robot_gym.controllers.failsafe_controller.cbf_utils.kinematics import CBFKinematics


class CBFSafetyController(SafetyController):
    """CBF-based safety controller for human-robot interaction.

    This controller uses Control Barrier Functions to ensure safety by solving
    a quadratic program that minimizes deviation from the desired control input
    while satisfying safety constraints.

    Additional Args (beyond SafetyController):
        min_distance (float): Minimum allowed distance to obstacles
        class_k_type (str): Type of Class K function ('linear', 'quadratic', 'exponential')
        class_k_scale (float): Scaling factor for Class K function
        opti_solver (str): Optimization solver ('qpoases', 'ipopt')
    """

    def __init__(
        self,
        min_distance=0.2,
        class_k_type='linear',
        class_k_scale=1.0,
        opti_solver='qpoases',
        **kwargs,
    ):
        """Initialize CBF safety controller."""
        # Check CasADi availability
        if not CASADI_AVAILABLE:
            raise ImportError(
                "CasADi is required for CBF safety controller. "
                "Please install it with: pip install casadi "
                "or: pip install -e .[cbf]"
            )

        # Store CBF-specific parameters
        self.min_distance = min_distance
        self.class_k_type = class_k_type
        self.class_k_scale = class_k_scale
        self.opti_solver = opti_solver

        # Initialize parent class
        super().__init__(**kwargs)

        # Initialize kinematics
        self.kinematics = CBFKinematics(self.robot_name)

        # Initialize Class K function
        monotonic_pos = MonotonicFunction(class_k_type, class_k_scale)
        monotonic_neg = MonotonicFunction(class_k_type, class_k_scale)
        self.class_k_function = ClassKFunction(monotonic_pos, monotonic_neg)

        # CBF constraint tracking
        self.constraint_keys = ['cbf1']  # Single constraint for now
        self.num_cbf_constraints = len(self.constraint_keys)

        # Initialize optimization problem
        self._setup_cbf_optimizer()

        # Human position tracking
        self.human_positions = []
        self.robot_position = np.zeros(3)

    def reset(self, base_pos=[0.0, 0.0, 0.0], base_orientation=[0.0, 0.0, 0.0, 1.0], shield_type="CBF"):
        """Reset the CBF controller.

        Args:
            base_pos (list): Robot base position [x, y, z]
            base_orientation (list): Robot base orientation [x, y, z, w]
            shield_type (str): Shield type (should be "CBF")
        """
        # Reset parent
        self.goal_qpos = None
        self.torques = None
        self.new_update = True
        self.sim.forward()
        self.update()
        self.joint_pos = np.array(self.sim.data.qpos[self.qpos_index])
        self.initial_joint = self.joint_pos
        self.initial_ee_pos = self.ee_pos
        self.initial_ee_ori_mat = self.ee_ori_mat

        # Update base transform
        rot = Rotation.from_quat([
            base_orientation[0], base_orientation[1],
            base_orientation[2], base_orientation[3]
        ])
        self.base_pos = np.array(base_pos)
        self.base_orientation = rot.as_euler("XYZ")

        # Reset safety state
        self.safety_intervention = False
        self.human_positions = []

    def set_human_measurement(self, human_measurement, time):
        """Set the human measurement for CBF constraint computation.

        Args:
            human_measurement (list): List of human joint positions [x, y, z]
            time (float): Time of the measurement
        """
        if human_measurement:
            self.human_positions = human_measurement

    def _setup_cbf_optimizer(self):
        """Setup the CBF optimization problem."""
        self.solver = self.opti_solver
        self.opts = {'printLevel': 'low'} if self.solver == 'qpoases' else {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0
        }

        # Create optimization problem
        self.opti = cs.Opti('conic' if self.solver == 'qpoases' else None)
        self.opti.solver(self.solver, self.opts)

        # Decision variable: control input
        n_joints = len(self.qpos_index)
        self.u_opti = self.opti.variable(n_joints, 1)

        # Parameters
        self.Lfh = self.opti.parameter(self.num_cbf_constraints, 1)  # Lie derivative of h
        self.Lgh = self.opti.parameter(self.num_cbf_constraints, n_joints)  # Control-dependent part
        self.u_desired = self.opti.parameter(n_joints, 1)  # Desired control input
        self.kappa_at_hx = self.opti.parameter(self.num_cbf_constraints, 1)  # Class K function values

        # Objective: minimize deviation from desired input
        self.cost = (self.u_desired - self.u_opti).T @ (self.u_desired - self.u_opti)
        self.opti.minimize(self.cost)

        # CBF constraints: Lfh + Lgh * u >= -kappa(h(x))
        for i in range(self.num_cbf_constraints):
            constraint = self.Lfh[i] + self.Lgh[i, :] @ self.u_opti >= -self.kappa_at_hx[i]
            self.opti.subject_to(constraint)

        # Initialize parameters with zeros
        self.opti.set_value(self.Lfh, np.zeros((self.num_cbf_constraints, 1)))
        self.opti.set_value(self.Lgh, np.zeros((self.num_cbf_constraints, n_joints)))
        self.opti.set_value(self.kappa_at_hx, np.zeros((self.num_cbf_constraints, 1)))

    def _update_cbf_conditions(self):
        """Update CBF constraint conditions based on current robot and human states."""
        # Get current robot end-effector position
        self.robot_position = self.ee_pos

        # Compute robot Jacobian
        jacobian = self.kinematics.compute_jacobian(self.joint_pos)

        # For each constraint
        for i, constraint_key in enumerate(self.constraint_keys):
            if len(self.human_positions) > 0:
                # Use closest human position
                distances = [np.linalg.norm(self.robot_position - hp) for hp in self.human_positions]
                min_idx = np.argmin(distances)
                closest_human_joint = self.human_positions[min_idx]

                # Compute distance and barrier function following original CBF implementation
                distance = distances[min_idx]
                grad_h = np.zeros(6)

                if distance < self.min_distance:
                    # Inside the safety zone - use original CBF logic
                    barrier_distance = 0.0
                    grad_h[:3] = (self.robot_position - closest_human_joint) / self.min_distance
                else:
                    # Outside the safety zone - normal gradient computation
                    barrier_distance = distance
                    if distance > 1e-8:
                        grad_h[:3] = (self.robot_position - closest_human_joint) / distance

                # Barrier function: h(x) = barrier_distance - min_distance
                h_value = barrier_distance - self.min_distance

                # Lie derivatives
                Lfh = 0.0  # No autonomous dynamics (qdot = u)
                Lgh = grad_h @ jacobian  # Control-dependent part

                # Class K function value
                kappa_value = self.class_k_function(h_value)

                # Set parameter values
                self.opti.set_value(self.Lfh[i], Lfh)
                self.opti.set_value(self.Lgh[i, :], Lgh)
                self.opti.set_value(self.kappa_at_hx[i], kappa_value)
            else:
                # No human detected - no constraints
                self.opti.set_value(self.Lfh[i], 0.0)
                self.opti.set_value(self.Lgh[i, :], np.zeros(len(self.qpos_index)))
                self.opti.set_value(self.kappa_at_hx[i], 0.0)

    # Abstract method implementations

    def _apply_safety_filter(self, desired_qpos):
        """Apply CBF safety filtering to the desired joint positions.

        Args:
            desired_qpos (np.ndarray): Desired joint positions

        Returns:
            np.ndarray: Safe joint positions
        """
        try:
            # Compute desired joint velocities
            desired_qvel = (desired_qpos - self.joint_pos) / self.control_sample_time

            # Update CBF conditions based on current state
            self._update_cbf_conditions()

            # Set parameters for optimization
            self.opti.set_value(self.u_desired, desired_qvel.reshape(-1, 1))

            # Solve the optimization problem
            sol = self.opti.solve()
            safe_qvel = sol.value(self.u_opti).flatten()

            # Convert back to positions
            safe_qpos = self.joint_pos + safe_qvel * self.control_sample_time

            # Check if CBF intervened
            intervention_threshold = 1e-6
            self.safety_intervention = np.linalg.norm(safe_qvel - desired_qvel) > intervention_threshold

            return safe_qpos

        except Exception as e:
            # If optimization fails, fall back to current position (safe but conservative)
            print(f"CBF optimization failed: {e}. Using current position.")
            self.safety_intervention = True
            return self.joint_pos.copy()

    def _handle_waypoint_action(self, action):
        """Handle waypoint-based actions with CBF safety filtering.

        Args:
            action (np.ndarray): Waypoint action
        """
        # For CBF implementation, we'll use the first waypoint as immediate goal
        action_2D = np.reshape(action, (self.n_waypoints, int(len(action)/self.n_waypoints)))
        action = action_2D[0]  # Use first waypoint

        # Apply safety filtering to the action
        jnt_dim = len(self.qpos_index)
        if len(action) != jnt_dim:
            raise ValueError(f"Action dimension {len(action)} != joint dimension {jnt_dim}")

        # Scale the action
        scaled_delta = self.scale_action(action)
        desired_qpos = self.joint_pos + scaled_delta

        # Apply safety filtering
        safe_qpos = self._apply_safety_filter(desired_qpos)
        self.goal_qpos = safe_qpos

    def _get_desired_motion(self):
        """Get the desired motion (uses the safety-filtered goal).

        Returns:
            tuple: (desired_qpos, desired_qvel, desired_qacc)
        """
        desired_qpos = self.goal_qpos if self.goal_qpos is not None else self.joint_pos
        desired_qvel = np.zeros_like(self.joint_pos)  # Assume zero desired velocity
        desired_qacc = np.zeros_like(self.joint_pos)  # Assume zero desired acceleration

        return desired_qpos, desired_qvel, desired_qacc

    @property
    def name(self):
        """Return controller name."""
        return "CBF_SAFETY"
