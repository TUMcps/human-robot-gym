"""CBF-based safety controller for human-robot interaction.

This controller implements Control Barrier Functions (CBFs) for ensuring safe robot operation
in the vicinity of humans, as an alternative to the SARA shield approach.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2025-09-05 JT Initial implementation
"""

import numpy as np
import os
import casadi as cs
from scipy.spatial.transform import Rotation

from robosuite.controllers.parts.generic.joint_pos import JointPositionController
from robosuite.utils.control_utils import set_goal_position

from human_robot_gym.controllers.failsafe_controller.failsafe_controller.failsafe_controller import FailsafeController
from human_robot_gym.controllers.failsafe_controller.cbf_utils.class_k import ClassKFunction, MonotonicFunction
from human_robot_gym.controllers.failsafe_controller.cbf_utils.kinematics import CBFKinematics


class CBFFailsafeController(FailsafeController):
    """CBF-based failsafe controller for safe human-robot interaction.

    This controller uses Control Barrier Functions to ensure safety by solving
    a quadratic program that minimizes deviation from the desired control input
    while satisfying safety constraints.

    Args:
        sim (MjSim): Simulator instance
        eef_name (str): Name of controlled robot arm's end effector
        joint_indexes (dict): Dictionary containing joint index information
        actuator_range (2-tuple): Robot joint actuator range (low, high)
        init_qpos (list): Initial joint angles
        robot_name (str): Name of the robot
        use_waypoints_action (bool): Whether to use waypoint actions
        n_waypoints (int): Number of waypoints for trajectory planning
        base_pos (list): Robot base position [x, y, z]
        base_orientation (list): Robot base orientation [x, y, z, w]
        shield_type (str): Type of safety ("CBF" for CBF-based safety)
        min_distance (float): Minimum allowed distance to obstacles
        class_k_type (str): Type of Class K function ('linear', 'quadratic', 'exponential')
        class_k_scale (float): Scaling factor for Class K function
        opti_solver (str): Optimization solver ('qpoases', 'ipopt')
        **kwargs: Additional arguments
    """

    def __init__(
        self,
        sim,
        eef_name,
        joint_indexes,
        actuator_range,
        init_qpos,
        robot_name,
        use_waypoints_action: bool = False,
        n_waypoints: int = 1,
        base_pos=[0.0, 0.0, 0.0],
        base_orientation=[0.0, 0.0, 0.0, 1.0],
        shield_type="CBF",
        min_distance=0.2,
        class_k_type='linear',
        class_k_scale=1.0,
        opti_solver='qpoases',
        input_max=1,
        input_min=-1,
        output_max=0.05,
        output_min=-0.05,
        kp=50,
        damping_ratio=1,
        impedance_mode="fixed",
        kp_limits=(0, 300),
        damping_ratio_limits=(0, 100),
        policy_freq=20,
        control_sample_time=0.004,
        qpos_limits=None,
        interpolator=None,
        part_name=None,
        naming_prefix="",
        lite_physics=True,
        **kwargs,
    ):
        """Initialize CBF failsafe controller."""
        # Store CBF-specific parameters
        self.min_distance = min_distance
        self.class_k_type = class_k_type
        self.class_k_scale = class_k_scale
        self.opti_solver = opti_solver
        
        # Initialize parent FailsafeController
        super().__init__(
            sim=sim,
            eef_name=eef_name,
            joint_indexes=joint_indexes,
            actuator_range=actuator_range,
            init_qpos=init_qpos,
            robot_name=robot_name,
            use_waypoints_action=use_waypoints_action,
            n_waypoints=n_waypoints,
            base_pos=base_pos,
            base_orientation=base_orientation,
            shield_type=shield_type,
            input_max=input_max,
            input_min=input_min,
            output_max=output_max,
            output_min=output_min,
            kp=kp,
            damping_ratio=damping_ratio,
            impedance_mode=impedance_mode,
            kp_limits=kp_limits,
            damping_ratio_limits=damping_ratio_limits,
            policy_freq=policy_freq,
            control_sample_time=control_sample_time,
            qpos_limits=qpos_limits,
            interpolator=interpolator,
            part_name=part_name,
            naming_prefix=naming_prefix,
            lite_physics=lite_physics,
            **kwargs,
        )

        # CBF-specific parameters
        self.use_waypoints_action = use_waypoints_action
        self.n_waypoints = n_waypoints
        self.shield_type = shield_type
        self.min_distance = min_distance
        self.control_sample_time = control_sample_time
        
        if self.use_waypoints_action:
            self.control_dim = len(joint_indexes["joints"]) * self.n_waypoints
            self.input_min = np.tile(self.input_min, n_waypoints)
            self.output_min = np.tile(self.output_min, n_waypoints)
            self.input_max = np.tile(self.input_max, n_waypoints)
            self.output_max = np.tile(self.output_max, n_waypoints)

        # Store eef_name and robot info
        self.eef_name = eef_name
        self.robot_name = robot_name

        # Initialize kinematics
        self.kinematics = CBFKinematics(robot_name)
        
        # Initialize Class K function
        monotonic_pos = MonotonicFunction(class_k_type, class_k_scale)
        monotonic_neg = MonotonicFunction(class_k_type, class_k_scale)
        self.class_k_function = ClassKFunction(monotonic_pos, monotonic_neg)
        
        # CBF constraint tracking
        self.constraint_keys = ['cbf1']  # For now, single constraint
        self.num_cbf_constraints = len(self.constraint_keys)
        
        # Initialize optimization problem
        self._setup_cbf_optimizer(opti_solver)
        
        # Safety state tracking
        self.safety_intervention = False
        self.human_positions = []
        self.robot_position = np.zeros(3)
        
        # Initialize workspace boundaries (simple table)
        rot = Rotation.from_quat([
            base_orientation[0], base_orientation[1], 
            base_orientation[2], base_orientation[3]
        ])
        self.base_pos = np.array(base_pos)
        self.base_orientation = rot.as_euler("XYZ")
        
    def _setup_cbf_optimizer(self, solver):
        """Setup the CBF optimization problem.
        
        Args:
            solver (str): Name of the solver to use
        """
        self.solver = solver
        self.opts = {'printLevel': 'low'} if solver == 'qpoases' else {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0
        }
        
        # Create optimization problem
        self.opti = cs.Opti('conic' if solver == 'qpoases' else None)
        self.opti.solver(solver, self.opts)
        
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

    def set_goal(self, action, set_qpos=None):
        """Set goal based on input action with CBF safety filtering.

        Args:
            action (np.ndarray): Desired relative joint position goal state
            set_qpos (np.ndarray): If set, overrides action with absolute joint positions
        """
        # Update state
        self.update()

        # Handle waypoint actions
        if self.use_waypoints_action:
            if self.position_limits is not None:
                action = np.clip(
                    action,
                    np.tile(self.position_limits[0], self.n_waypoints),
                    np.tile(self.position_limits[1], self.n_waypoints)
                )
            # For CBF implementation, we'll use the first waypoint as immediate goal
            action_2D = np.reshape(action, (self.n_waypoints, int(len(action)/self.n_waypoints)))
            action = action_2D[0]  # Use first waypoint
            
        # Normal operation - apply CBF safety filter
        jnt_dim = len(self.qpos_index)
        
        if len(action) != jnt_dim:
            raise ValueError(f"Action dimension {len(action)} != joint dimension {jnt_dim}")
            
        # Scale the action
        if action is not None:
            scaled_delta = self.scale_action(action)
        else:
            scaled_delta = None

        # Compute desired goal position
        desired_qpos = set_goal_position(
            scaled_delta,
            self.joint_pos,
            position_limit=self.position_limits,
            set_pos=set_qpos,
        )
        
        # Apply CBF safety filtering
        safe_qpos = self._apply_cbf_filter(desired_qpos)
        self.goal_qpos = safe_qpos

    def _apply_cbf_filter(self, desired_qpos):
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

    def _update_cbf_conditions(self):
        """Update CBF constraint conditions based on current robot and human states."""
        # Get current robot end-effector position
        self.robot_position = self.ee_pos
        
        # Compute robot Jacobian (simplified)
        jacobian = self.kinematics.compute_jacobian(self.joint_pos)
        
        # For each constraint (currently just one)
        for i, constraint_key in enumerate(self.constraint_keys):
            if len(self.human_positions) > 0:
                # Use closest human position
                distances = [np.linalg.norm(self.robot_position - hp) for hp in self.human_positions]
                min_idx = np.argmin(distances)
                closest_human = self.human_positions[min_idx]
                
                # Compute barrier function: h(x) = distance - min_distance
                distance = distances[min_idx]
                h_value = distance - self.min_distance
                
                # Compute gradient of barrier function
                if distance > 1e-8:
                    grad_h_pos = (self.robot_position - closest_human) / distance
                    grad_h = np.zeros(6)
                    grad_h[:3] = grad_h_pos
                    
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
                    # Very close or overlapping - set conservative values
                    self.opti.set_value(self.Lfh[i], 0.0)
                    self.opti.set_value(self.Lgh[i, :], np.zeros(len(self.qpos_index)))
                    self.opti.set_value(self.kappa_at_hx[i], 1000.0)  # Large penalty
            else:
                # No human detected - no constraints
                self.opti.set_value(self.Lfh[i], 0.0)
                self.opti.set_value(self.Lgh[i, :], np.zeros(len(self.qpos_index)))
                self.opti.set_value(self.kappa_at_hx[i], 0.0)

    def set_human_measurement(self, human_measurement, time):
        """Set the human measurement for CBF constraint computation.

        Args:
            human_measurement (list): List of human joint positions [x, y, z]
            time (float): Time of the measurement
        """
        if human_measurement:
            # For now, extract a representative human position (e.g., torso or head)
            # In a full implementation, this would process all human joint positions
            if len(human_measurement) > 0 and len(human_measurement[0]) >= 3:
                # Use first position as representative human position
                self.human_positions = [np.array(human_measurement[0][:3])]

    @property
    def ee_pos(self):
        """Get the end-effector position from the simulation."""
        try:
            return self.sim.data.get_site_xpos(self.eef_name)
        except Exception:
            print("Warning: Could not find site {} in the model.".format(self.eef_name))
            return np.zeros(3)

    @property  
    def ee_ori_mat(self):
        """Get the end-effector orientation matrix from the simulation."""
        try:
            return self.sim.data.get_site_xmat(self.eef_name).reshape(3, 3)
        except Exception:
            print("Warning: Could not find site {} in the model.".format(self.eef_name))
            return np.eye(3)

    def run_controller(self):
        """Calculate the torques required to reach the desired setpoint safely.

        Returns:
            np.array: Command torques
        """
        # Make sure goal has been set
        if self.goal_qpos is None:
            self.set_goal(np.zeros(self.control_dim))

        # Update joint states
        self.joint_pos = np.array(self.sim.data.qpos[self.qpos_index])
        self.joint_vel = np.array(self.sim.data.qvel[self.qvel_index])

        # Use the safe goal position computed by CBF
        desired_qpos = self.goal_qpos
        desired_qvel = np.zeros_like(self.joint_vel)  # Assume zero desired velocity
        desired_qacc = np.zeros_like(self.joint_vel)  # Assume zero desired acceleration

        # Compute PID control
        position_error = desired_qpos - self.joint_pos
        vel_error = desired_qvel - self.joint_vel
        
        feedback_torque = (
            np.multiply(np.array(position_error), np.array(self.kp)) + 
            np.multiply(vel_error, self.kd)
        )

        # Add acceleration feedforward and gravity compensation
        feedback_torque = feedback_torque + desired_qacc
        self.torques = (
            np.dot(self.mass_matrix, feedback_torque) + 
            self.torque_compensation
        )

        # Clip torques to safe limits
        self.torques = self.clip_torques(torques=self.torques)
        self.new_update = True

        return self.torques

    def get_safety(self):
        """Return if the CBF controller intervened in this step.

        Returns:
            bool: True if safe (no intervention), False if unsafe (intervention occurred)
        """
        return not self.safety_intervention

    def get_robot_capsules(self):
        """Return robot capsules for visualization (placeholder).
        
        Returns:
            list: Empty list (no capsules for basic CBF implementation)
        """
        return []

    def get_human_capsules(self):
        """Return human capsules for visualization (placeholder).
        
        Returns:
            list: Empty list (no capsules for basic CBF implementation)
        """
        return []

    @property
    def name(self):
        """Return controller name."""
        return "CBF_FAILSAFE"