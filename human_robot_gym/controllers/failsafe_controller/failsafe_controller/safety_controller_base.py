"""Abstract base class for safety controllers.

This module provides the base class that all safety controllers inherit from.
It contains common functionality for PD control, robot state management, 
and the interface that safety controllers must implement.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2025-09-05 JT Initial implementation
"""

import numpy as np
from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation

from robosuite.controllers.parts.generic.joint_pos import JointPositionController
from robosuite.utils.control_utils import set_goal_position


class SafetyController(JointPositionController, ABC):
    """Abstract base class for safety controllers in human-robot interaction.

    This class provides common functionality for all safety controllers including:
    - PD control with gravity compensation
    - Robot state management
    - Common interface methods
    - Abstract safety filtering that subclasses must implement

    Args:
        sim (MjSim): Simulator instance this controller will pull robot state updates from
        eef_name (str): Name of controlled robot arm's end effector (from robot XML)
        joint_indexes (dict): Each key contains sim reference indexes to relevant robot joint information
        actuator_range (2-tuple of array of float): 2-Tuple (low, high) representing the robot joint actuator range
        init_qpos (list): Initial joint angles
        robot_name (str): Name of the robot
        use_waypoints_action (bool): Whether to use waypoint actions
        n_waypoints (int): Number of waypoints for trajectory planning
        base_pos (list): Robot base position [x, y, z]
        base_orientation (list): Robot base orientation [x, y, z, w]
        shield_type (str): Type of safety shield ("SSM", "PFL", "CBF", "OFF")
        mocap_file (str): Motion capture configuration file
        control_sample_time (float): Control sample time
        **kwargs: Additional controller parameters
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
        shield_type="SSM",
        mocap_file="mujoco_mocap.yaml",
        control_sample_time=0.004,
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
        qpos_limits=None,
        interpolator=None,
        part_name=None,
        naming_prefix="",
        lite_physics=True,
        **kwargs,
    ):
        """Initialize the safety controller base class."""
        # Initialize parent controller
        super().__init__(
            sim,
            joint_indexes,
            actuator_range,
            ref_name=eef_name,
            part_name=part_name,
            naming_prefix=naming_prefix,
            lite_physics=lite_physics,
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
            qpos_limits=qpos_limits,
            interpolator=interpolator,
        )

        # Common safety controller parameters
        self.use_waypoints_action = use_waypoints_action
        self.n_waypoints = n_waypoints
        self.shield_type = shield_type
        self.control_sample_time = control_sample_time
        
        if self.use_waypoints_action:
            self.control_dim = len(joint_indexes["joints"]) * self.n_waypoints
            self.input_min = np.tile(self.input_min, n_waypoints)
            self.output_min = np.tile(self.output_min, n_waypoints)
            self.input_max = np.tile(self.input_max, n_waypoints)
            self.output_max = np.tile(self.output_max, n_waypoints)

        # Store robot info
        self.eef_name = eef_name
        self.robot_name = robot_name
        self.init_qpos = init_qpos
        self.mocap_file = mocap_file

        # Robot base transformation
        rot = Rotation.from_quat([
            base_orientation[0], base_orientation[1], 
            base_orientation[2], base_orientation[3]
        ])
        self.base_pos = np.array(base_pos)
        self.base_orientation = rot.as_euler("XYZ")

        # Safety state tracking
        self.safety_intervention = False

    def set_goal(self, action, set_qpos=None):
        """Set goal based on input action with safety filtering.

        This method handles both waypoint and regular actions, applies safety filtering,
        and sets the final safe goal position.

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
            # Handle waypoint trajectory
            self._handle_waypoint_action(action)
            return
            
        # Normal operation
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
        
        # Apply safety filtering (implemented by subclasses)
        safe_qpos = self._apply_safety_filter(desired_qpos)
        self.goal_qpos = safe_qpos

    def run_controller(self):
        """Calculate the torques required to reach the desired setpoint safely.

        Uses PD control with gravity compensation and safety-filtered goals.

        Returns:
            np.array: Command torques
        """
        # Make sure goal has been set
        if self.goal_qpos is None:
            self.set_goal(np.zeros(self.control_dim))

        # Update joint states
        self.joint_pos = np.array(self.sim.data.qpos[self.qpos_index])
        self.joint_vel = np.array(self.sim.data.qvel[self.qvel_index])

        # Get desired motion from safety controller
        desired_qpos, desired_qvel, desired_qacc = self._get_desired_motion()

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

    def set_human_measurement(self, human_measurement, time):
        """Set the human measurement for safety constraint computation.

        Args:
            human_measurement (list): List of human joint positions
            time (float): Time of the measurement
        """
        # Base implementation - subclasses can override if needed
        pass

    def get_safety(self):
        """Return if the safety controller intervened in this step.

        Returns:
            bool: True if safe (no intervention), False if unsafe (intervention occurred)
        """
        return not self.safety_intervention

    def get_robot_capsules(self):
        """Return robot capsules for visualization.
        
        Returns:
            list: List of robot capsules (empty by default)
        """
        return []

    def get_human_capsules(self):
        """Return human capsules for visualization.
        
        Returns:
            list: List of human capsules (empty by default)
        """
        return []

    # Abstract methods that subclasses must implement

    @abstractmethod
    def _apply_safety_filter(self, desired_qpos):
        """Apply safety filtering to the desired joint positions.
        
        Args:
            desired_qpos (np.ndarray): Desired joint positions
            
        Returns:
            np.ndarray: Safe joint positions
        """
        pass

    @abstractmethod
    def _handle_waypoint_action(self, action):
        """Handle waypoint-based actions.
        
        Args:
            action (np.ndarray): Waypoint action
        """
        pass

    @abstractmethod
    def _get_desired_motion(self):
        """Get the desired motion (position, velocity, acceleration).
        
        Returns:
            tuple: (desired_qpos, desired_qvel, desired_qacc)
        """
        pass

    @abstractmethod
    def reset(self, base_pos=[0.0, 0.0, 0.0], base_orientation=[0.0, 0.0, 0.0, 1.0], shield_type="SSM"):
        """Reset the safety controller.

        Args:
            base_pos (list): Robot base position [x, y, z]
            base_orientation (list): Robot base orientation [x, y, z, w]  
            shield_type (str): Shield type
        """
        pass

    @property
    @abstractmethod
    def name(self):
        """Return controller name."""
        pass