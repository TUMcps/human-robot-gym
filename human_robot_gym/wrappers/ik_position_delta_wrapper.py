"""A wrapper to convert actions from a position deltas to a joint angle deltas.

This wrapper enables the use of cartesian position actions (fixed orientation),
while employing the failsafe control pipeline for safe online reinforcement learning.

Author: Rafael Cabral
"""

from typing import Optional
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from robosuite.wrappers import Wrapper
from robosuite.environments.base import MujocoEnv
import robosuite.utils.transform_utils as T


class IKPositionDeltaWrapper(Wrapper):
    """Redifine action for cartesian position control.

    Maps a delta position action to a delta joint angle action
    through inverse kinematics using pybullet.
    """

    def __init__(
        self,
        env: MujocoEnv,
        urdf_file: str,
        action_limits: np.ndarray = np.array([[-0.15, -0.15, -0.15], [0.15, 0.15, 0.15]]),
        x_output_max: float = 1.0,
        x_position_limits: Optional[np.ndarray] = None,
        residual_threshold: float = 1e-3,
        max_iter: int = 50,
        use_orientation: bool = False,
        goal_update_mode: str = "achieved",
        input_ref_frame: str = "world",
        action_scale: Optional[float] = None,
        input_min: float = -1.0,
        input_max: float = 1.0,
        **kwargs,
    ):  # noqa: D107
        """Initialize the position delta wrapper.

        Args:
            env (gym.env): The gym environment
            urdf_file (string): path to robot urdf file, used for inverse kinematics.
                Should not start with fixed joints to work as expected.
            action_limits (2D numpy array (2, X)): limits the action to [[mins], [maxs]].
                X is 3 if use_orientation is False, else 6.
            x_output_max (double): limits the end effector velocity.
                Maximum L1 distance of cartesian position delta.
                If this value is not 1, the action does not represent the delta position anymore.
            x_position_limits (2D numpy array (2, 3)):
                if not None, limits the target cartesian positions to [[mins], [maxs]].
            residual_threshold (double):
                Refine the IK solution until the distance
                between target and actual end effector position is below residual_threshold,
                or until max_iter is reached.
            max_iter (int): maximum number of iterations in IK solution.
            use_orientation (bool): Whether to use orientation control.
            goal_update_mode (str): How to update goals - "achieved" or "desired".
            input_ref_frame (str): Reference frame for actions - "world" or "base".
        """
        super().__init__(env)
        self.robot = self.unwrapped.robots[0]
        self.urdf_file = urdf_file
        # unwrapped_env = env.unwrapped
        self.base_position = self.robot.base_pos
        self.base_orientation = self.robot.base_ori
        self.num_joints = len(self.robot.arm_joint_indexes)
        self.end_effector_index = self.num_joints

        # pybullet for inverse kinematics
        self.p_client_id = p.connect(p.DIRECT)

        # Convert base orientation from rotation matrix to quaternion for PyBullet
        if self.base_orientation.shape == (3, 3):
            # Convert 3x3 rotation matrix to quaternion [x, y, z, w]
            rotation = Rotation.from_matrix(self.base_orientation)
            base_orientation_quat = rotation.as_quat()  # Returns [x, y, z, w]
        else:
            # Assume it's already in the correct format
            base_orientation_quat = self.base_orientation

        self.p_robot_id = p.loadURDF(
            fileName=self.urdf_file,
            basePosition=self.base_position,
            baseOrientation=base_orientation_quat,
        )
        self.residual_threshold = residual_threshold
        self.max_iter = max_iter
        self.use_orientation = use_orientation
        self.goal_update_mode = goal_update_mode
        self.input_ref_frame = input_ref_frame

        # get and maintain initial orientation
        init_q = self.robot.init_qpos
        for i in range(self.num_joints):
            p.resetJointState(self.p_robot_id, i, init_q[i])
        ee_state = p.getLinkState(self.p_robot_id, self.end_effector_index)
        init_ori = ee_state[5]  # worldLinkFrameOrientation as quaternion [x y z w]

        # Control dimension
        self.control_dim = 6 if self.use_orientation else 3
        self.target_orientation = init_ori

        # Initialize goals for orientation tracking
        self.goal_ori = None

        # Store initial end-effector pose for reference
        ee_state = p.getLinkState(self.p_robot_id, self.end_effector_index)
        init_pos = ee_state[4]  # worldLinkFramePosition [x y z]
        init_ori_quat = ee_state[5]  # worldLinkFrameOrientation as quaternion [x y z w]

        # Convert quaternion to rotation matrix for consistency with robosuite
        init_ori_mat = T.quat2mat(init_ori_quat)
        self.ref_ori_mat = init_ori_mat
        self.ref_pos = np.array(init_pos)

        # Redefining action space
        # Calculate gripper DOF: total action dim - robot DOF
        if len(self.robot.gripper) > 0:
            self.gripper_action_dim = self.robot.gripper[self.robot.arms[0]].dof
        else:
            self.gripper_action_dim = 0
        self.action_lb = np.append(action_limits[0], -np.ones(self.gripper_action_dim))
        self.action_ub = np.append(action_limits[1], np.ones(self.gripper_action_dim))

        self.action_scale = action_scale
        self.input_min = input_min
        self.input_max = input_max

        # Cartesian action limits and x
        self.x_output_max = x_output_max
        self.x_position_limits = x_position_limits

    @property
    def action_spec(self):
        """Override the action space to be cartesian position delta.

        Returns:
            2-tuple:
                - (np.array) minimum (low) action values
                - (np.array) maximum (high) action values
        """
        return (self.action_lb, self.action_ub)

    def step(self, action):
        """Transform and apply the action to the environment.

        Transform the action from cartesian position delta to joint position delta,
        append gripper action (if present),
        and apply action to the environment.
        """
        # Clip action to action space
        action = self.scale_action(action)
        action = np.clip(action, self.action_spec[0], self.action_spec[1])

        # Get current joint positions
        q_current = self.robot.part_controllers["right"].joint_pos

        target_position, target_orientation = self.compute_goal_pose(action, q_current)

        # inverse kinematics, selectively damped least squares
        joint_poses = self.inverse_kinematics_step(target_position, target_orientation)
        q_goal = np.array(joint_poses[: self.num_joints])

        # joint delta action
        q_action = q_goal - q_current

        # handle gripper action
        if len(action) > self.control_dim:
            q_action = np.append(q_action, action[self.control_dim :])

        next_obs, reward, done, info = super().step(q_action)
        return next_obs, reward, done, info

    def compute_goal_pose(self, delta, q_current):
        """
        Compute new goal pose, given a delta to update.

        Args:
            delta (np.array): Desired relative change in position [dx, dy, dz, (optional) dax, day, daz]
            q_current (np.array): Current joint positions

        Returns:
            target_position (np.array): updated goal position [x, y, z]
        target_orientation (np.array): updated goal orientation as quaternion [x, y, z, w]
        """
        # Reset pybullet to current joint positions
        for i, val in enumerate(q_current):
            p.resetJointState(self.p_robot_id, i, val)
        # get pybullet EEF state
        ee_state = p.getLinkState(self.p_robot_id, self.end_effector_index)
        ee_pos = ee_state[4]  # worldLinkFramePosition [x y z]
        ee_ori_quat = ee_state[5]  # worldLinkFrameOrientation as quaternion [x y z w]

        # Position
        position_delta = delta[:3]
        # scale action from [-1, 1] to output range
        position_delta *= self.x_output_max
        # calculate and clip target position
        target_position = ee_pos + position_delta
        if self.x_position_limits:
            target_position = np.clip(target_position, self.x_position_limits[0], self.x_position_limits[1])

        # Calculate target orientation if using orientation control
        target_orientation = self.target_orientation
        if self.use_orientation:
            # Get orientation delta from action
            ori_delta = delta[3:6] * self.x_output_max  # scale orientation action
            # Compute goal orientation using delta
            target_orientation = self.compute_goal_ori(ori_delta, ee_ori_quat)

        return target_position, target_orientation

    def inverse_kinematics_step(self, target_position, target_orientation):
        """Apply inverse kinematics to reach a target position and orientation.

        Args:
            target_position (np.array): Desired target position [x, y, z]
            target_orientation (np.array): Desired target orientation as quaternion [x, y, z, w]

        Returns:
            desired joint positions to reach the target pose.
        """
        return p.calculateInverseKinematics(
            bodyUniqueId=self.p_robot_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            residualThreshold=self.residual_threshold,
            maxNumIterations=self.max_iter,
        )

    def compute_goal_ori(self, delta, current_ori_quat):
        """
        Compute new goal orientation, given a delta to update.

        Args:
            delta (np.array): Desired relative change in orientation, in axis-angle form [ax, ay, az]

        Returns:
            np.array: updated goal orientation as quaternion [x, y, z, w] for PyBullet
        """
        if self.goal_ori is None:
            # Initialize goal orientation to current orientation
            if self.input_ref_frame == "world":
                self.goal_ori = T.quat2mat(current_ori_quat)
            else:  # base frame not fully implemented yet
                self.goal_ori = T.quat2mat(current_ori_quat)

        # Convert axis-angle delta to rotation matrix
        quat_delta = T.axisangle2quat(delta)
        rotation_mat_delta = T.quat2mat(quat_delta)

        if self.goal_update_mode == "desired":
            # Update goal orientation relative to current desired goal
            new_goal_ori = np.dot(rotation_mat_delta, self.goal_ori)
        elif self.goal_update_mode == "achieved":
            # Update goal orientation relative to current achieved orientation
            current_ori_mat = T.quat2mat(current_ori_quat)
            new_goal_ori = np.dot(rotation_mat_delta, current_ori_mat)
        else:
            raise ValueError(f"Invalid goal_update_mode: {self.goal_update_mode}")

        # Store the updated goal orientation
        self.goal_ori = new_goal_ori

        # Convert back to quaternion for PyBullet
        goal_quat = T.mat2quat(new_goal_ori)
        return goal_quat

    def scale_action(self, action):
        """
        Clips @action to be within self.input_min and self.input_max, and then re-scale the values to be within
        the range self.output_min and self.output_max

        Args:
            action (Iterable): Actions to scale

        Returns:
            np.array: Re-scaled action
        """

        if self.action_scale is None:
            self.action_scale = abs(self.action_ub - self.action_lb) / abs(self.input_max - self.input_min)
            self.action_output_transform = (self.action_ub + self.action_lb) / 2.0
            self.action_input_transform = (self.input_max + self.input_min) / 2.0
        action = np.clip(action, self.input_min, self.input_max)
        transformed_action = (action - self.action_input_transform) * self.action_scale + self.action_output_transform

        return transformed_action
