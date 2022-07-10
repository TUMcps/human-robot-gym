"""A wrapper to convert actions from a position deltas to a joint angle deltas.

This wrapper enables the use of cartesian position actions with fixed orientation,
while employing the failsafe control pipeline for safe online reinforcement learning.

Author: Rafael Cabral
"""

import numpy as np
import pybullet as p
from gym.core import Wrapper
from gym.spaces import Box

class IKPositionDeltaWrapper(Wrapper):
    """Redifine action for cartesian position control.

    Maps a delta position action to a delta joint angle action
    through inverse kinematics using pybullet.
    note: Uses the gym wrapper instead of the robosuite wrapper, 
        given that I did not find out, how to redifine the action space with the latter.
    """

    def __init__(
        self, 
        env, 
        urdf_file, 
        x_output_max=0.2,  # max l1 distance of cartesian delta
        x_position_limits=None,  # 2d np array (2, n) [min, max]
        residual_threshold=1e-3,
        max_iter=50,
        **kwargs
    ):  # noqa: D107
        """Initialize the position delta wrapper.

        Args:
            env (gym.env): The gym environment
            urdf_file (string): path to robot urdf file, used for inverse kinematics.
                Should not start with fixed joints to work as expected.
            x_output_max (double): limits the end effector velocity. 
                Maximum L1 distance of cartesian position delta.
            x_position_limits (2D numpy array (2, 3)): 
                if not None, limits the target cartesian positions to [[mins], [maxs]].
            residual_threshold (double): 
                Refine the IK solution until the distance 
                between target and actual end effector position is below residual_threshold, 
                or until max_iter is reached.
            max_iter (int): maximum number of iterations in IK solution.
        """
        super().__init__(env)
        self.env = env
        self.urdf_file = urdf_file

        self.base_position = env.robots[0].base_pos
        self.base_orientation = env.robots[0].base_ori
        self.num_joints = len(env.robots[0].init_qpos)
        self.end_effector_index = self.num_joints

        # pybullet for inverse kinematics
        self.p_client_id = p.connect(p.DIRECT)
        self.p_robot_id = p.loadURDF(
            fileName=self.urdf_file,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
        )
        self.residual_threshold = residual_threshold
        self.max_iter = max_iter

        # get and maintain initial orientation
        init_q = env.robots[0].init_qpos
        for i in range(self.num_joints):
            p.resetJointState(self.p_robot_id, i, init_q[i])
        ee_state = p.getLinkState(self.p_robot_id, self.end_effector_index)
        init_ori = ee_state[5]  # worldLinkFrameOrientation as quaternion [x y z w]

        # Control dimension
        self.control_dim = 3
        self.target_orientation = init_ori

        # Redefining action space
        self.gripper_action_dim = env.robots[0].gripper.dof
        ub = np.ones(self.control_dim + self.gripper_action_dim)
        self.action_space = Box(-ub, ub)

        # Cartesian action limits and x
        self.x_output_max = x_output_max
        self.x_position_limits = x_position_limits


    def step(self, action):
        """Transform and apply the action to the environment.

        Transform the action from cartesian position delta to joint position delta,
        append gripper action (if present),
        and apply action to the environment.
        """
        ws_action = np.zeros(self.control_dim)
        ws_action[: self.control_dim] = action[: self.control_dim]

        # get pybullet end-effector position
        q_current  = self.env.robots[0].controller.joint_pos
        for i, val in enumerate(q_current):
            p.resetJointState(self.p_robot_id, i, val)
        ee_state = p.getLinkState(self.p_robot_id, self.end_effector_index)
        ee_pos = ee_state[4]  # worldLinkFramePosition [x y z]

        # scale action from [-1, 1] to output range
        ws_action *= self.x_output_max

        # calculate and clip target position
        target_position = ee_pos + ws_action
        if self.x_position_limits:
            target_position = np.clip(
                target_position, self.x_position_limits[0], self.x_position_limits[1]
            )

        # inverse kinematics, selectively damped least squares
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.p_robot_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=target_position,
            targetOrientation=self.target_orientation,
            solver=p.IK_SDLS,
            residualThreshold=self.residual_threshold,
            maxNumIterations=self.max_iter,
        )
        q_goal = np.array(joint_poses[: self.num_joints])

        # joint delta action
        q_action = q_goal - q_current

        # handle gripper action
        if len(action) > self.control_dim:
            q_action = np.append(q_action, action[self.control_dim:])

        next_obs, reward, done, info = self.env.step(q_action)
        return next_obs, reward, done, info
