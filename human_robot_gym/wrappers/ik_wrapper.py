"""A wrapper to convert a position delta to a joint angle delta.

Author: Rafael Cabral
WIP, currently working for position
"""

import numpy as np
from gym.spaces import Box
import gym.core
import pybullet as p

from human_robot_gym.utils.mjcf_utils import file_path_completion


def clip_and_scale(a, in_min, in_max, out_min, out_max):
    """Scale_action in robosuite base_controller, but with custom arguments.

    Clips a to be within in_min and in_max, and then re-scale the values to be within
    the range out_min and out_max
    Args:
        a (np array): array to scale
    Returns:
        np.array: Re-scaled action
    """
    scale = abs(out_max - out_min) / abs(in_max - in_min)
    a_out_transform = (out_max + out_min) / 2.0
    a_in_transform = (in_max + in_min) / 2.0
    a = np.clip(a, in_min, in_max)
    transformed_action = (a - a_in_transform) * scale + a_out_transform
    return transformed_action


class IKWrapper(gym.core.Wrapper):
    """Redifine action to position or pose control.

    Map a delta position (pose) desired action to a delta joint angle action
    through inverse kinematics using pybullet.

    Args:
        env: gym environment to wrap
    """

    def __init__(
        self,
        env,
        reward_scale=1.0,
        x_input_max=1,  # input from policy
        x_input_min=-1,
        x_output_max=0.1,  # max l1 distance of cartesian delta
        x_output_min=-0.1,
        x_position_limits=None,  # 2d np array (2, n) [min, max]
        control_ori=False,  # not implemented yet
        num_joints=6,
        robot_end_effector_index=6,  # schunk in adapted urdf: {5: lastjoint, 6: eefcenter, 7: eefend}
        residual_threshold=1e-3,
        max_iter=50,
        **kwargs
    ):  # noqa: D107
        super().__init__(env)
        self._reward_scale = reward_scale
        self.env = env

        if not env.robot_names[0] == "Schunk":
            raise NotImplementedError
        self.urdf_file = file_path_completion(
            "models/assets/robots/schunk/robot_pybullet.urdf"
        )

        self.base_position = env.robots[0].base_pos
        self.base_orientation = env.robots[0].base_ori

        # pybullet for inverse kinematics
        self.num_joints = num_joints
        self.robot_end_effector_index = robot_end_effector_index
        self.clid = p.connect(p.DIRECT)
        self.robot_id = p.loadURDF(
            fileName=self.urdf_file,
            basePosition=self.base_position,
            baseOrientation=self.base_orientation,
        )
        self.residual_threshold = residual_threshold
        self.max_iter = max_iter

        # initial orientation to maintain
        init_q = env.robots[0].init_qpos
        for i in range(num_joints):
            p.resetJointState(self.robot_id, i, init_q[i])
        ls = p.getLinkState(self.robot_id, self.robot_end_effector_index)
        init_ori = ls[5]

        # Determine whether pos ori or just pos
        self.use_ori = control_ori

        # Control dimension
        if self.use_ori:
            self.control_dim = 6
            self.name_suffix = "POSE"
            self.target_orientation = None
            raise NotImplementedError
        else:
            self.control_dim = 3
            self.name_suffix = "POSITION"
            self.target_orientation = init_ori

        # Redefining action space
        ub = np.ones(self.control_dim + 1)
        self.action_space = Box(x_input_min * ub, x_input_max * ub)

        # Cartesian action limits and x
        self.x_input_max = np.ones(self.control_dim) * x_input_max
        self.x_input_min = np.ones(self.control_dim) * x_input_min
        self.x_output_max = np.ones(self.control_dim) * x_output_max
        self.x_output_min = np.ones(self.control_dim) * x_output_min
        self.x_position_limits = x_position_limits

    def dx_to_dq(self, action):
        """Preprocess work space action and perform inverse kinematics."""
        # Update state
        self.env.robots[0].controller.update(
            force=True
        )  # for current self.ee_pos and self.joint_pos

        # scale_action
        scaled_action = clip_and_scale(
            action,
            self.x_input_min,
            self.x_input_max,
            self.x_output_min,
            self.x_output_max,
        )

        ee_pos = self.env.robots[0].controller.ee_pos
        target_position = ee_pos + scaled_action
        if self.x_position_limits:
            target_position = np.clip(
                target_position, self.x_position_limits[0], self.x_position_limits[1]
            )

        # diff IK
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.robot_end_effector_index,
            targetPosition=target_position,
            targetOrientation=self.target_orientation,
            residualThreshold=self.residual_threshold,
            maxNumIterations=self.max_iter,
        )

        # update pybullet model
        for i, val in enumerate(joint_poses[: self.num_joints]):
            p.resetJointState(self.robot_id, i, val)

        # calculate joint delta action for FailsafeController
        q_goal = np.array(joint_poses[: self.num_joints])
        q_action = q_goal - self.env.robots[0].controller.joint_pos
        return q_action

    def step(self, action):
        """Apply action to the environment."""
        gripper_action = action[-1]
        robot_ws_action = np.zeros(self.control_dim)
        robot_ws_action[: self.control_dim] = action[: self.control_dim]

        # inverse kinematics using pybullet
        q_action = self.dx_to_dq(robot_ws_action)

        # linear mappings, simpler approaches for higher policy rates
        # J = self.env.robots[0].controller.J_full
        # q_action = J.T@robot_ws_action # jacobian transpose control
        # q_action = np.linalg.pinv(J, rcond=1e-5)@robot_ws_action # jacobian inverse control

        new_act = np.append(q_action, gripper_action)

        wrapped_step = self.env.step(new_act)
        next_obs, reward, done, info = wrapped_step
        return next_obs, reward * self._reward_scale, done, info

    def __str__(self):
        """Return env as string."""
        return "IK: %s" % self.env
