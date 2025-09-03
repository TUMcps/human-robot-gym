"""A wrapper to convert actions from a position deltas to a joint angle deltas.

This wrapper enables the use of cartesian position actions (fixed orientation),
while employing the failsafe control pipeline for safe online reinforcement learning.

Author: Rafael Cabral
"""

import numpy as np
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper


class IKWayPointsDeltaWrapper(IKPositionDeltaWrapper):
    """Redifine action for cartesian position control.

    Maps a delta position action to a delta joint angle action
    through inverse kinematics using pybullet.

    Instead of a single action, takes in a sequence of waypoints to reach in Cartesian space.
    The waypoints are executed sequentially in the step function.
    """

    def __init__(
        self,
        n_waypoints: int,
        flat_action_space: bool = True,
        **kwargs,
    ):  # noqa: D107
        """Initialize the position delta wrapper.

        Args:
            n_waypoints (int): Number of waypoints to execute in each step.
            flat_action_space (bool): If true, expects actions of shape [n_waypoints * action_dim], otherwise expects
                shape [n_waypoints, action_dim].
        """
        super().__init__(**kwargs)
        self.n_waypoints = n_waypoints
        self.full_action_dim = self.control_dim + self.gripper_action_dim
        self.full_joint_action_dim = self.num_joints + self.gripper_action_dim
        self.flat_action_space = flat_action_space

    @property
    def action_spec(self):
        """Override the action space to be cartesian position delta.

        Returns:
            2-tuple:
                - (np.array) minimum (low) action values
                - (np.array) maximum (high) action values
        """
        return (np.tile(self.action_lb, self.n_waypoints), np.tile(self.action_ub, self.n_waypoints))

    def step(self, action):
        """Transform and apply the action to the environment.

        Transform the action from cartesian position delta to joint position delta,
        append gripper action (if present),
        and apply action to the environment.
        """
        if not self.flat_action_space:
            action = action.flatten()
        # Clip action to action space
        action = self.scale_action(action)
        action = np.clip(action, self.action_spec[0], self.action_spec[1])

        q_current = self.robot.part_controllers["right"].joint_pos

        joint_waypoints = np.zeros((self.n_waypoints * self.full_joint_action_dim,))
        # Get current joint positions
        for i in range(self.n_waypoints):
            waypoint_action = self.get_i_th_action(action, i)
            target_position, target_orientation = self.compute_goal_pose(waypoint_action, q_current)
            # inverse kinematics, selectively damped least squares
            joint_poses = self.inverse_kinematics_step(target_position, target_orientation)
            q_goal = np.array(joint_poses[: self.num_joints])
            # joint absolute action
            q_action = q_goal
            # handle gripper action
            if len(waypoint_action) > self.control_dim:
                q_action = np.append(q_action, waypoint_action[self.control_dim:])
            joint_waypoints[i*self.full_joint_action_dim:(i+1)*self.full_joint_action_dim] = q_action
            # update current joint positions
            q_current = q_goal

        return self.env.step(joint_waypoints)

    def scale_action(self, action):
        """
        Clips @action to be within self.input_min and self.input_max, and then re-scale the values to be within
        the range self.output_min and self.output_max

        Args:
            action (Iterable): Actions to scale

        Returns:
            np.array: Re-scaled action
        """
        for i in range(self.n_waypoints):
            waypoint_action = self.get_i_th_action(action, i)
            action[i*self.full_action_dim:(i+1)*self.full_action_dim] = super().scale_action(waypoint_action)
        return action

    def get_i_th_action(self, action, i):
        """Get the i-th waypoint action from the full action.

        Args:
            action (np.array): Full action of shape [n_waypoints * action_dim].
            i (int): Index of the waypoint to get.

        Returns:
            np.array: The i-th waypoint action of shape [action_dim].
        """
        return action[i * self.full_action_dim:(i + 1) * self.full_action_dim]
