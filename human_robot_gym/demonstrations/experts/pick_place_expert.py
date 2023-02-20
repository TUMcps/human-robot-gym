"""This file implements an expert policy for the pick-place task.

The policy does not take the human into consideration,
but only the values defined in the PickPlaceExpertObservation data class.

Author:
    Felix Trost (FT)

Changelog:
    08.02.23 FT File creation
"""
from typing import Any, Dict
from dataclasses import dataclass
import numpy as np
import time

from gym.spaces import Box

from human_robot_gym.utils.ou_noise import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts import Expert


@dataclass
class PickPlaceExpertObservation:
    """Data class to encapsulate all observation values relevant for the expert.

    Args:
        object_gripped (bool): whether both finger pads of the robot
            have contact with the object
        dist_to_next_objective (np.ndarray):
            if the object is gripped:
                vector from object to target
            otherwise:
                vector from robot end effector to object
        robot0_gripper_qpos (np.ndarray):
            joint positions of the two fingers
        robot0_gripper_qvel (np.ndarray):
            joint velocities of the two fingers
    """
    object_gripped: bool
    dist_to_next_objective: np.ndarray
    robot0_gripper_qpos: np.ndarray
    robot0_gripper_qvel: np.ndarray


class PickPlaceExpert(Expert):
    """Expert policy for the pick-place task.

    Does not take the human into account.
    Relevant observation data encapsulated in the PickPlaceExpertObservation data class.
    Behavior:
        -Move above object and open gripper
        -Move to object
        -Close gripper
        -Move above target
        -Move to target
        -Repeat

    Noise can be added to the motion parameters. We use Ornstein-Uhlenbeck (OU) processes
    with asymptotic mean 0 and variance of half the motion action limit as described in the action space.
    The OU discretization time delta reflects the clock time between two calls of the expert policy.
    Formula to obtain motion action parameters:
        motion = expert_policy * signal_to_noise_ratio + noise * (1 - signal_to_noise_ratio)

    Args:
        observation_space (Space): the environment observation space
        action_space (Space): the environment action space
        signal_to_noise_ratio (float): interpolation between
            noise signal (Ornstein-Uhlenbeck process): signal_to_noise_ratio = 0
            and expert policy: signal_to_noise_ratio = 1
        hover_dist (float): vertical distance the expert should reach
            while moving above the next objective
        tan_theta (float): tangent of the opening angle of the cone describing
            points above the next objective the expert should reach
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        signal_to_noise_ratio: float = 1,
        hover_dist: float = 0.2,
        tan_theta: float = 0.5,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space,
        )

        self._motion_action_limit = action_space.high[0]
        self._gripper_action_limit = action_space.high[-1]
        # Minimum difference of both gripper joint positions at which
        # the gripper is considered to be fully opened
        self._gripper_fully_opened_threshold = 0.02
        # Maximum horizontal distance for the expert
        # to be considered at the next objective position
        self._horiz_dist_to_next_objective_threshold = 0.02
        # Maximum vertical distance for the expert to be considered at the next objective position
        # Has to be chosen high enough to respect the safety margin to the table
        # and low enough to ensure the object can be gripped
        self._vert_dist_to_next_objective_threshold = 0.015
        self._tan_theta = tan_theta
        self._hover_dist = hover_dist
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._time = time.time()
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0,
            sigma=self._motion_action_limit * 0.5,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select actions based on observations.

        Args:
            obs_dict (Dict): observation dictionary

        Returns:
            np.ndarray: action
        """
        obs = self._expert_observation_from_dict(obs_dict)
        action = np.zeros(4)
        motion = self._select_motion(obs).clip(
            -self._motion_action_limit,
            self._motion_action_limit
        )

        # Interpolate between expert policy and noise signal
        action[:3] = (
            motion * self._signal_to_noise_ratio +
            self._motion_noise.step(self._get_delta_time()) * (1 - self._signal_to_noise_ratio)
        ).clip(-self._motion_action_limit, self._motion_action_limit)

        action[3] = self._select_gripper_action(obs).clip(
            -self._gripper_action_limit,
            self._gripper_action_limit,
        )

        return action

    def _get_delta_time(self) -> float:
        """Get time difference between now and last call of this function."""
        delta_time = time.time() - self._time
        self._time = time.time()
        return delta_time

    def _expert_observation_from_dict(self, obs_dict: Dict[str, Any]) -> PickPlaceExpertObservation:
        """Convert observation dictionary to PickPlaceExpertObservation data object."""
        return PickPlaceExpertObservation(
            object_gripped=obs_dict["object_gripped"],
            dist_to_next_objective=obs_dict["dist_to_next_objective"],
            robot0_gripper_qpos=obs_dict["robot0_gripper_qpos"],
            robot0_gripper_qvel=obs_dict["robot0_gripper_qvel"],
        )

    def _select_motion(self, obs: PickPlaceExpertObservation) -> np.ndarray:
        """Select motion arguments of action (action[0:3])"""
        if self._above_next_objective(obs):
            if obs.object_gripped or self._gripper_fully_opened(obs):
                return self._move_to_next_objective(obs)
            else:
                return self._move_to_above_next_objective(obs)
        else:
            if obs.object_gripped:
                return self._move_to_above_next_objective(obs)
            else:
                return self._move_to_above_next_objective(obs)

    def _select_gripper_action(self, obs: PickPlaceExpertObservation) -> np.ndarray:
        """Select gripper actuation argument of action (action[3])"""
        if obs.object_gripped or self._at_next_objective(obs):
            return self._close_gripper()
        else:
            return self._open_gripper()

    def _gripper_fully_opened(self, obs: PickPlaceExpertObservation) -> bool:
        """Determine whether the gripper is opened further than a given threshold"""
        gripper_pos = obs.robot0_gripper_qpos
        return gripper_pos[0] - gripper_pos[1] > self._gripper_fully_opened_threshold

    def _at_next_objective(self, obs: PickPlaceExpertObservation) -> bool:
        """Determine whether the distance to the next objective is below a given threshold"""
        return (
            np.linalg.norm(obs.dist_to_next_objective[:2]) < self._horiz_dist_to_next_objective_threshold and
            -obs.dist_to_next_objective[2] < self._vert_dist_to_next_objective_threshold
        )

    def _above_next_objective(self, obs: PickPlaceExpertObservation) -> bool:
        """If the object is gripped:
            Determine whether the object is within a truncated cone above the target.
        Otherwise:
            Determine whether the gripper is within a truncated cone above the object.

        Minimum radius of the cone: self._dist_to_next_objective_threshold
        Cone angle: arctan(self._tan_theta)
        """
        max_radius = self._horiz_dist_to_next_objective_threshold - obs.dist_to_next_objective[2] * self._tan_theta
        return (
            np.linalg.norm(obs.dist_to_next_objective[:2]) < max_radius and
            obs.dist_to_next_objective[2] < 0
        )

    def _move_to_next_objective(self, obs: PickPlaceExpertObservation) -> np.ndarray:
        """Get the motion vector toward the next objective position (object or target)"""
        return obs.dist_to_next_objective

    def _move_to_above_next_objective(self, obs: PickPlaceExpertObservation) -> np.ndarray:
        """Get the motion vector toward a point a given distance above the next objective (object or target)"""
        vector_to_above_next_objective = obs.dist_to_next_objective + np.array([0, 0, self._hover_dist])
        vector_to_above_next_objective[2] *= 5
        return vector_to_above_next_objective

    def _open_gripper(self) -> np.ndarray:
        """Get a gripper actuation for opening the gripper"""
        return np.ones(1) * -1

    def _close_gripper(self) -> np.ndarray:
        """Get a gripper actuation for closing the gripper"""
        return np.ones(1)
