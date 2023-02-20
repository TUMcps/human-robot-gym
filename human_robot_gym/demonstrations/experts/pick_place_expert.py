"""This file implements an expert policy for the pick-place task.

The policy does not take the human into consideration,
    but only the values defined in the PickPlaceExpertObservation data class

Author:
    Felix Trost (FT)

Changelog:
    08.02.23 FT File creation
"""
from typing import Any, Dict
from dataclasses import dataclass
import numpy as np

from gym.spaces import Box

from human_robot_gym.utils.ou_noise import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts import Expert


@dataclass
class PickPlaceExpertObservation:
    """Data class to encapsulate relevant observation values.

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

    Args:
        observation_space (Space): the environment observation space
        action_space (Space): the environment action space
        expert_observation_from_array (np.ndarray -> PickPlaceExpertObservation):
            function to extract the relevant observation data from arrays
        motion_action_limit (float): action limit for motion in each direction
        gripper_action_limit (float): limit for the gripper actuation
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        signal_to_noise_ratio: float = 0.9
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space,
        )

        self._motion_action_limit = action_space.high[0]
        self._gripper_action_limit = action_space.high[-1]
        self._gripper_fully_opened_threshold = 0.02
        self._dist_to_next_objective_threshold = 0.02
        self._hover_dist = 0.2
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0,
            sigma=self._motion_action_limit,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select actions based on observations.

        Args:
            obs_array (np.ndarray): observation in array form

        Returns:
            np.ndarray: action
        """
        obs = self._expert_observation_from_dict(obs_dict)
        action = np.zeros(4)
        motion = self._select_motion(obs).clip(
            -self._motion_action_limit,
            self._motion_action_limit
        )

        action[:3] = (
            motion * self._signal_to_noise_ratio +
            self._motion_noise.step(0.01) * (1 - self._signal_to_noise_ratio)
        ).clip(-self._motion_action_limit, self._motion_action_limit)

        action[3] = self._select_gripper_action(obs).clip(
            -self._gripper_action_limit,
            self._gripper_action_limit,
        )

        return action

    def _expert_observation_from_dict(self, obs_dict: Dict[str, Any]) -> PickPlaceExpertObservation:
        return PickPlaceExpertObservation(
            object_gripped=obs_dict["object_gripped"],
            dist_to_next_objective=obs_dict["dist_to_next_objective"],
            robot0_gripper_qpos=obs_dict["robot0_gripper_qpos"],
            robot0_gripper_qvel=obs_dict["robot0_gripper_qvel"],
        )

    def _select_motion(self, obs: PickPlaceExpertObservation) -> np.ndarray:
        """Select motion part of action (action[0:3])"""
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
        """Select gripper actuation part of action (action[3])"""
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
            np.linalg.norm(obs.dist_to_next_objective[:2]) < self._dist_to_next_objective_threshold and
            obs.dist_to_next_objective[2] > -0.015
        )

    def _above_next_objective(self, obs: PickPlaceExpertObservation) -> bool:
        """Determine whether the horizontal distance to the next objective lies below a threshold
        and the vertical direction to the next objective is negative
        """
        # Above objective if within a cone above it
        max_radius = self._dist_to_next_objective_threshold - obs.dist_to_next_objective[2] * 0.5
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
        """Get the gripper actuation to open the gripper"""
        return np.ones(1) * -1

    def _close_gripper(self) -> np.ndarray:
        """Get the gripper actuation to close the gripper"""
        return np.ones(1)
