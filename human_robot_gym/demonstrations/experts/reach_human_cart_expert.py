"""This file implements an expert for the `ReachHumanCart` environment.

The policy does not take the human into consideration
and simply moves the end-effector directly towards the goal.

Author:
    Felix Trost (FT)

Changelog:
    16.06.23 FT File created
"""
from typing import Any, Dict, Optional

import numpy as np

from gym.spaces import Box

from human_robot_gym.utils.ou_process import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts.expert import Expert
from human_robot_gym.demonstrations.experts.reach_human_expert import ReachHumanExpertObservation


class ReachHumanCartExpert(Expert):
    """Expert policy for the `ReachHumanCart` environment.

    Does not take the human into account.
    Behavior:
        Moves the end-effector directly towards the goal.

    Noise can be added to the motion parameters. We draw from an Ornstein-Uhlenbeck (OU) process
    with asymptotic mean 0 and variance of half the motion action limit.

    Note that the OU process maintains a custom random number generator that is not affected by np.random.seed calls.
    Formula to obtain motion action parameters:
    `motion = expert_policy * signal_to_noise_ratio + noise * (1 - signal_to_noise_ratio)`

    Args:
        observation_space: The observation space of the environment.
        action_space: The action space of the environment.
        signal_to_noise_ratio: Interpolation factor between
            noise signal (Ornstein-Uhlenbeck process) -> signal_to_noise_ratio = 0
            and expert policy -> signal_to_noise_ratio = 1
        delta_time: Time step size for the OU process.
        seed: Seed for the OU process.
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        signal_to_noise_ratio: float = 1,
        delta_time: float = 0.01,
        seed: Optional[int] = None,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space
        )

        self._motion_action_limit = action_space.high[0]
        self._delta_time = delta_time
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0,
            sigma=self._motion_action_limit * 0.5,
            seed=seed,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select actions based on observations.
        Move directly towards the goal position (with added noise).

        Args:
            obs_dict (Dict[str, Any]): Dictionary containing observations.

        Returns:
            np.ndarray: Actions to be taken.
        """
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)
        motion = obs.goal_difference.clip(-self._motion_action_limit, self._motion_action_limit)
        motion = (
            self._signal_to_noise_ratio * motion +
            self._motion_noise.step(dt=self._delta_time) * (1 - self._signal_to_noise_ratio)
        ).clip(-self._motion_action_limit, self._motion_action_limit)

        return np.append(motion, 0.0)

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> ReachHumanExpertObservation:
        """Convert observation dictionary to `ReachHumanExpertObservation` observation."""
        return ReachHumanExpertObservation(
            goal_difference=obs_dict["goal_difference"]
        )
