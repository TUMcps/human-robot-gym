"""This file implements an expert for the `ReachHuman` environment.

The policy does not take the human into consideration
and simply moves the joints towards the target angles.

Author:
    Felix Trost (FT)

Changelog:
    16.06.23 FT File created
"""
from typing import Any, Dict, Optional
from dataclasses import dataclass

import numpy as np

from gym.spaces import Box

from human_robot_gym.utils.ou_process import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts.expert import Expert


@dataclass
class ReachHumanExpertObservation:
    """Data class to encapsulate all observation values relevant for the expert.

    Attributes:
        goal_difference: The difference between current and target state across all action parameters.
    """
    goal_difference: np.ndarray


class ReachHumanExpert(Expert):
    """Expert policy for the `ReachHuman` environment.

    Does not take the human into account.
    Behavior:
        Moves the joints directly towards their goal angles.

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

        self._delta_time = delta_time
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=action_space.shape[0],
            alpha=10,
            mu=0,
            sigma=0.5,
            seed=seed,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)

        # Append 0 as gripper actuation
        motion = np.append(obs.goal_difference, 0).clip(self.action_space.low, self.action_space.high)
        motion = (
            self._signal_to_noise_ratio * motion +
            self._motion_noise.step(dt=self._delta_time) * (1 - self._signal_to_noise_ratio) *
            0.5 * (self.action_space.high - self.action_space.low)
        ).clip(self.action_space.low, self.action_space.high)

        return motion

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> ReachHumanExpertObservation:
        return ReachHumanExpertObservation(
            goal_difference=obs_dict["goal_difference"]
        )
