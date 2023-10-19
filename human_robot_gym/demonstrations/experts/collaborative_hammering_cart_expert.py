"""This file implements an expert policy for the `CollaborativeHammeringCart` environment.

The policy does not take the human into consideration,
but only the values defined in the `CollaborativeHammeringCartExpertObservation` dataclass.

Author:
    Felix Trost (FT)

Changelog:
    16.09.23 FT File created
"""
from typing import Any, Dict, Optional
from dataclasses import dataclass

import numpy as np

from gym.spaces import Box

from human_robot_gym.utils.ou_process import ReparameterizedOrnsteinUhlenbeckProcess
from human_robot_gym.demonstrations.experts.expert import Expert


@dataclass
class CollaborativeHammeringCartExpertObservation:
    vec_eef_to_nail: np.ndarray


class CollaborativeHammeringCartExpert(Expert):
    """Expert policy for the `CollaborativeHammeringCart` environment.

    Does not take the human into account.

    The relevant observation data is described in the `CollaborativeHammeringCartExpertObservation` dataclass.

    Noise can be added to the motion parameters. We draw from an Ornstein-Uhlenbeck (OU) process
    with asymptotic mean 0 and variance of half the motion action limit as described in the action space.
    Note that the OU process maintains a custom random number generator that is not affected by np.random.seed calls.
    Formula to obtain motion action parameters:
    `motion = expert_policy * signal_to_noise_ratio + noise * (1 - signal_to_noise_ratio)`

    Args:
        observation_space (Space): the environment observation space
        action_space (Space): the environment action space
        signal_to_noise_ratio (float): interpolation between
            - noise signal (Ornstein-Uhlenbeck process) -> `signal_to_noise_ratio = 0`
            - expert signal -> `signal_to_noise_ratio = 1`
        delta_time (float): time between two calls of the expert policy. Used to step the OU process
        seed (int): random seed for the noise signal
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        signal_to_noise_ratio: float = 1,
        delta_time: float = 0.1,
        seed: Optional[int] = None,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space,
        )

        self._motion_action_limit = action_space.high[0]

        self.motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0,
            sigma=self._motion_action_limit * 0.5,
            seed=seed,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select actions based on observations.

        Args:
            obs_dict (Dict): observation dictionary

        Returns:
            np.ndarray: action
        """
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)

        motion = np.clip(obs.vec_eef_to_nail + np.array([-0.1, 0, 0]), -0.1, 0.1)

        action = np.zeros(4)

        action[:-1] = motion
        action[-1] = self._close_gripper()

        return action

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> CollaborativeHammeringCartExpertObservation:
        """Convert oobservation dictionary to `CollaborativeHammeringExpertObservation` data object."""
        return CollaborativeHammeringCartExpertObservation(
            vec_eef_to_nail=obs_dict["vec_eef_to_nail"],
        )

    def _close_gripper(self) -> np.ndarray:
        """Get a gripper actuation for closing the gripper"""
        return np.ones(1)
