"""This file implements an expert policy for the `CollaborativeLiftingCart` environment.

The policy regards the point in the center between the human's hands. The end effector should
be at the same height as this point.
The horizontal motion is calculated from the board's orientation and size and this point.

The observation values required are contained in the `CollaborativeLiftingCartExpertObservation` dataclass.

Author:
    Felix Trost (FT)

Changelog:
    12.07.23 FT File created
"""
from typing import Any, Dict, Optional
from dataclasses import dataclass

import numpy as np

from gym.spaces import Box

from human_robot_gym.demonstrations.experts import Expert
from human_robot_gym.utils.mjcf_utils import quat_to_rot
from human_robot_gym.utils.ou_process import ReparameterizedOrnsteinUhlenbeckProcess


@dataclass
class CollaborativeLiftingCartExpertObservation:
    """Data class to encapsulate all observation values relevant for the expert.

    Attributes:
        vec_eef_to_human_lh: Vector from the end effector to the human's left hand.
        vec_eef_to_human_rh: Vector from the end effector to the human's right hand.
        board_quat: Quaternion representing the board's orientation.
    """
    vec_eef_to_human_lh: np.ndarray
    vec_eef_to_human_rh: np.ndarray
    board_quat: np.ndarray


class CollaborativeLiftingCartExpert(Expert):
    """Expert policy for the `CollaborativeLiftingCart` environment.

    The policy regards the point in the center between the human's hands. The end effector should
    be at the same height as this point.
    The horizontal motion is calculated from the board's orientation and size and this point.

    The observation values required are contained in the `CollaborativeLiftingCartExpertObservation` dataclass.

    Noise can be added to the motion parameters. We draw from an Ornstein-Uhlenbeck (OU) process
    with asymptotic mean 0 and variance of half the motion action limit as described in the action space.
    Note that the OU process maintains a custom random number generator that is not affected by np.random.seed calls.
    Formula to obtain motion action parameters:
    `motion = expert_policy * signal_to_noise_ratio + noise * (1 - signal_to_noise_ratio)`

    Args:
        observation_space (Space): the environment observation space
        action_space (Space): the environment action space
        board_size (np.ndarray): [x, y, z] size of the board as an array
        signal_to_noise_ratio (float): interpolation between
            noise signal (Ornstein-Uhlenbeck process) -> signal_to_noise_ratio = 0
            and expert policy -> signal_to_noise_ratio = 1
        delta_time (float): approximate time between two calls of the expert policy. Only used to step the OU process
        seed (int): random seed for the noise signal
    """
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        board_size: np.ndarray,

        signal_to_noise_ratio: float,
        delta_time: float = 0.01,
        seed: Optional[int] = None,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space
        )

        self._motion_action_limit = action_space.high[0]
        self._gripper_action_limit = action_space.high[-1]
        self._board_size = board_size
        self._signal_to_noise_ratio = signal_to_noise_ratio
        self._delta_time = delta_time
        self._motion_noise = ReparameterizedOrnsteinUhlenbeckProcess(
            size=3,
            alpha=0.5,
            mu=0.0,
            sigma=self._motion_action_limit * 0.5,
            seed=seed,
        )

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        """Select an action based on an expert observation dict.

        Args:
            obs_dict (Dict[str, Any]): expert observation dict

        Returns:
            np.ndarray: action
        """
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)

        vec_eef_to_human = (obs.vec_eef_to_human_lh + obs.vec_eef_to_human_rh) / 2

        height = vec_eef_to_human[2]
        flat_vec = quat_to_rot(obs.board_quat).apply(np.array([-1, 0, 0]))[:2]
        # Factor of 0.9 to account for the human not gripping the board at the corners
        human_robot_xy_vec = self._board_size[0] * flat_vec * 0.9

        motion = np.append(vec_eef_to_human[:2] - human_robot_xy_vec, height).clip(
            -self._motion_action_limit, self._motion_action_limit
        )

        motion_noise = self._motion_noise.step(dt=self._delta_time).clip(
            -self._motion_action_limit, self._motion_action_limit
        )

        motion_action = motion * self._signal_to_noise_ratio + motion_noise * (
            1 - self._signal_to_noise_ratio
        )

        return np.append(motion_action, self._gripper_action_limit)

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> CollaborativeLiftingCartExpertObservation:
        """Convert observation dictionary to `CollaborativeLiftingCartExpertObservation` data object."""
        return CollaborativeLiftingCartExpertObservation(
            vec_eef_to_human_lh=obs_dict["vec_eef_to_human_lh"],
            vec_eef_to_human_rh=obs_dict["vec_eef_to_human_rh"],
            board_quat=obs_dict["board_quat"],
        )
