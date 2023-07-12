from typing import Any, Dict, Optional
from dataclasses import dataclass

import numpy as np

from gym.spaces import Box

from human_robot_gym.demonstrations.experts import Expert
from human_robot_gym.utils.mjcf_utils import quat_to_rot


@dataclass
class CollaborativeLiftingCartExpertObservation:
    vec_eef_to_human_lh: np.ndarray
    vec_eef_to_human_rh: np.ndarray
    board_quat: np.ndarray


class CollaborativeLiftingCartExpert(Expert):
    def __init__(
        self,
        observation_space: Box,
        action_space: Box,
        board_size: np.ndarray,
        signal_to_noise_ratio: float,
        seed: Optional[int] = None,
    ):
        super().__init__(
            observation_space=observation_space,
            action_space=action_space
        )

        self.board_size = board_size

    def __call__(self, obs_dict: Dict[str, Any]) -> np.ndarray:
        obs = self.expert_observation_from_dict(obs_dict=obs_dict)

        vec_eef_to_human = (obs.vec_eef_to_human_lh + obs.vec_eef_to_human_rh) / 2

        height = vec_eef_to_human[2]
        flat_vec = quat_to_rot(obs.board_quat).apply(np.array([-1, 0, 0]))[:2]
        human_robot_xy_vec = self.board_size[0] * flat_vec * 0.9

        action = np.append(vec_eef_to_human[:2] - human_robot_xy_vec, height)

        # return np.array([0, 0, height, 1])
        return np.append(np.clip(action, -0.1, 0.1), 1)  #  * np.array([0.1, 0.1, 1, 1])

    @staticmethod
    def expert_observation_from_dict(obs_dict: Dict[str, Any]) -> CollaborativeLiftingCartExpertObservation:
        return CollaborativeLiftingCartExpertObservation(
            vec_eef_to_human_lh=obs_dict["vec_eef_to_human_lh"],
            vec_eef_to_human_rh=obs_dict["vec_eef_to_human_rh"],
            board_quat=obs_dict["board_quat"],
        )
