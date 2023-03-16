"""This file implements the base class for expert policies.

Author:
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
"""
from gym import Space
import numpy as np
from typing import Any, Dict


class Expert:
    """Expert policy base class.

    Args:
        observation_space (Space): gym space for observations;
            should be the same as in the environment
        action_space (Space): gym space for actions;
            should be the same as in the environment
    """
    def __init__(
        self,
        observation_space: Space,
        action_space: Space,
    ):
        self.observation_space = observation_space
        self.action_space = action_space

    def __call__(self, observation: Dict[str, Any]) -> np.ndarray:
        """Query an action based on the given observation.

        Args:
            observation (np.ndarray): observation from current state

        Returns:
            np.ndarray: action
        """
        return self.action_space.sample()
