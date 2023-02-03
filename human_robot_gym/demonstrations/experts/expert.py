from gym import Space
import numpy as np


class Expert:
    def __init__(
        self,
        observation_space: Space,
        action_space: Space,
    ):
        self.observation_space = observation_space
        self.action_space = action_space

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        return self.action_space.sample()
