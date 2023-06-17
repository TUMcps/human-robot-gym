"""This file implements wrappers for utilizing datasets created by the
`human_robot_gym/training/create_expert_dataset.py` script.

The wrappers below leverage this data for observation normalization or reference state initialization (RSI).

Author:
    Felix Trost (FT)

Changelog:
    17.06.23 (FT): File created
"""
from typing import Any, Dict, List, Optional, Tuple
import os

import numpy as np

import gym

from human_robot_gym.utils.mjcf_utils import file_path_completion
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper


class DatasetWrapper(gym.Wrapper):
    """Base class for wrappers leveraging datasets created by the
    `human_robot_gym/training/create_expert_dataset.py` script.

    Loads the entire dataset from disk into memory on initialization.
    Be careful when using large datasets or many parallel environments as large amounts of memory might be required.

    Args:
        env (gym.Env): The environment to wrap
        dataset_name (str): The name of the dataset to use
    """
    def __init__(
        self,
        env: gym.Env,
        dataset_name: str,
    ):
        super().__init__(env=env)
        self.dataset = self.load_dataset(dataset_name=dataset_name)

    @staticmethod
    def load_dataset(dataset_name: str) -> List[Tuple[str, Dict[str, Any]]]:
        """Loads the dataset from disk. The dataset is stored in a list of tuples containing the xml string and a
        dictionary containing the states, observations, expert observations, and actions of each transition in the
        dataset.

        Args:
            dataset_name (str): The name of the dataset to use

        Returns:
            List[Tuple[str, Dict[str, Any]]]: A list of tuples containing the xml string and a dictionary containing
                the states, observations, expert observations, and actions of each transition in the dataset

        Raises:
            [AssertionError: Dataset x does not exist at y]
        """
        dataset_path = file_path_completion(f"../datasets/{dataset_name}")

        assert os.path.exists(dataset_path), f"Dataset {dataset_name} does not exist at {dataset_path}"

        ep_folders = [folder for folder in next(os.walk(dataset_path))[1] if folder.startswith("ep_")]
        dataset = []

        for ep_folder in ep_folders:
            ep_path = f"{dataset_path}/{ep_folder}"

            xml_path = os.path.join(ep_path, "model.xml")
            states_path = os.path.join(ep_path, "state.npz")

            with open(xml_path, "r") as f:
                xml_file = f.read()

            with np.load(states_path, allow_pickle=True) as dic_file:
                dic = dict(
                    states=dic_file["states"],
                    observations=dic_file["observations"],
                    expert_observations=dic_file["expert_observations"],
                    actions=dic_file["actions"],
                )

            dataset.append((xml_file, dic))

        return dataset


class DatasetRSIWrapper(DatasetWrapper):
    """Wrapper for initializing the environment from dataset states on reset calls.

    Adds the possibility to perform reference state initialization (RSI) by choose random states from along the
    episode trajectories.

    Args:
        env (gym.Env): The environment to wrap
        dataset_name (str): The name of the dataset to use
        rsi_prob (float): The probability of performing RSI on reset calls.
            If set to 0, the environment is always initialized from the first state of a random episode.
            Otherwise, chooses a random state from a random episode. Defaults to 0.
    """
    def __init__(
        self,
        env: gym.Env,
        dataset_name: str,
        rsi_prob: float = 0,
    ):
        super().__init__(env=env, dataset_name=dataset_name)

        self._rsi_prob = rsi_prob
        self._dic = None
        self._ep_idx = None
        self._dataset_transition_count = None
        self._dataset_ep_step_idx = None

    def reset(self) -> np.ndarray:
        """Extend reset method to initialize from dataset states.

        Returns:
            np.ndarray: The initial observation"""
        super().reset()  # Observation gained in this step is discarded

        self._ep_idx = np.random.randint(len(self.dataset))
        self._dic = self.dataset[self._ep_idx][1]

        self._dataset_transition_count = len(self._dic["states"]) - 1
        self._dataset_ep_step_idx = self._get_initial_dataset_ep_step_idx()

        self.unwrapped.reset_from_xml_string(self.dataset[self._ep_idx][0])
        self.unwrapped.set_state(self._dic["states"][self._dataset_ep_step_idx])

        expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(self.env)
        if expert_obs_wrapper is not None:
            expert_obs_wrapper._current_expert_observation = self._dic["expert_observations"][self._dataset_ep_step_idx]

        return self._dic["observations"][self._dataset_ep_step_idx]

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Extend step method to keep track of the corresponding state index in the dataset episode.

        Args:
            action (np.ndarray): The action to perform

        Returns:
            Tuple[np.ndarray, float, bool, Dict[str, Any]]: The next observation, the reward, whether the episode is
                done, and additional info
        """
        obs, reward, done, info = super().step(action)

        self._dataset_ep_step_idx = min(self._dataset_ep_step_idx + 1, self._dataset_transition_count)

        return obs, reward, done, info

    def _get_initial_dataset_ep_step_idx(self) -> int:
        if np.random.rand() < self._rsi_prob:
            return np.random.randint(self._dataset_transition_count)
        else:
            return 0


class DatasetObsNormWrapper(DatasetWrapper):
    r"""Wrapper for normalizing observations based on dataset statistics.

    Obtains mean and std of per observation value from the dataset and normalizes observations accordingly.
    Additionally supports squashing observations to [-1, 1] by applying tanh with a given factor.
    This induces a fish-eye effect when using cartesian vector observations:
    It exaggerates value differences close to the mean and compresses value differences far away from it.
    Using squashing on distance metrics might not have the desired effect as short and long distances are
    compressed alike.

    The used dataset is removed from memory after obtaining the statistics to save memory.

    Formula:
        normed_obs = (obs - \mu) / \sigma
        If squash_factor is not None:
            normed_obs = tanh(squash_factor * normed_obs)

    Args:
        env (gym.Env): The environment to wrap
        dataset_name (str): The name of the dataset to use
        squash_factor (float, optional): The factor to use for squashing observations.
    """
    def __init__(
        self,
        env: gym.Env,
        dataset_name: str,
        squash_factor: Optional[float] = None,
    ):
        super().__init__(env=env, dataset_name=dataset_name)

        observations = np.concatenate([dic["observations"] for _, dic in self.dataset], axis=0)
        self._obs_mean = np.mean(observations, axis=0)
        self._obs_std = np.std(observations, axis=0)

        assert self._obs_mean.shape == self.observation_space.shape, \
            "Environment and dataset observation space do not match!"

        # Remove dataset from memory to save memory
        del self.dataset
        del observations

        self._squash_factor = squash_factor

        # Without squashing, the observation space is not guaranteed to be bounded in [-1, 1]
        if squash_factor is not None:
            self.observation_space = gym.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=self.observation_space.shape,
                dtype=self.observation_space.dtype,
            )

    def reset(self):
        """Extend reset method to normalize observations.

        Returns:
            np.ndarray: The initial observation after normalization
        """
        obs = super().reset()

        normed_obs = (obs - self._obs_mean) / self._obs_std

        if self._squash_factor is not None:
            normed_obs = np.tanh(self._squash_factor * normed_obs)

        return normed_obs
