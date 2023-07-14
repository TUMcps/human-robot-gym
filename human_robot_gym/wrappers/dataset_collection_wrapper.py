"""This file contains a gym wrapper that can be used to record data from an environment by storing
the state representations, observations, actions and expert observations in a dataset directory.

Inside the dataset folder, subfolders for each episode are created. The names of these folders follows
the following scheme: ep_000000, ep_000001, ep_000002, ...

At the beginning of each episode, the model xml file is stored. At the end, a state.npz file is created
containing the states, observations, actions and expert observations of the episode.

Modified version of robosuite.wrappers.DataCollectionWrapper.

Author:
    Felix Trost (FT)

Changelog:
    15.06.23 FT file created
"""
import os
from typing import Any, Dict, Tuple

import numpy as np

import gym

from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
from human_robot_gym.environments.manipulation.human_env import HumanEnv


class DatasetCollectionWrapper(gym.Wrapper):
    """This wrapper can be used to record data from an environment by storing the state representations,
    observations, actions and expert observations in a dataset directory.

    Inside the dataset folder, subfolders for each episode are created. The names of these folders follows
    the following scheme: ep_000000, ep_000001, ep_000002, ...

    At the beginning of each episode, the model xml file is stored. At the end, a state.npz file is created
    containing the states, observations, actions and expert observations of the episode.

    As only the initial model file is stored, changes to it during the episode will not be reflected in the
    stored data.

    Requires the wrapped environment to be a HumanEnv.

    Args:
        env (gym.Env): The environment to monitor.
        directory (str): Where to store the dataset.
        start_episode (int): The index of the first episode. Defaults to 0.
        store_expert_observations (bool): Whether to store expert observations. Defaults to False.
        verbose (bool): Whether to print out debug information about the data collection. Defaults to False.

    Raises:
        [AssertionError: DatasetCollectionWrapper only works with HumanEnv environments!]
    """
    def __init__(
        self,
        env: gym.Env,
        directory: str,
        start_episode: int = 0,
        store_expert_observations: bool = False,
        verbose: bool = False,
    ):
        super().__init__(env)

        assert isinstance(self.unwrapped, HumanEnv), "DatasetCollectionWrapper only works with HumanEnv environments!"

        # the base directory for all logging
        self.directory = directory

        self._verbose = verbose

        self.states = []
        self.actions = []
        self.observations = []
        self._store_expert_observations = store_expert_observations
        self.expert_observations = []

        if not os.path.exists(directory):
            if self._verbose:
                print("DataCollectionWrapper: making new directory at {}".format(directory))

            os.makedirs(directory)

        # store logging directory for current episode
        self.ep_directory = None

        # remember whether any environment interaction has occurred
        self.has_interaction = False

        # some variables for remembering the current episode's initial state and model xml
        self._current_task_instance_state = None
        self._current_task_instance_xml = None

        self._episode_index = start_episode

    def _start_new_episode(self):
        """Bookkeeping to do at the start of each new episode."""
        # flush any data left over from the previous episode if any interactions have happened
        if self.has_interaction:
            self._flush()

        # timesteps in current episode
        self.t = 0
        self.has_interaction = False

        # save the task instance (will be saved on the first env interaction)
        self._current_task_instance_xml = self.unwrapped.sim.model.get_xml()
        self._current_task_instance_state = self.unwrapped.get_environment_state()

        # trick for ensuring that we can play MuJoCo demonstrations back
        # deterministically by using the recorded actions open loop
        self.unwrapped.reset_from_xml_string(self._current_task_instance_xml)
        self.unwrapped.set_environment_state(self._current_task_instance_state)

    def _on_first_interaction(self):
        """Bookkeeping for first timestep of episode.
        This function is necessary to make sure that logging only happens after the first
        step call to the simulation, instead of on the reset (people tend to call
        reset more than is necessary in code).

        Raises:
            AssertionError: [Could not store episode data, path already exists!]
        """

        self.has_interaction = True

        # create a directory with the current episode index
        self.ep_directory = os.path.join(self.directory, "ep_%06i" % self._episode_index)
        assert not os.path.exists(self.ep_directory), "Could not store episode data, path already exists!"
        if self._verbose:
            print("DataCollectionWrapper: making folder at {}".format(self.ep_directory))
        os.makedirs(self.ep_directory)

        # save the model xml
        xml_path = os.path.join(self.ep_directory, "model.xml")
        with open(xml_path, "w") as f:
            f.write(self._current_task_instance_xml)

        # save initial state and action
        assert len(self.states) == 0
        self.states.append(self._current_task_instance_state)
        if self._store_expert_observations:
            self.expert_observations.append(ExpertObsWrapper.get_from_wrapped_env(self).current_expert_observation)

        self._episode_index += 1

    def _flush(self):
        """Method to flush internal state to disk."""
        state_path = os.path.join(self.ep_directory, "state.npz")
        assert not os.path.exists(state_path), "Could not flush episode state data, path already exists!"
        env_name = self.unwrapped.__class__.__name__

        data_to_store = dict(
            states=self.states,
            actions=self.actions,
            observations=self.observations,
            env=env_name,
        )

        if self._store_expert_observations:
            data_to_store["expert_observations"] = self.expert_observations

        np.savez(
            state_path,
            **data_to_store
        )

        self.states = []
        self.actions = []
        self.expert_observations = []

    def reset(self) -> np.ndarray:
        """Extends vanilla reset() function call to accommodate data collection

        Returns:
            np.ndarray: Environment observation space after reset occurs
        """
        obs = super().reset()
        self._start_new_episode()
        self.observations = [obs]
        return obs

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Extends vanilla step() function call to accommodate data collection

        Args:
            action (np.array): Action to take in environment

        Returns:
            4-tuple:
                - (np.ndarray) observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (Dict[str, Any]) misc information
        """
        # on the first time step, make directories for logging
        if not self.has_interaction:
            self._on_first_interaction()

        obs, rew, done, info = super().step(action)
        self.t += 1

        # collect the current simulation state
        state = self.unwrapped.get_environment_state()
        self.states.append(state)

        self.observations.append(obs)
        self.actions.append(np.array(action))

        if self._store_expert_observations:
            self.expert_observations.append(ExpertObsWrapper.get_current_expert_observation_from_info(info))

        return obs, rew, done, info

    def close(self):
        """Override close method in order to flush left over data"""
        if self.has_interaction:
            self._flush()
        self.env.close()
