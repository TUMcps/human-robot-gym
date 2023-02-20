"""This file implements a wrapper for facilitating compatibility with OpenAI gym.

This wrapper is specifically designed to work with scripted expert demonstrations.
As such, it assembles observations for the expert and stores them in the info dictionary.

These observations correspond refer to the previous state so that querying the expert can be done in another wrapper.

Author:
    Felix Trost (FT)

Changelog:
    15.02.23 FT File creation
"""
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from robosuite.environments import MujocoEnv
from robosuite.wrappers import Wrapper
from gym import Env, spaces


class ExpertObsWrapper(Wrapper, Env):
    """Expert observation gym wrapper.

    This class allows to assemble observations for scripted expert policies.
    In each environment step, the observations data of interest to the expert are stored in a dict
    and submitted in the info dict under the 'previous_expert_observation' key.

    Args:
        env (MujocoEnv): the environment to wrap
        agent_keys (None or list of str): if provided, each observation will
            consist of concatenated keys from the wrapped environment's observation dictionary.
            Defaults to proprio-state, optionally combined with object-state, and/or image,
            depending on flags set in wrapped environment (use_object_obs, use_camera_obs)
        expert_keys (None or list of str): used to filter the observation dict
            to assemble the expert observation. If not provided default to proprio-state,
            optionally combined with object-state, and/or image,
            depending on flags set in wrapped environment (use_object_obs, use_camera_obs)
    """
    def __init__(
        self,
        env: MujocoEnv,
        agent_keys: Optional[List[str]] = None,
        expert_keys: Optional[List[str]] = None,
    ):
        super().__init__(env=env)
        # Create name for gym
        robots = "".join([type(robot.robot_model).__name__ for robot in self.env.robots])
        self.name = robots + "_" + type(self.env).__name__

        self.reward_range = (0, self.env.reward_scale)

        self.agent_keys = self._get_keys(agent_keys)
        self.expert_keys = self._get_keys(expert_keys)

        # Gym specific attributes
        self.env.spec = None
        self.metadata = None

        # set up observation and action spaces
        obs = self.env.reset()
        self.agent_modality_dims = {key: obs[key].shape for key in self.agent_keys}
        self.expert_modality_dims = {key: obs[key].shape for key in self.expert_keys}
        flat_ob = self._flatten_obs(agent_keys, obs)
        self.obs_dim = flat_ob.size
        high = np.inf * np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low=low, high=high)
        low, high = self.env.action_spec
        self.action_space = spaces.Box(low=low, high=high)

        self._prev_expert_obs: Dict[str, Any] = {}

    def _get_keys(self, keys: Optional[List[str]]) -> List[str]:
        """Implement default values for keys.

        Default keys consist of robot proprio-states.
        If use_object_obs is set in the wrapped environment, add 'object-state'.
        If use_camera_obs is set in the wrapped environment, add image observations.

        Args:
            keys (None or list of str): the optional keys

        Returns:
            list of str: list of keys, None replaced by default key list
        """
        if keys is None:
            keys = []
            if self.env.use_object_obs:
                keys += ["object-state"]
            if self.env.use_camera_obs:
                keys += [f"{cam_name}_image" for cam_name in self.env.camera_names]
            for idx in range(len(self.env.robots)):
                keys += ["robot{}_proprio-state".format(idx)]

        return keys

    def _flatten_obs(
        self,
        keys: List[str],
        obs_dict: Dict[str, Any],
        verbose: bool = False
    ) -> np.ndarray:
        """Filters keys of interest out and concatenate the information.

        Args:
            keys (list of str): keys of interest
            obs_dict (OrderedDict): ordered dictionary of observations
            verbose (bool): Whether to print out to console as observation keys are processed

        Returns:
            np.array: observations flattened into a 1d array
        """
        ob_lst = []
        for key in keys:
            if key in obs_dict:
                if verbose:
                    print("adding key: {}".format(key))
                ob_lst.append(np.array(obs_dict[key]).flatten())
        return np.concatenate(ob_lst)

    def reset(self):
        """Extend environment's reset method to return flattened observation instead of normal OrderedDict.
        The expert observation is internally stored as a dictionary.

        Returns:
            np.array: Flattened environment observation space after reset occurs
        """
        obs_dict = self.env.reset()
        self._prev_expert_obs = {key: obs_dict[key] for key in self.expert_keys if key in obs_dict}

        return self._flatten_obs(
            keys=self.agent_keys,
            obs_dict=obs_dict,
        )

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Extend environment's step function call to return flattened observation instead of normal OrderedDict.
        The observation from before the step is added to the info dictionary.

        Args:
            action (np.array): Action to take in environment

        Returns:
            4-tuple:

                - (np.array) flattened observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        """
        obs_dict, reward, done, info = self.env.step(action)
        info["previous_expert_observation"] = self._prev_expert_obs
        self._prev_expert_obs = {key: obs_dict[key] for key in self.expert_keys if key in obs_dict}

        flat_agent_obs = self._flatten_obs(
            keys=self.agent_keys,
            obs_dict=obs_dict,
        )

        return flat_agent_obs, reward, done, info

    def seed(self, seed: Optional[float] = None):
        """Utility function to set numpy seed

        Args:
            seed (None or int): If specified, numpy seed to set

        Raises:
            TypeError: [Seed must be integer]
        """
        # Seed the generator
        if seed is not None:
            try:
                np.random.seed(seed)
            except Exception:
                TypeError("Seed must be an integer type!")

    def compute_reward(self, achieved_goal, desired_goal, info):
        """
        Dummy function to be compatible with gym interface that simply returns environment reward

        Args:
            achieved_goal: [NOT USED]
            desired_goal: [NOT USED]
            info: [NOT USED]

        Returns:
            float: environment reward
        """
        # Dummy args used to mimic Wrapper interface
        return self.env.reward()
