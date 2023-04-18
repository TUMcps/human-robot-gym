"""This file implements a wrapper for facilitating compatibility with OpenAI gym.

This wrapper is specifically designed to work with Hindsight Experience Replay,
so it sets up a goal environment.
Each observation has the following form:

    - '`observation`': Box,
    - '`achieved_goal`': Box,
    - '`desired_goal`': Box

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import numpy as np
from gym import spaces
from gym.core import Env

from robosuite.wrappers import Wrapper


class GoalEnvironmentGymWrapper(Wrapper, Env):
    """Goal environment gym wrapper.

    this class allows us to use hindsight experience replay (HER).
    The main point of goal environments is that both the desired and achieved goal are
    part of the observation.
    The observation must have the following form:
    gym.spaces.Dict({

        - '`observation`': gym.spaces.Box,
        - '`achieved_goal`': gym.spaces.Box,
        - '`desired_goal`': gym.spaces.Box

    })

    Args:
        env (MujocoEnv): The environment to wrap.
        keys (None or list of str): If provided, each observation will
            consist of concatenated keys from the wrapped environment's
            observation dictionary. Defaults to proprio-state and object-state.

    Raises:
        AssertionError: [Object observations must be enabled if no keys]
    """

    def __init__(self, env, keys=None):  # noqa: D107
        # Run super method
        super().__init__(env=env)
        # Create name for gym
        robots = "".join(
            [type(robot.robot_model).__name__ for robot in self.env.robots]
        )
        self.name = robots + "_" + type(self.env).__name__

        # Get reward range
        self.reward_range = (0, self.env.reward_scale)

        if keys is None:
            keys = []
            # Add object obs if requested
            if self.env.use_object_obs:
                keys += ["object-state"]
            # Add image obs if requested
            if self.env.use_camera_obs:
                keys += [f"{cam_name}_image" for cam_name in self.env.camera_names]
            # Iterate over all robots to add to state
            for idx in range(len(self.env.robots)):
                keys += ["robot{}_proprio-state".format(idx)]
            keys += ["desired_goal"]
        self.keys = keys

        # Gym specific attributes
        self.env.spec = None
        self.metadata = None

        # set up observation and action spaces
        obs = self.env.reset()
        flat_ob = self._flatten_obs(obs)
        obs_dim = flat_ob["observation"].size
        obs_high = np.inf * np.ones(obs_dim)
        obs_low = -obs_high
        goal_dim = flat_ob["desired_goal"].size
        goal_high = np.inf * np.ones(goal_dim)
        goal_low = -goal_high
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(low=obs_low, high=obs_high),
                "desired_goal": spaces.Box(low=goal_low, high=goal_high),
                "achieved_goal": spaces.Box(low=goal_low, high=goal_high),
            }
        )
        self.modality_dims = {key: obs[key].shape for key in self.keys}
        low, high = self.env.action_spec
        self.action_space = spaces.Box(low=low, high=high)

    def _flatten_obs(self, obs_dict, verbose=False):
        """Convert the observation to the observation, achieved, and desired goal dict format.

        Args:
            obs_dict (OrderedDict): ordered dictionary of observations
            verbose (bool): Whether to print out to console as observation keys are processed

        Returns:
            np.array: observations flattened into a 1d array
        """
        a_g = self.env._get_achieved_goal_from_obs(obs_dict)
        d_g = self.env._get_desired_goal_from_obs(obs_dict)
        ob_lst = []
        for key in self.keys:
            if key in obs_dict:
                if verbose:
                    print("adding key: {}".format(key))
                ob_lst.append(np.array(obs_dict[key]).flatten())
        observation = np.concatenate(ob_lst)
        obs = {"observation": observation, "achieved_goal": a_g, "desired_goal": d_g}
        return obs

    def reset(self):
        """Extend env reset method to return flattened observation instead of normal OrderedDict.

        Returns:
            np.array: Flattened environment observation space after reset occurs
        """
        ob_dict = self.env.reset()
        return self._flatten_obs(ob_dict)

    def step(self, action):
        """Extend vanilla step function call to return flattened observation instead of normal OrderedDict.

        Args:
            action (np.array): Action to take in environment

        Returns:
            4-tuple:

                - (np.array) flattened observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        """
        ob_dict, reward, done, info = self.env.step(action)
        return self._flatten_obs(ob_dict), reward, done, info

    def seed(self, seed=None):
        """Set numpy seed.

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
        """Compute the step reward.

        This externalizes the reward function and makes
        it dependent on an a desired goal and the one that was achieved. If you wish to include
        additional rewards that are independent of the goal, you can include the necessary values
        to derive it in info and compute it accordingly.

        Args:
            achieved_goal (object): the goal that was achieved during execution
            desired_goal (object): the desired goal that we asked the agent to attempt to achieve
            info (dict): an info dictionary with additional information
        Returns:
            float: The reward that corresponds to the provided achieved goal w.r.t. to the desired
            goal. Note that the following should always hold true:

                - `ob, reward, done, info = env.step()`
                - `assert reward == env.compute_reward(ob[achieved_goal], ob[goal], info)`

        """
        return self.env._compute_reward(achieved_goal, desired_goal, info)

    def compute_done(self, achieved_goal, desired_goal, info):
        """Compute the done flag.

        This externalizes the done function and makes
        it dependent on an a desired goal and the one that was achieved.

        Args:
            achieved_goal (object): the goal that was achieved during execution
            desired_goal (object): the desired goal that we asked the agent to attempt to achieve
            info (dict): an info dictionary with additional information
        Returns:
            bool or List[bool]
        """
        return self.env._compute_done(achieved_goal, desired_goal, info)

    def render(self, **kwargs):
        """Run the environment render function.

        Args:
            **kwargs (dict): Any args to pass to environment render function
        """
        return self.env.render(**kwargs)
