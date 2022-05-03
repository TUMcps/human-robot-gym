"""This file descibes the normalized box environment wrapper.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import numpy as np
from gym.spaces import Box
import gym.core


class NormalizedBoxEnv(gym.core.Wrapper):
    """Normalize action to lie in [-1, 1].

    Optionally normalize observations and scale reward.

    Args:
        env: gym environment to wrap
        reward_scale: scale the reward
        obs_mean: usual mean of the observation
        obs_std: standard deviation of the observation
    """

    def __init__(
        self,
        env,
        reward_scale=1.0,
        obs_mean=None,
        obs_std=None,
    ):  # noqa: D107
        super().__init__(env)
        self._should_normalize = not (obs_mean is None and obs_std is None)
        if self._should_normalize:
            if obs_mean is None:
                obs_mean = np.zeros_like(env.observation_space.low)
            else:
                obs_mean = np.array(obs_mean)
            if obs_std is None:
                obs_std = np.ones_like(env.observation_space.low)
            else:
                obs_std = np.array(obs_std)
        self._reward_scale = reward_scale
        self._obs_mean = obs_mean
        self._obs_std = obs_std
        ub = np.ones(self.env.action_space.shape)
        self.action_space = Box(-1 * ub, ub)

    def estimate_obs_stats(self, obs_batch, override_values=False):
        """Estimate the obs mean and standard deviation.

        Args:
            obs_batch: Batch of observations
            override_values: self._obs_mean and self._obs_std will be overridden.
        """
        if self._obs_mean is not None and not override_values:
            raise Exception(
                "Observation mean and std already set. To "
                "override, set override_values to True."
            )
        self._obs_mean = np.mean(obs_batch, axis=0)
        self._obs_std = np.std(obs_batch, axis=0)

    def _apply_normalize_obs(self, obs):
        return (obs - self._obs_mean) / (self._obs_std + 1e-8)

    def step(self, action):
        """Step the environment.

        Scale the action and normalize the observations.
        """
        lb = self.env.action_space.low
        ub = self.env.action_space.high
        scaled_action = lb + (action + 1.0) * 0.5 * (ub - lb)
        scaled_action = np.clip(scaled_action, lb, ub)

        wrapped_step = self.env.step(scaled_action)
        next_obs, reward, done, info = wrapped_step
        if self._should_normalize:
            next_obs = self._apply_normalize_obs(next_obs)
        return next_obs, reward * self._reward_scale, done, info

    def __str__(self):
        """Return env as string."""
        return "Normalized: %s" % self.env
