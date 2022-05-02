"""This file defines the time limit wrapper.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
from robosuite.wrappers import Wrapper


class TimeLimit(Wrapper):
    """Wraps a robosuite environment with a time limit functionality.

    Args:
        env: Robotsuite environment to wrap.
        max_episode_steps: Maximum number of steps before timeout.
    """

    def __init__(self, env, max_episode_steps=None):  # noqa: D107
        super(TimeLimit, self).__init__(env)
        if max_episode_steps is None and self.env.spec is not None:
            max_episode_steps = env.spec.max_episode_steps
        if self.env.spec is not None:
            self.env.spec.max_episode_steps = max_episode_steps
        self._max_episode_steps = max_episode_steps
        self._elapsed_steps = None

    def step(self, action):
        """Step the environment.

        Sets the done flag after the max number of steps.
        """
        assert (
            self._elapsed_steps is not None
        ), "Cannot call env.step() before calling reset()"
        observation, reward, done, info = self.env.step(action)
        self._elapsed_steps += 1
        if self._elapsed_steps >= self._max_episode_steps:
            info["TimeLimit.truncated"] = not done
            done = True
        return observation, reward, done, info

    def reset(self, **kwargs):
        """Reset the environment step counter."""
        self._elapsed_steps = 0
        return self.env.reset(**kwargs)
