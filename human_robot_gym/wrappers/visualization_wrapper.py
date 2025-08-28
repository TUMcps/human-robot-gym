"""This file defines the a visualization wrapper for a robosuite environment.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
from robosuite.wrappers import Wrapper


class VisualizationWrapper(Wrapper):
    """Wraps a robosuite environment with a visualization functionality.

    Args:
        env: Robosuite environment to wrap.
    """

    def __init__(self, env):  # noqa: D107
        super(VisualizationWrapper, self).__init__(env)

    def step(self, action):
        """Step the environment and render the visualization."""
        observation, reward, done, info = self.env.step(action)
        self.unwrapped.render()
        return observation, reward, done, info
