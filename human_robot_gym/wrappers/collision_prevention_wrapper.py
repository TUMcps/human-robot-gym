import numpy as np
from robosuite.wrappers import Wrapper
from gym import spaces


class CollisionPreventionWrapper(Wrapper):
    """Checks if the given action would result in a collision and replaces the unsafe action with another action."""

    def __init__(self, env, collision_check_fn, replace_type=0):
        """Initializes the collision prevention wrapper.

        Args:
            env (gym.env): The gym environment
            collision_check_fn (function): Function that checks if the action has a collision.
                Must take action as input.
                Must return true for collision and false for no collision.
            replace_type (int):
                0 - replace with zero action
                1 - TODO: resample new action from action space
                2 - TODO: find close safe action
        """
        assert isinstance(
            env.action_space, spaces.Box
        ), f"expected Box action space, got {type(env.action_space)}"

        super().__init__(env)
        self.collision_check_fn = collision_check_fn
        if replace_type == 0:
            self.replace_action = self.replace_zero
        else:
            raise NotImplementedError

    def step(self, action):
        """Wraps the step function with the replaced action and adds the new action to the info dict."""
        action = self.action(action)
        obs, reward, done, info = self.env.step(action)
        info["action"] = action
        return obs, reward, done, info

    def action(self, action):
        """Replaces the action if a collision is detected."""
        if self.collision_check_fn(action):
            action = self.replace_action(action)
            self.action_resamples += 1
        return action

    def replace_zero(self, action):
        """Replaces the action with a zero action.
        Args:
            action (np.array): Action to execute
        Returns:
            action (np.array)
        """
        return np.zeros([len(action)])
