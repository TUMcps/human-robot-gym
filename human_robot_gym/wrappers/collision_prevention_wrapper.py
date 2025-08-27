"""This file contains a robosuite wrapper that checks if the given action would result in a collision
and replaces the unsafe action with another action.

Author: Jakob Thumm
Changelog:
    27.08.25 JT Changed to robosuite wrapper
"""

import numpy as np
from robosuite.wrappers import Wrapper


class CollisionPreventionWrapper(Wrapper):
    """Checks if the given action would result in a collision and replaces the unsafe action with another action."""

    def __init__(self, env, collision_check_fn, replace_type=0, n_resamples=20):
        """Initialize the collision prevention wrapper.

        Args:
            env (MujocoEnv): The robosuite environment
            collision_check_fn (function): Function that checks if the action has a collision.
                Must take action as input.
                Must return true for collision and false for no collision.
            replace_type (int):
                0 - replace with zero action
                1 - resample new action from action space
                2 - find close safe action
            n_resamples (int): Number of resamples for type 1 or 2.
        """
        super().__init__(env)
        self.collision_check_fn = collision_check_fn
        self.n_resamples = n_resamples
        self.action_resamples = 0
        if replace_type == 0:
            self.replace_action = self.replace_zero
        elif replace_type == 1:
            self.replace_action = self.replace_random
        elif replace_type == 2:
            self.replace_action = self.replace_close
        else:
            raise NotImplementedError

    def step(self, action):
        """Wrap the step function with the replaced action and adds the new action to the info dict."""
        action = self.action(action)
        obs, reward, done, info = self.env.step(action)
        info["action"] = action
        info["action_resamples"] = self.action_resamples
        return obs, reward, done, info

    def action(self, action):
        """Replace the action if a collision is detected."""
        if self.collision_check_fn(action):
            action = self.replace_arm_only(action, self.replace_action)
            self.action_resamples += 1
        return action

    def replace_arm_only(self, action, replace_fn):
        """Replace only the arm part of the action, keep the rest.

        Args:
            action (np.array): Action to execute
            replace_fn (function): Function that replaces the action
        Returns:
            action (np.array)
        """
        composite_controller = self.unwrapped.robots[0].composite_controller
        for part_name, _ in composite_controller.part_controllers.items():
            start_idx, end_idx = composite_controller._action_split_indexes[part_name]
            if part_name not in composite_controller.grippers.keys():
                action[start_idx:end_idx] = replace_fn(action[start_idx:end_idx])
        return action

    def replace_zero(self, action):
        """Replace the action with a zero action.

        Args:
            action (np.array): Action to execute
        Returns:
            action (np.array)
        """
        return np.zeros_like(action)

    def replace_random(self, action):
        """Replace the action with a random action that is not in collision.

        Try to resample random action for n times, then just use zero action.
        Uses n = self.n_resamples

        Args:
            action (np.array): Action to execute
        Returns:
            action (np.array)
        """
        for _ in range(self.n_resamples):
            action = self.env.action_space.sample()[:action.shape[0]]
            if not self.collision_check_fn(action):
                return action
        return self.replace_zero(action)

    def replace_close(self, action):
        """Replace the action with a random action that is not in collision and close to action.

        Try to resample random action for n times, calculate the MSE distance to the given action,
        and pick the closest replacement action that is not in collision.
        If no safe action is found, use zero action.
        Uses n = self.n_resamples

        Args:
            action (np.array): Action to execute
        Returns:
            action (np.array)
        """
        replace_actions = []
        for _ in range(self.n_resamples):
            a = self.env.action_space.sample()[:action.shape[0]]
            if not self.collision_check_fn(a):
                replace_actions += [a]
        if len(replace_actions) > 0:
            min_distance = np.inf
            best_pick = -1
            for i in range(len(replace_actions)):
                dist = np.sum(np.power(action - replace_actions[i], 2))
                if dist < min_distance:
                    best_pick = i
                    min_distance = dist
            return replace_actions[best_pick]
        else:
            return self.replace_zero(action)

    def reset(self, **kwargs):
        """Reset the action wrapper variables and calls env.reset()."""
        self.action_resamples = 0
        return self.env.reset(**kwargs)
