"""This file implements wrappers to add reward
for similarity between the actions of the agent and an expert.

Author:
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
"""
from typing import Tuple

import numpy as np
from gym.core import Env, Wrapper

from human_robot_gym.demonstrations.experts.expert import Expert


class ExpertImitationRewardWrapper(Wrapper):
    """Expert imitation reward gym wrapper.

    This is an abstract super class for expert imitation reward gym wrappers.
    Subclasses provide implementations for the get_imitation_reward method
    to use custom similarity metrics between actions.

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with same observation and action spaces as the environment
        alpha (float): linear interpolation factor between
            just environment reward (alpha = 0) and
            just imitation reward (alpha = 1)

    Raises:
        AssertionError: [Environment and expert have different observation space shapes]
        AssertionError: [Environment and expert have different action space shapes]
    """
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float = 0.1,
    ):
        assert env.observation_space.shape == expert.observation_space.shape, \
            "Environment and expert have different observation space shapes"

        assert env.action_space.shape == expert.action_space.shape, \
            "Environment and expert have different action space shapes"

        super().__init__(env)
        self._expert = expert
        self._last_obs = self.observation_space.sample()
        self._alpha = alpha

    def reset(self) -> np.ndarray:
        """Extend env reset method to store previous observation,
        as it is required for querying the expert.

        Returns:
            np.ndarray: observation after reset
        """
        obs: np.ndarray = super().reset()
        self._last_obs = obs.copy()
        return obs

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """Extend env step method to query expert on the same observation
        and add imitation reward.

        Args:
            action (np.ndarray): action chosen by agent to take in environment

        Returns:
            4-tuple:
                -(np.ndarray): new observation
                -(float): combined reward
                -(bool): whether the current episode is completed
                -(dict): misc information

        Raises:
            NotImplementedError [get_imitation_reward method not implemented in ExpertImitationRewardWrapper]
        """
        expert_action = self._expert(self._last_obs)
        obs, env_rew, done, info = super().step(action)
        self._last_obs = obs.copy()
        imitation_reward = self.get_imitation_reward(
            action,
            expert_action,
        )

        reward = self._combine_reward(env_rew, imitation_reward)
        return obs, reward, done, info

    def _combine_reward(
        self,
        env_reward: float,
        imitation_reward: float,
    ) -> float:
        """Combine the environment and imitation reward values by linear interpolation.

        Args:
            env_reward (float): reward given from wrapped env
            imitation_reward (float): reward obtained from imitating the expert

        Returns:
            float: combined reward
        """
        return imitation_reward * self._alpha + env_reward * (1 - self._alpha)

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray,
    ) -> float:
        """Extract the imitation reward from the similarity between agent and expert actions.
        Depends on the action space, therefore implemented in subclasses.

        Args:
            agent_action (np.ndarray): action chosen by the agent
            expert_action (np.ndarray): action chosen by the expert

        Returns:
            float: Imitation reward

        Raises:
            NotImplementedError [get_imitation_reward method not implemented in ExpertImitationRewardWrapper]
        """
        raise NotImplementedError("get_imitation_reward method not implemented in ExpertImitationRewardWrapper")


class CartActionsExpertImitationRewardWrapper(ExpertImitationRewardWrapper):
    """Expert imitation reward gym wrapper for the cartesian action space.
    Implements the get_imitation_reward method with a similarity metric taylored to cartesian control.

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with the same observation space as the environment
            and cartesian action space of the form (motion_x, motion_y, motion_z, gripper_actuation)
        alpha (float): linear interpolation factor between
            just environment reward (alpha = 0) and
            just imitation reward (alpha = 1)
        beta (float): linear interpolation factor to weight movement and gripper similarity in the imitation reward
            just gripper similarity: beta = 0
            just motion similarity: beta = 1
        motion_hwhm: scaling for differences between expert and agent movement actions;
            if the distance between their movements is motion_hwhm, the motion imitation reward is 0.5
        gripper_hwhm: scaling for differences between expert and agent gripper actions;
            if the distance between their gripper actuations is gripper_hwhm, the gripper imitation reward is 0.5

    Raises:
        AssertionError [Environment and expert have different observation space shapes]
        AssertionError [Environment and expert have different action space shapes]
        AssertionError [Environment does not have a 4-dim cartesian + gripper action space]
    """
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float,
        beta: float,
        motion_hwhm: float,
        gripper_hwhm: float,
    ):
        assert env.action_space.shape == (4,), "Environment does not have a 4-dim cartesian + gripper action space"
        super().__init__(env, expert, alpha)
        self._motion_hwhm = motion_hwhm
        self._gripper_hwhm = gripper_hwhm
        self._beta = beta

    def _similarity_fn(self, dist: float, hwhm: float) -> float:
        """Form a reward from the distance between agent and expert actions.
        Use a Gaussian density function with mean 0 and variance 1.
        Rescale distances so that dist=0 => reward=1 and dist=hwhm => reward=0.5.
        DeepMimic (Peng et al., 2018) uses a similar model for the end-effector similarity reward.

        Args:
            dist (float): euclidean distance between agent and expert
            hwhm (float): half width at half maximum;
                distance after which the reward should be at 0.5
        Returns:
            float: similarity based on distance
        """
        # exp form:
        # norm_factor = np.sqrt(2 * np.log(2))
        # return np.exp(-0.5 * (dist * norm_factor / hwhm)**2)

        return np.power(2, (dist / hwhm)**2)

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray
    ) -> float:
        """Override super class method to compute the imitation reward for the cartesian action space.
        Obtain similarities for both movement and gripper actuation and interpolate between them.

        Args:
            agent_action (np.ndarray): action chosen by the agent
            expert_action (np.ndarray): action chosen by the expert

        Returns:
            float: Imitation reward
        """
        motion_imitation_rew = self._similarity_fn(
            dist=np.linalg.norm(agent_action[:3] - expert_action[:3]),
            hwhm=self._motion_hwhm,
        )

        gripper_imitation_rew = self._similarity_fn(
            np.abs(agent_action[3] - expert_action[3]),
            self._gripper_hwhm,
        )

        return motion_imitation_rew * self._beta + gripper_imitation_rew * (1 - self._beta)
