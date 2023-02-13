"""This file implements wrappers to add reward
for similarity between the actions of the agent and an expert.

Author:
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
    13.02.23 FT Integration of requested changes
"""
from typing import Tuple

import numpy as np
from gym.core import Env, Wrapper

from human_robot_gym.demonstrations.experts.expert import Expert


class ActionBasedExpertImitationRewardWrapper(Wrapper):
    """Expert imitation reward gym wrapper.

    This is an abstract super class for gym wrappers that reward the agent
    for the similarity between their actions and the ones of a given expert policy.
    Subclasses provide implementations for the get_imitation_reward method
    to use custom similarity metrics between actions.

    The reward is given by this formula:
        r = r_i * alpha + r_{env} * (1 - alpha)

    Where:
        r_{env}: reward from wrapped environment.
        r_i: reward obtained from imitating the expert's actions

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with same action space as the environment
        alpha (float): linear interpolation factor between
            just environment reward (alpha = 0) and
            just imitation reward (alpha = 1)

    Raises:
        AssertionError: [Environment and expert have different action space shapes]
    """
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float = 0,
    ):
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
        """Extend environment's step method to query the expert on the same observation
        and add an imitation reward.

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


class CartActionBasedExpertImitationRewardWrapper(ActionBasedExpertImitationRewardWrapper):
    """Expert imitation reward gym wrapper for the cartesian action space.
    Implements the get_imitation_reward method with a similarity metric taylored to cartesian control.
    The action space is expected to be of the form (motion_x, motion_y, motion_z, gripper_actuation)

    The reward is given by this formula:
        r = r_i * alpha + r_{env} * (1 - alpha)

    Where:
        r_{env}: reward from wrapped environment
        r_i = r_{motion} * beta + r_{gripper} * (1 - beta)
        r_{motion} = 2^{-(||a_m^a - a_m^e|| / iota_m)^2}
        r_{gripper} = 2^{-(|a_g^a - a_g^e| / iota_g)^2}

        a_m^a: motion action parameters of agent
        a_m^e: motion action parameters of expert
        a_g^a: gripper actuation action parameter of agent
        a_g^e: gripper actuation action parameter of expert

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with a cartesian action space of the form
            (motion_x, motion_y, motion_z, gripper_actuation)
        alpha (float): linear interpolation factor between
            just environment reward (alpha = 0) and
            just imitation reward (alpha = 1)
        beta (float): linear interpolation factor to weight movement and gripper similarity in the imitation reward
            just gripper similarity: beta = 0
            just motion similarity: beta = 1
        iota_m: scaling for differences between expert and agent movement actions;
            if the distance between their movements is iota_m, the motion imitation reward is 0.5
        iota_g: scaling for differences between expert and agent gripper actions;
            if the distance between their gripper actuations is iota_g, the gripper imitation reward is 0.5

    Raises:
        AssertionError [Environment and expert have different action space shapes]
        AssertionError [Environment does not have a 4-dim cartesian + gripper action space]
    """
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float = 0,
        beta: float = 0,
        iota_m: float = 0.1,
        iota_g: float = 0.25,
    ):
        assert env.action_space.shape == (4,), "Environment does not have a 4-dim cartesian + gripper action space"
        super().__init__(env, expert, alpha)
        self._iota_m = iota_m
        self._iota_g = iota_g
        self._beta = beta

    def _similarity_fn(self, dist: float, iota: float) -> float:
        """Form a reward from the distance between agent and expert actions.
        Use a Gaussian density function with mean 0 and variance 1.
        Rescale distances so that dist=0 => reward=1 and dist=iota => reward=0.5.
        DeepMimic (Peng et al., 2018) uses a similar model for the end-effector similarity reward.
        https://arxiv.org/abs/1804.02717

        Exponential form:
        exp(-1/2 * (dist * nu / iota)^2)

        where:
        nu = sqrt{2 * ln(2)}

        Simplifies to:
        2^{-(dist / iota)^2}

        Args:
            dist (float): euclidean distance between agent and expert
            iota (float): half width at half maximum;
                distance after which the reward should be at 0.5
        Returns:
            float: similarity based on distance
        """
        return np.power(2, -(dist / iota)**2)

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray,
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
            iota=self._iota_m,
        )

        gripper_imitation_rew = self._similarity_fn(
            dist=np.abs(agent_action[3] - expert_action[3]),
            iota=self._iota_g,
        )

        return motion_imitation_rew * self._beta + gripper_imitation_rew * (1 - self._beta)
