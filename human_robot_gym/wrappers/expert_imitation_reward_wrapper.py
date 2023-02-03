from typing import Tuple

import numpy as np
from gym.core import Env, Wrapper

from human_robot_gym.demonstrations.experts.expert import Expert


class ExpertImitationRewardWrapper(Wrapper):
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float,
    ):
        assert env.observation_space.shape == expert.observation_space.shape, \
            "Environment and expert have different observation space shapes!"

        assert env.action_space.shape == expert.action_space.shape, \
            "Environment and expert have different action space shapes"

        super().__init__(env)
        self.expert = expert
        self._last_obs = self.observation_space.sample()
        self._alpha = alpha

    def reset(self) -> np.ndarray:
        obs: np.ndarray = super().reset()
        self._last_obs = obs.copy()
        return obs

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        expert_action = self.expert(self._last_obs)
        obs, env_rew, done, info = super().step(action)
        self._last_obs = obs.copy()
        imitation_reward = self.get_imitation_reward(
            action,
            expert_action,
        )

        reward = self.combine_reward(env_rew, imitation_reward)
        return obs, reward, done, info

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray,
    ) -> float:
        raise NotImplementedError()

    def combine_reward(
        self,
        env_reward: float,
        imitation_reward: float,
    ) -> float:
        return imitation_reward * self._alpha + env_reward * (1 - self._alpha)


class CartActionsExpertImitationRewardWrapper(ExpertImitationRewardWrapper):
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float,
        beta: float,
        motion_limit: float,
        gripper_limit: float,
    ):
        assert env.action_space.shape == (4,), "Environment does not have a cartesian + gripper action space"
        super().__init__(env, expert, alpha)
        self._motion_limit = motion_limit
        self._gripper_limit = gripper_limit
        self._beta = beta

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray
    ) -> float:
        motion_dist = ((agent_action[:3] - expert_action[:3])**2).sum() / (2 * self._motion_limit)
        gripper_dist = np.abs(agent_action[3] - expert_action[3]) / (2 * self._motion_limit)
        return motion_dist * self._beta + gripper_dist * (1 - self._beta)
