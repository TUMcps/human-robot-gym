"""This file implements wrappers to add reward
for similarity between the actions of the agent and an expert.

Author:
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
    13.02.23 FT Integration of requested changes
"""
from typing import List, Tuple, Union

import numpy as np
from gym.core import Env, Wrapper

from human_robot_gym.demonstrations.experts.expert import Expert
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
from human_robot_gym.utils.expert_imitation_reward_utils import similarity_fn


class ActionBasedExpertImitationRewardWrapper(Wrapper):
    r"""Abstract super class for gym wrappers generating imitation reward
    based on the similarity of expert and agent actions.

    This is an abstract super class for gym wrappers that reward the agent
    for the similarity between their actions and the ones of a given expert policy.
    Subclasses provide implementations for the get_imitation_reward method
    to use custom similarity metrics between actions.

    The reward is given by this formula:
        $r = r_i * \alpha + r_{env} * (1 - \alpha)$

    Where:
        $r_{env}$: reward from wrapped environment.
        $r_i$: reward obtained from imitating the expert's actions

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with same action space as the environment
        alpha (float): linear interpolation factor between
            just environment reward (`alpha = 0`) and
            just imitation reward (`alpha = 1`)

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
        self._alpha = alpha

        self._imitation_rewards = None
        self._environment_rewards = None

    def reset(self) -> np.ndarray:
        """Extend environment's reset method to empty the list of environment and imitation rewards collected."""
        self._imitation_rewards = []
        self._environment_rewards = []

        return super().reset()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """Extend environment's step method to query the expert on the same observation and add an imitation reward.

        Args:
            action (np.ndarray): action chosen by agent to take in environment

        Returns:
            4-tuple:
                -(np.ndarray): new observation
                -(float): combined reward
                -(bool): whether the current episode is completed
                -(dict): misc information

        Raises:
            NotImplementedError [get_imitation_reward method not implemented in ActionBasedExpertImitationRewardWrapper]
            AssertionError [Expert observation not stored in info dict]
        """
        obs, env_reward, done, info = super().step(action)

        assert "previous_expert_observation" in info, "Expert observation not stored in info dict"
        expert_action = self._expert(ExpertObsWrapper.get_previous_expert_observation_from_info(info))
        imitation_reward = self.get_imitation_reward(
            action,
            expert_action,
        )

        self._imitation_rewards.append(imitation_reward)
        self._environment_rewards.append(env_reward)

        reward = self._combine_reward(env_reward, imitation_reward)

        # Log the imitation and env rewards
        if done:
            self._add_reward_to_info(info)

        return obs, reward, done, info

    def _add_reward_to_info(self, info: dict):
        """Add data to the info dict.

        Add the following data to the info dict:
            - `ep_im_rew_mean`: sum of imitation rewards in episode
            - `ep_env_rew_mean`: sum of environment rewards in episode
            - `ep_full_rew_mean`: sum of combined imitation and environment rewards in episode
            - `im_rew_mean`: mean of imitation rewards in episode
            - `env_rew_mean`: mean of environment rewards in episode
            - `full_rew_mean`: mean of combined imitation and environment rewards in episode
        """
        info["ep_im_rew_mean"] = np.sum(self._imitation_rewards)
        info["ep_env_rew_mean"] = np.sum(self._environment_rewards)
        info["ep_full_rew_mean"] = self._combine_reward(
            env_reward=info["ep_env_rew_mean"],
            imitation_reward=info["ep_im_rew_mean"],
        )

        info["im_rew_mean"] = np.nan if len(self._imitation_rewards) == 0 else np.mean(self._imitation_rewards)
        info["env_rew_mean"] = np.nan if len(self._environment_rewards) == 0 else np.mean(self._environment_rewards)
        info["full_rew_mean"] = self._combine_reward(
            env_reward=info["env_rew_mean"],
            imitation_reward=info["im_rew_mean"],
        )

    def _combine_reward(
        self,
        env_reward: Union[float, List[float]],
        imitation_reward: Union[float, List[float]],
    ) -> float:
        """Combine the environment and imitation reward values by linear interpolation.

        Values either given as single values or as two lists of values.

        Args:
            env_reward (float | List[float]): reward given from wrapped env
            imitation_reward (float | List[float]): reward obtained from imitating the expert

        Returns:
            float: combined reward

        Raises:
            ValueError ["Either both or none of env and imitation are expected to be lists"]
        """
        if isinstance(env_reward, List) and isinstance(imitation_reward, List):
            return [r_im * self._alpha + r_env * (1 - self._alpha) for r_im, r_env in zip(imitation_reward, env_reward)]
        elif isinstance(env_reward, List) or isinstance(imitation_reward, List):
            raise ValueError("Either both or none of env and imitation are expected to be lists")
        else:
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


class JointActionBasedExpertImitationRewardWrapper(ActionBasedExpertImitationRewardWrapper):
    r"""Action-based expert imitation reward gym wrapper for the joint position action space.
    Implements the get_imitation_reward method with a similarity metric taylored to joint space control.

    The reward is given by this formula:
        $r = r_i * \alpha + r_{env} * (1 - \alpha)$

    Where:
        $r_{env}$: reward from wrapped environment
        $r_i = r_{motion} * \beta + r_{gripper} * (1 - \beta)$
        $r_{motion} = sim_{motion}(||a_m^a - a_m^e||, \iota_m)$
        $r_{gripper} = sim_{gripper}(|a_g^a - a_g^e|, \iota_g)$

        $a_m^a$: target joint position action parameters of agent.
        $a_m^e$: target joint position action parameters of expert
        $a_g^a$: gripper actuation action parameter of agent
        $a_g^e$: gripper actuation action parameter of expert
        $sim_{motion}$ and $sim_{gripper}: similarity functions, either sim_G or sim_T
            For more details, see `human_robot_gym.utils.expert_imitation_reward_utils`

    The target joint position action parameters can be rescaled to the range [-1, 1]
    by setting `normalize_joint_actions=True`. This way, the imitation reward is independent
    of the joint limits of the individual joints.

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with a cartesian action space of the form
            `(motion_x, motion_y, motion_z, gripper_actuation)`
        alpha (float): linear interpolation factor between
            just environment reward (`alpha = 0`) and
            just imitation reward (`alpha = 1`)
        beta (float): linear interpolation factor
            to weight joint movement and gripper similarity in the imitation reward:
            - just gripper similarity: `beta = 0`
            - just motion similarity: `beta = 1`
        iota_m: scaling for differences between expert and agent target joint position actions;
            if the distance between their target joint positions is iota_m, the joint motion imitation reward is 0.5
        iota_g: scaling for differences between expert and agent gripper actions;
            if the distance between their gripper actuations is iota_g, the gripper imitation reward is 0.5
        m_sim_fn: similarity function to use for the target joint position imitation reward.
            Can be either `"gaussian"` or `"tanh"`.
        g_sim_fn: similarity function to use for the gripper imitation reward. Can be either `"gaussian"` or `"tanh"`.
        normalize_joint_actions: whether to normalize the joint actions to the range [-1, 1]
            If `True`, takes the joint limits of the individual joints into account. Default: `False`

    Raises:
        AssertionError [Environment and expert have different action space shapes]
    """
    def __init__(
        self,
        env: Env,
        expert: Expert,
        alpha: float = 0,
        beta: float = 0,
        iota_m: float = 0.1,
        iota_g: float = 0.25,
        m_sim_fn: str = "gaussian",
        g_sim_fn: str = "gaussian",
        normalize_joint_actions: bool = False,
    ):
        super().__init__(env, expert, alpha)
        self._iota_m = iota_m
        self._iota_g = iota_g
        self._beta = beta
        self._m_sim_fn = m_sim_fn
        self._g_sim_fn = g_sim_fn
        self._motion_action_dim = env.action_space.shape[0] - 1
        self._normalize_joint_actions = normalize_joint_actions

    def _get_joint_action_delta(self, agent_action: np.ndarray, expert_action: np.ndarray) -> np.ndarray:
        """Compute the delta between the agent and expert joint actions.

        If requested, normalize the joint actions to the range [-1, 1].

        Args:
            agent_action (np.ndarray): action chosen by the agent
            expert_action (np.ndarray): action chosen by the expert

        Returns:
            np.ndarray: delta between agent and expert joint actions
        """
        if self._normalize_joint_actions:
            lower_limit = np.array(self.env.action_space.low[:-1])
            upper_limit = np.array(self.env.action_space.high[:-1])

            agent_action = 2 * (agent_action - lower_limit) / (upper_limit - lower_limit) - 1
            expert_action = 2 * (expert_action - lower_limit) / (upper_limit - lower_limit) - 1

        return np.linalg.norm(agent_action - expert_action)

    def get_imitation_reward(
        self,
        agent_action: np.ndarray,
        expert_action: np.ndarray,
    ) -> float:
        """Override super class method to compute the imitation reward for the joint action space.

        Obtain similarities for both target joint angles and gripper actuation and interpolate between them.

        Args:
            agent_action (np.ndarray): action chosen by the agent
            expert_action (np.ndarray): action chosen by the expert

        Returns:
            float: Imitation reward
        """
        motion_imitation_rew = similarity_fn(
            name=self._m_sim_fn,
            delta=self._get_joint_action_delta(
                agent_action=agent_action[:-1],
                expert_action=expert_action[:-1]
            ),
            iota=self._iota_m,
        )

        gripper_imitation_rew = similarity_fn(
            name=self._g_sim_fn,
            delta=np.abs(agent_action[-1] - expert_action[-1]),
            iota=self._iota_g,
        )

        return motion_imitation_rew * self._beta + gripper_imitation_rew * (1 - self._beta)


class CartActionBasedExpertImitationRewardWrapper(ActionBasedExpertImitationRewardWrapper):
    r"""Action-based expert imitation reward gym wrapper for the cartesian action space.
    Implements the get_imitation_reward method with a similarity metric taylored to cartesian control.
    The action space is expected to be of the form `(motion_x, motion_y, motion_z, gripper_actuation)`

    The reward is given by this formula:
        $r = r_i * \alpha + r_{env} * (1 - \alpha)$

    Where:
        $r_{env}$: reward from wrapped environment
        $r_i = r_{motion} * \beta + r_{gripper} * (1 - \beta)$
        $r_{motion} = sim_{motion}(||a_m^a - a_m^e||, \iota_m)$
        $r_{gripper} = sim_{gripper}(|a_g^a - a_g^e|, \iota_g)$

        $a_m^a$: motion action parameters of agent
        $a_m^e$: motion action parameters of expert
        $a_g^a$: gripper actuation action parameter of agent
        $a_g^e$: gripper actuation action parameter of expert
        $sim_{motion}$ and $sim_{gripper}: similarity functions, either sim_G or sim_T
            For more details, see `human_robot_gym.utils.expert_imitation_reward_utils`

    Args:
        env (Env): gym environment to wrap
        expert (Expert): expert with a cartesian action space of the form
            `(motion_x, motion_y, motion_z, gripper_actuation)`
        alpha (float): linear interpolation factor between
            just environment reward (`alpha = 0`) and
            just imitation reward (`alpha = 1`)
        beta (float): linear interpolation factor to weight movement and gripper similarity in the imitation reward
            just gripper similarity: `beta = 0`
            just motion similarity: `beta = 1`
        iota_m: scaling for differences between expert and agent movement actions;
            if the distance between their movements is iota_m, the motion imitation reward is 0.5
        iota_g: scaling for differences between expert and agent gripper actions;
            if the distance between their gripper actuations is iota_g, the gripper imitation reward is 0.5
        m_sim_fn: similarity function to use for movement imitation reward. Can be either `"gaussian"` or `"tanh"`.
        g_sim_fn: similarity function to use for gripper imitation reward. Can be either `"gaussian"` or `"tanh"`.

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
        m_sim_fn: str = "gaussian",
        g_sim_fn: str = "gaussian",
    ):
        assert env.action_space.shape == (4,), "Environment does not have a 4-dim cartesian + gripper action space"
        super().__init__(env, expert, alpha)
        self._iota_m = iota_m
        self._iota_g = iota_g
        self._beta = beta
        self._m_sim_fn = m_sim_fn
        self._g_sim_fn = g_sim_fn

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

        # Action values limited in each direction separately -> maximum distance: 2*sqrt(3)*action_max
        motion_imitation_rew = similarity_fn(
            name=self._m_sim_fn,
            delta=np.linalg.norm(agent_action[:3] - expert_action[:3]),
            iota=self._iota_m,
        )

        gripper_imitation_rew = similarity_fn(
            name=self._g_sim_fn,
            delta=np.abs(agent_action[3] - expert_action[3]),
            iota=self._iota_g,
        )

        return motion_imitation_rew * self._beta + gripper_imitation_rew * (1 - self._beta)
