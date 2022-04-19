import gym
from gym import error


class GoalEnvironmentWrapper(gym.Wrapper):
    def __init__(self, env):
        super(GoalEnvironmentWrapper, self).__init__(env)

    def reset(self):
        # Enforce that each GoalEnv uses a Goal-compatible observation space.
        if not isinstance(self.env.observation_space, gym.spaces.Dict):
            raise error.Error('GoalEnv requires an observation space of type gym.spaces.Dict')
        result = self.env.reset()
        for key in ['observation', 'achieved_goal', 'desired_goal']:
            if key not in result:
                raise error.Error('GoalEnv requires the "{}" key to be part of the observation dictionary.'.format(key))
        return result

    def compute_reward(self, achieved_goal, desired_goal, info):
        """Compute the step reward. This externalizes the reward function and makes
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
                ob, reward, done, info = env.step()
                assert reward == env.compute_reward(ob['achieved_goal'], ob['goal'], info)
        """
        return self.env.compute_reward(achieved_goal, desired_goal, info)