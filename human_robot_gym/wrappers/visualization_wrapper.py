from robosuite.wrappers import Wrapper


class VisualizationWrapper(Wrapper):
    def __init__(self, env, max_episode_steps=None):
        super(VisualizationWrapper, self).__init__(env)

    def step(self, action):
        observation, reward, done, info = self.env.step(action)
        self.env.render()
        return observation, reward, done, info
