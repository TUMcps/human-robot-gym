"""This script shows an example of a Panda robot in an human environment.

For instance, this can be used with our provided training function to train an RL agent in an human environment.
"""

import robosuite as suite
import time
from robosuite.wrappers import GymWrapper

import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401


if __name__ == "__main__":
    # Notice how the environment is wrapped by the wrapper
    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Panda",  # use Sawyer robot
            robot_base_offset=[0, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=True,  # make sure we can render to the screen
            render_camera=None,
            reward_shaping=True,  # use dense rewards
            control_freq=20,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            verbose=True,
        )
    )

    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(1000):
            action = env.action_space.sample()  # np.array([0, 1, 0, 0, 0, 0, 0, 0])
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))
