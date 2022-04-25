"""
This script shows how to adapt an environment to be compatible
with the OpenAI Gym-style API. This is useful when using
learning pipelines that require supporting these APIs.

For instance, this can be used with OpenAI Baselines
(https://github.com/openai/baselines) to train agents
with RL.


We base this script off of some code snippets found
in the "Getting Started with Gym" section of the OpenAI
gym documentation.

The following snippet was used to demo basic functionality.

    import gym
    env = gym.make('CartPole-v0')
    for i_episode in range(20):
        observation = env.reset()
        for t in range(100):
            env.render()
            print(observation)
            action = env.action_space.sample()
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break

To adapt our APIs to be compatible with OpenAI Gym's style, this script
demonstrates how this can be easily achieved by using the GymWrapper.
"""

import robosuite as suite
import time

from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401

if __name__ == "__main__":

    # Notice how the environment is wrapped by the wrapper
    controller_config = dict()
    controller_conig_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    robot_conig_path = file_path_completion(
        "/home/jakob/Promotion/code/human-robot-gym/human_robot_gym/models/robots/config/schunk.json"
    )
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    controller_configs = [controller_config]

    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Schunk",  # use Sawyer robot
            robot_base_offset=[-0.36, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=True,  # make sure we can render to the screen
            render_camera=None,
            render_collision_mesh=False,
            reward_shaping=True,  # use dense rewards
            control_freq=10,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            horizon=1000,
            controller_configs=controller_configs,
            use_failsafe_controller=True,
            visualize_failsafe_controller=False,
            visualize_pinocchio=False,
            base_human_pos_offset=[0.0, 0.0, 0.0],
        )
    )
    t_max = 100
    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            env.render()
            action = env.action_space.sample()  # np.array([0, 0.01, 0, 0, 0, 0, 0])
            observation, reward, done, info = env.step(action)
            if done or t == t_max:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))
