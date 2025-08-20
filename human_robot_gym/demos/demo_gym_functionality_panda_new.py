"""This script shows an example of the Schunk robot being safely controlled in an human environment.

For instance, this can be used with our provided training function to train a safe RL agent.
"""

import robosuite as suite
import time
import numpy as np  # noqa: F401

from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)

if __name__ == "__main__":
    # Notice how the environment is wrapped by the wrapper
    controller_config = dict()
    controller_conig_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    robot_conig_path = file_path_completion("models/robots/config/panda.json")
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    controller_configs = [controller_config]

    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Panda",  # use Sawyer robot
            robot_base_offset=[0, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=True,  # make sure we can render to the screen
            render_camera=None,
            render_collision_mesh=False,
            reward_shaping=True,  # use dense rewards
            control_freq=5,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            horizon=1000,
            controller_configs=controller_configs,
            shield_type="SSM",
            visualize_failsafe_controller=True,
            visualize_pinocchio=False,
            base_human_pos_offset=[0.1, 0.0, 0.0],
            verbose=True,
            goal_dist=0.0001,
            human_rand=[0.0, 0.0, 0.0]
        ),
        keys=[
            "object-state",
            "robot0_proprio-state",
            "goal_difference"
        ]
    )

    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0
    )

    env = VisualizationWrapper(env)

    t_max = 100
    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            action = env.action_space.sample()
            pos = np.array([env.sim.data.qpos[x] for x in env.robots[0]._ref_joint_pos_indexes])
            goal = env.desired_goal
            action[:pos.shape[0]] = np.clip(goal-pos, -0.5, 0.5)
            observation, reward, done, info = env.step(action)
            print("Reward: {}".format(reward))
            if done or t == t_max:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, t / (time.time() - t1)))
