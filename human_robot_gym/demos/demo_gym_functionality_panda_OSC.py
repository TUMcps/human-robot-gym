"""This script shows an example of the Schunk robot being safely controlled in an human environment.

For instance, this can be used with our provided training function to train a safe RL agent.
"""

import robosuite as suite
import time
import numpy as np  # noqa: F401

from robosuite.wrappers.gym_wrapper import GymWrapper
import robosuite as suite
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)

if __name__ == "__main__":
    """Main."""
    arm_controller_config = suite.load_part_controller_config(default_controller="OSC_POSE")
    controller_configs = refactor_composite_controller_config(
        arm_controller_config, "Panda", ["right"]
    )

    env = suite.make(
        "ReachHumanCart",
        robots="Panda",  # use Sawyer robot
        robot_base_offset=[0, 0, 0],
        use_camera_obs=False,  # do not use pixel observations
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        has_renderer=True,  # make sure we can render to the screen
        render_camera=None,
        renderer="mjviewer",
        render_collision_mesh=False,
        reward_shaping=True,  # use dense rewards
        control_freq=5,  # control should happen fast enough so that simulation looks smooth
        hard_reset=False,
        horizon=1000,
        controller_configs=controller_configs,
        use_failsafe_controller=False,
        shield_type="SSM",
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.1, 0.0, 0.0],
        init_joint_pos=np.array([0.0, -1.28, 0.0, -2.54, 0.0, 1.553, 0.654]),
        verbose=True,
        goal_dist=0.0001,
        human_rand=[0.0, 0.0, 0.0]
    )

    env = VisualizationWrapper(env)

    env = GymWrapper(
        env,
        keys=[
            "object-state",
            "robot0_proprio-state",
            "goal_difference"
        ]
    )

    t_max = 100
    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            action = np.zeros_like(env.action_space.sample())
            eef_pos = env._eef_xpos
            goal = np.array([0.2, 0.2, 0.8])  # env.desired_goal
            action[:3] = np.clip(goal[0:3]-eef_pos[0:3], -0.05, 0.05)
            observation, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            print("Reward: {}".format(reward))
            if done or t == t_max:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, t / (time.time() - t1)))
