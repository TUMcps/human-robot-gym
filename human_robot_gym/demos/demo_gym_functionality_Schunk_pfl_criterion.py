"""
Demo for testing PFL criterion.
With PFL, robot can move past the human with reduced speed.
With SSM, robot cannot move past the human because it is too close

Author:
Leonardo Maglanoc
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
    robot_conig_path = file_path_completion("models/robots/config/schunk.json")
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    controller_configs = [controller_config]

    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Schunk",  # use Sawyer robot
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
            shield_type="PFL",
            visualize_failsafe_controller=True,
            visualize_pinocchio=False,
            human_animation_names=["Static/tpose"],
            base_human_pos_offset=[1.15, 0, 0.8],
            verbose=True,
            goal_dist=0.0001,
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

    t_max = 200
    t_episode = 10
    for i_episode in range(t_episode):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            action = np.zeros(7)
            pos = np.array([env.sim.data.qpos[x] for x in env.robots[0]._ref_joint_pos_indexes])
            goal = np.array([1.4 * np.sin(4 * t/t_max * 2*np.pi), 1.5, 0.0, 0.0, 0.0, 0.0])
            action[:6] = np.clip(goal-pos, -1, 1)
            observation, reward, done, info = env.step(action)
            time.sleep(0.025)
            if done or t == t_max:
                break
