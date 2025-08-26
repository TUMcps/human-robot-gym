"""This script shows an example of the Schunk robot being safely controlled in an human environment.

For instance, this can be used with our provided training function to train a safe RL agent.
"""
import cProfile
import robosuite as suite
import numpy as np  # noqa: F401

from robosuite.wrappers import GymWrapper

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper  # noqa: F401
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)


def run_episode(env):
    env.reset()
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        if done:
            break


if __name__ == "__main__":
    # Notice how the environment is wrapped by the wrapper
    # Load custom failsafe controller config
    failsafe_config_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    robot_config_path = file_path_completion("models/robots/config/schunk.json")

    # Load the failsafe controller config from file
    import json
    with open(failsafe_config_path, 'r') as f:
        failsafe_config = json.load(f)

    # Load robot-specific limits
    with open(robot_config_path, 'r') as f:
        robot_config = json.load(f)

    # Merge robot limits into failsafe config
    controller_config = {'body_parts': {'right': {}}}
    controller_config['body_parts']['right'] = merge_configs(failsafe_config['body_parts']['right'], robot_config)
    controller_configs = [controller_config]

    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Schunk",  # use Sawyer robot
            robot_base_offset=[0, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=False,  # make sure we can render to the screen
            render_camera=None,
            renderer="mjviewer",
            render_collision_mesh=False,
            reward_shaping=False,  # use dense rewards
            control_freq=10,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            horizon=1000,
            controller_configs=controller_configs,
            shield_type="SSM",
            visualize_failsafe_controller=False,
            visualize_pinocchio=False,
            base_human_pos_offset=[0.0, 0.0, 0.0],
        )
    )

    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0
    )

    cProfile.run("run_episode(env)", sort="cumtime")
