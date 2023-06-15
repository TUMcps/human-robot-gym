"""Demo script for position control in a failsafe control environment.

This script shows an example of the Schunk robot being safely controlled
with the inverse kinematics wrapper in a human environment.
The environment adapts the functionality from ReachHuman to change the
active observable, set an initial joint configuration and set a position goal.

Can be used with our provided training function
to train a safe RL agent with work space position actions.

Note that some goals are not reachable
and the motion remains well-behaved at workspace boundaries.

Author: Rafael Cabral
"""

import robosuite as suite
import cProfile
import numpy as np

from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_cartesian_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper


def run_episode(env):
    env.reset()
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            break


if __name__ == "__main__":
    pybullet_urdf_file = file_path_completion(
        "models/assets/robots/schunk/robot_pybullet.urdf"
    )
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
            "ReachHumanCart",
            robots="Schunk",  # use Schunk robot
            robot_base_offset=[0, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=False,  # make sure we can render to the screen
            render_camera=None,
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
            init_joint_pos=np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, 0]),
        )
    )
    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0
    )

    env = IKPositionDeltaWrapper(env=env, urdf_file=pybullet_urdf_file)

    cProfile.run("run_episode(env)", sort="cumtime")
