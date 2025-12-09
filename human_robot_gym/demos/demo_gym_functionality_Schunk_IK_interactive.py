"""Demo script for position control in a failsafe control environment.

This script shows an example of the Schunk robot being safely controlled
with the inverse kinematics wrapper in a human environment.
The environment adapts the functionality from ReachHuman to change the
active observable, set an initial joint configuration and set a position goal.
Control using the keyboard with the following shortcuts:
w: move forward
s: move backward
a: move left
d: move right
r: move up
f: move down
t: open gripper
g: close gripper

q: reset environment

Note that some goals are not reachable
and the motion remains well-behaved at workspace boundaries.

Author:
    Felix Trost

Changelog:
    05.02.23 FT File creation
"""
import robosuite as suite
import time
import numpy as np

from robosuite.wrappers import GymWrapper


from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
import human_robot_gym.environments.manipulation.reach_human_cartesian_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper

if __name__ == "__main__":
    pybullet_urdf_file = file_path_completion(
        "models/assets/robots/schunk/robot_pybullet.urdf"
    )
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

    rsenv = suite.make(
        "ReachHumanCart",
        robots="Schunk",  # use Schunk robot
        robot_base_offset=[0.0, 0, 0],
        use_camera_obs=False,  # do not use pixel observations
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        has_renderer=True,  # make sure we can render to the screen
        render_camera=None,
        renderer="mjviewer",
        render_collision_mesh=False,
        reward_shaping=False,  # use dense rewards
        control_freq=5,  # control should happen fast enough so that simulation looks smooth
        hard_reset=False,
        horizon=1000,
        controller_configs=controller_configs,
        shield_type="SSM",
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.0, 0.0, 0.0],
        init_joint_pos=np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, 0]),
        verbose=True,
    )
    rsenv = CollisionPreventionWrapper(
        env=rsenv,
        collision_check_fn=rsenv.check_collision_action,
        replace_type=0,
    )
    action_limits = np.array([[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]])
    rsenv = IKPositionDeltaWrapper(
        env=rsenv,
        urdf_file=pybullet_urdf_file,
        action_limits=action_limits
    )
    rsenv = VisualizationWrapper(rsenv)
    env = GymWrapper(rsenv, keys=[
            "object-state",
            "goal_difference"
          ])

    agent = KeyboardControllerAgentCart(
        env=env,
        speed=0.1,
        gripper_torque_scale=1,
    )

    t_max = 300
    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            # testing environment structure
            eef_pos = env._eef_xpos
            goal = env.desired_goal
            action = agent()
            observation, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            if done or t == t_max:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))
