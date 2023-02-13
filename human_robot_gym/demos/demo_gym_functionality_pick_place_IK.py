"""Demo script for the pick place environment using a failsafe controller.

Can be used with our provided training function
to train a safe RL agent with work space position actions.

Available observations (possible GymWrapper keys):
    robot0_eef_pos:
        (x,y,z) absolute position the end effector position
    robot0_gripper_qpos
        (l,r) gripper joint position
    robot0_gripper_qvel
        (l,r) gripper joint velocity
    human_head_to_eff
        (x,y,z) vector from end effector to human head
    human_lh_to_eff
        (x,y,z) vector from end effector to human left hand
    human_rh_to_eff
        (x,y,z) vector from end effector to human right hand
    target_pos
        (x,y,z) absolute position of the target
    object_pos
        (x,y,z) absolute position of the object
    object_gripped
        (True/False) whether the object has contact to both fingerpads
    eef_to_object
        (x,y,z) vector from end effector to object (object_pos - robot0_eef_pos)
    object_to_target
        (x,y,z) vector from object to target (target_pos - object_pos)
    eef_to_target
        (x,y,z) vector from end effector to target (target_pos - robot0_eef_pos)
    dist_to_next_objective
        (x,y,z)
            if the object is gripped (object_gripped):
                vector from object to target (object_to_target)
            otherwise:
                vector from end effector to object (eef_to_object)
    robot0_proprio-state
        (7-tuple) concatenation of
            -robot0_eef_pos (robot0_proprio-state[0:3])
            -robot0_gripper_qpos (robot0_proprio-state[3:5])
            -robot0_gripper_qvel (robot0_proprio-state[5:7])
    object-state
        (16-tuple) concatenation of
            -human_head_to_eff (object-state[0:3])
            -human_lh_to_eff (object-state[3:6])
            -human_rh_to_eff (object-state[6:9])
            -object_pos (object-state[9:12])
            -eef_to_object (object-state[12-15])
            -object_gripped (object-state[15])
    goal-state
        (12-tuple) concatenation of
            -target_pos (object-state[0:3])
            -object_to_target (object-state[3:6])
            -eef_to_target (object-state[6:9])
            -dist_to_next_objective (object-state[9:12])

Author:
    Felix Trost

Changelog:
    08.02.23 FT File creation
"""

import robosuite as suite
import time
import numpy as np

from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
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
            "PickPlaceHumanCart",
            robots="Schunk",  # use Schunk robot
            robot_base_offset=[0, 0, 0],
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=True,  # make sure we can render to the screen
            render_camera=None,
            render_collision_mesh=False,
            reward_shaping=False,  # use dense rewards
            control_freq=5,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            horizon=1000,
            done_at_success=False,
            controller_configs=controller_configs,
            use_failsafe_controller=True,
            visualize_failsafe_controller=False,
            visualize_pinocchio=False,
            base_human_pos_offset=[0.0, 0.0, 0.0],
            verbose=True,
        ),
        keys=[
            "object_gripped",
            "dist_to_next_objective",
            "robot0_gripper_qpos",
            "robot0_gripper_qvel",
        ]
    )
    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0
    )
    env = VisualizationWrapper(env)
    action_limits = np.array([[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]])
    env = IKPositionDeltaWrapper(env=env, urdf_file=pybullet_urdf_file, action_limits=action_limits)
    agent = KeyboardControllerAgentCart(env=env)

    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        t = 0
        while True:
            t += 1
            action = env.action_space.sample()
            # testing environment structure
            eef_pos = env.sim.data.site_xpos[env.robots[0].eef_site_id]
            action[:] = agent()
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))