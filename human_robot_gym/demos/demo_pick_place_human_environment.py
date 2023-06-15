"""Demo script for the pick place environment using a failsafe controller.
Uses a scripted expert to demonstrate the environment functionality.

Pressing 'o' switches between scripted policy and keyboard control.

Can be used with our provided training function
to train a safe RL agent with work space position actions.

Available observations (possible GymWrapper keys):
    robot0_eef_pos:
        (x,y,z) absolute position the end effector position
    robot0_gripper_qpos
        (l,r) gripper joint position
    robot0_gripper_qvel
        (l,r) gripper joint velocity
    eef_to_human_head
        (x,y,z) vector from end effector to human head
    eef_to_human_lh
        (x,y,z) vector from end effector to human left hand
    eef_to_human_rh
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
    vec_to_next_objective
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
            -vec_to_next_objective (object-state[9:12])

Author:
    Felix Trost

Changelog:
    08.02.23 FT File creation
    20.02.23 FT Added scripted policy
"""

import robosuite as suite
import time
import numpy as np
import glfw

from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
from human_robot_gym.utils.env_util import ExpertObsWrapper
from human_robot_gym.demonstrations.experts import PickPlaceExpert
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper
from human_robot_gym.wrappers.action_based_expert_imitation_reward_wrapper import (
    CartActionBasedExpertImitationRewardWrapper
)

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

    rsenv = suite.make(
        "PickPlaceHumanCart",
        robots="Schunk",  # use Schunk robot
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
        done_at_collision=True,
        controller_configs=controller_configs,
        use_failsafe_controller=True,
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.0, 0.0, 0.0],
        verbose=True,
    )

    env = ExpertObsWrapper(
        env=rsenv,
        agent_keys=[
            "object_gripped",
            "vec_to_next_objective",
            "robot0_gripper_qpos",
            "robot0_gripper_qvel",
        ],
        expert_keys=[
            "object_gripped",
            "eef_to_object",
            "eef_to_target",
            "robot0_gripper_qpos",
        ]
    )
    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0,
    )
    env = VisualizationWrapper(env)
    action_limits = np.array([[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]])
    env = IKPositionDeltaWrapper(env=env, urdf_file=pybullet_urdf_file, action_limits=action_limits)
    kb_agent = KeyboardControllerAgentCart(env=env)
    expert = PickPlaceExpert(
        observation_space=env.observation_space,
        action_space=env.action_space,
        signal_to_noise_ratio=0.99,
    )

    env = CartActionBasedExpertImitationRewardWrapper(
        env=env,
        expert=expert,
        alpha=0.1,
        beta=0.95,
        iota_m=0.01,
        iota_g=0.01,
    )

    sc_agent = PickPlaceExpert(
        observation_space=env.observation_space,
        action_space=env.action_space,
        signal_to_noise_ratio=0.98,
    )

    use_kb_agent = False

    def switch_agent():
        global use_kb_agent
        use_kb_agent = not use_kb_agent

    kb_agent.add_keypress_callback(glfw.KEY_O, lambda *_: switch_agent())

    expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env)

    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        t = 0
        while True:
            t += 1
            expert_observation = expert_obs_wrapper.current_expert_observation

            action = kb_agent() if use_kb_agent else sc_agent(expert_observation)

            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, t / (time.time() - t1)))
