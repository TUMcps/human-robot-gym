"""Demo script for the collaborative lifting environment using a failsafe controller.
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
    vec_eef_to_human_head
        (x,y,z) vector from end effector to human head
    vec_eef_to_human_lh
        (x,y,z) vector from end effector to human left hand
    vec_eef_to_human_rh
        (x,y,z) vector from end effector to human right hand
    board_pos
        (x,y,z) absolute position of the board
    board_quat
        (x,y,z,w) absolute orientation of the board
    board_balance
        The dot product of the board normal and the up vector (0,0,1)
    board_gripped
        (True/False) whether the board has contact to both fingerpads
    vec_eef_to_board
        (x,y,z) vector from end effector to object (object_pos - robot0_eef_pos)
    quat_eef_to_board
        (x,y,z,w) relative quaternion from end effector to object
        quat_eef_to_board = board_quat * robot0_eef_quat^{-1}
    robot0_proprio-state
        (7-tuple) concatenation of
            -robot0_eef_pos (robot0_proprio-state[0:3])
            -robot0_gripper_qpos (robot0_proprio-state[3:5])
            -robot0_gripper_qvel (robot0_proprio-state[5:7])
    object-state
        (23-tuple) concatenation of
            -vec_human_head_to_eef (object-state[0:3])
            -vec_human_lh_to_eef (object-state[3:6])
            -vec_human_rh_to_eef (object-state[6:9])
            -board_pos (object-state[9:12])
            -board_quat (object-state[12:16])
            -vec_eef_to_board (object-state[16:19])
            -quat_eef_to_board (object-state[19:23])
    goal-state
        (2-tuple) concatenation of
            -board_balance (goal-state[0])
            -board_gripped (goal-state[1])
Author:
    Felix Trost

Changelog:
    02.05.2023 FT File creation
"""
import robosuite as suite
import time
import numpy as np
import glfw

import mujoco_py

from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
from human_robot_gym.utils.env_util import ExpertObsWrapper
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)
from human_robot_gym.wrappers.ik_position_delta_wrapper import IKPositionDeltaWrapper
from human_robot_gym.demonstrations.experts import CollaborativeLiftingCartExpert

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
        "CollaborativeLiftingCart",
        robots="Schunk",  # use Schunk robot
        use_camera_obs=False,  # do not use pixel observations
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        has_renderer=True,  # make sure we can render to the screen
        render_camera=None,
        render_collision_mesh=False,
        control_freq=5,  # control should happen fast enough so that simulation looks smooth
        hard_reset=False,
        horizon=1000,
        done_at_success=False,
        controller_configs=controller_configs,
        use_failsafe_controller=True,
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.0, 0.0, 0.0],
        human_rand=[0, 0.0, 0.0],
        verbose=True,
    )

    env = ExpertObsWrapper(
        env=rsenv,
        agent_keys=[
            "vec_eef_to_human_head",
            "vec_eef_to_human_lh",
            "vec_eef_to_human_rh",
        ],
        expert_keys=[
            "vec_eef_to_human_lh",
            "vec_eef_to_human_rh",
            "board_quat",
        ]
    )
    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0,
    )
    env = VisualizationWrapper(env)
    action_limits = np.array([[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]])
    env = IKPositionDeltaWrapper(env=env, urdf_file=pybullet_urdf_file, action_limits=action_limits)
    kb_agent = KeyboardControllerAgentCart(env=env)

    use_kb_agent = False

    def switch_agent():
        global use_kb_agent
        use_kb_agent = not use_kb_agent

    def toggle_board():
        eq_l_id = mujoco_py.functions.mj_name2id(
            rsenv.sim.model, mujoco_py.const.OBJ_EQUALITY, "lh_mocap_object_connect",
        )

        eq_r_id = mujoco_py.functions.mj_name2id(
            rsenv.sim.model, mujoco_py.const.OBJ_EQUALITY, "rh_mocap_object_connect",
        )

        rsenv.sim.model.eq_active[eq_l_id] = not rsenv.sim.model.eq_active[eq_l_id]
        rsenv.sim.model.eq_active[eq_r_id] = not rsenv.sim.model.eq_active[eq_r_id]

    kb_agent.add_keypress_callback(glfw.KEY_O, lambda *_: switch_agent())
    kb_agent.add_keypress_callback(glfw.KEY_B, lambda *_: toggle_board())

    expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env)

    expert = CollaborativeLiftingCartExpert(
        observation_space=env.observation_space,
        action_space=env.action_space,
        board_size=np.array([1.0, 0.5, 0.03]),
        signal_to_noise_ratio=0.0,
    )

    for i_episode in range(20):
        observation = env.reset()
        print(observation)
        t1 = time.time()
        t = 0
        while True:
            t += 1
            expert_observation = expert_obs_wrapper.current_expert_observation

            action = expert(expert_observation)

            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, t / (time.time() - t1)))
