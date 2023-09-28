"""Demo script for the collaborative stacking environment using a failsafe controller.
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
    gripper_aperture
        normalized distance between the gripper fingers in [0, 1]
    vec_eef_to_human_head
        (x,y,z) vector from end effector to human head
    dist_eef_to_human_head
        euclidean distance between human head and end effector
    vec_eef_to_human_lh
        (x,y,z) vector from end effector to human left hand
    dist_eef_to_human_lh
        euclidean distance between human left hand and end effector
    vec_eef_to_human_rh
        (x,y,z) vector from end effector to human right hand
    dist_eef_to_human_rh
        euclidean distance between human right hand and end effector
    next_target_pos
        (x,y,z) absolute coordinates of the next target position.
        If there is currently no next target, the current eef position is returned.
    object_a_pos
        (x,y,z) absolute coordinates of manipulation object A of the robot
    object_b_pos
        (x,y,z) absolute coordinates of manipulation object B of the robot
    object_l_pos
        (x,y,z) absolute coordinates of the cube from the human's left hand
    object_r_pos
        (x,y,z) absolute coordinates of the cube from the human's right hand
    all_object_pos
        12-tuple: absolute coordinates of all objects to stack: concatenation of
            - object_a_pos
            - object_b_pos
            - object_l_pos
            - object_r_pos
    vec_eef_to_object_a
        (x,y,z) vector from end effector to object A (object_a_pos - robot0_eef_pos)
    vec_eef_to_object_b
        (x,y,z) vector from end effector to object B (object_b_pos - robot0_eef_pos)
    vec_eef_to_object_l
        (x,y,z) vector from end effector to object L (object_l_pos - robot0_eef_pos)
    vec_eef_to_object_r
        (x,y,z) vector from end effector to object R (object_r_pos - robot0_eef_pos)
    vec_eef_to_all_objects
        12-tuple: vectors between end effector and all objects to stack: concatenation of
            - vec_eef_to_object_a
            - vec_eef_to_object_b
            - vec_eef_to_object_l
            - vec_eef_to_object_r
    vec_eef_to_object
        (x,y,z) vector from end effector to the object the robot should add to the stack next
    vec_eef_to_target
        (x,y,z) vector from end effector to the next target position
    object_gripped
        (True/False) whether the object has contact to both fingerpads
    vec_eef_to_next_objective
        (x,y,z)
            if an object is gripped (object_gripped):
                vector from end effector to the next target (vec_eef_to_target)
            otherwise:
                vector from end effector to the next object to stack (vec_eef_to_object)
    robot0_proprio-state
        (7-tuple) concatenation of
            - robot0_eef_pos (robot0_proprio-state[0:3])
            - robot0_gripper_qpos (robot0_proprio-state[3:5])
            - robot0_gripper_qvel (robot0_proprio-state[5:7])
    object-state
        (41-tuple) concatenation of
            - gripper_aperture (object-state[0])
            - vec_eef_to_human_lh (object-state[1:4])
            - dist_eef_to_human_lh (object-state[4])
            - vec_eef_to_human_rh (object-state[5:8])
            - dist_eef_to_human_rh (object-state[8])
            - vec_eef_to_human_head (object-state[9:12])
            - dist_eef_to_human_head (object-state[12])
            - object_a_pos (object-state[13:16])
            - object_b_pos (object-state[16:19])
            - object_l_pos (object-state[19:22])
            - object_r_pos (object-state[22:25])
            - all_object_pos (object-state[13:25])
            - vec_eef_to_object_a (object-state[25:28])
            - vec_eef_to_object_b (object-state[28:31])
            - vec_eef_to_object_l (object-state[31:34])
            - vec_eef_to_object_r (object-state[34:37])
            - vec_eef_to_all_objects (object-state[25:37])
            - vec_eef_to_object (object-state[37:40])
            - object_gripped (object-state[40])
    goal-state
        (9-tuple) concatenation of
            - next_target_pos (object-state[0:3])
            - vec_eef_to_target (object-state[3:6])
            - vec_eef_to_next_objective (object-state[6:9])

Author:
    Felix Trost

Changelog:
    14.09.2023 FT File creation
"""
import robosuite as suite
import time
import numpy as np
import glfw

from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
from human_robot_gym.utils.env_util import ExpertObsWrapper
from human_robot_gym.demonstrations.experts import PickPlaceHumanCartExpert
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
        "CollaborativeStackingCart",
        robots="Schunk",  # use Schunk robot
        use_camera_obs=False,  # do not use pixel observations
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        has_renderer=True,  # make sure we can render to the screen
        render_camera=None,
        render_collision_mesh=False,
        reward_shaping=False,  # use dense rewards
        control_freq=5,  # control should happen fast enough so that simulation looks smooth
        hard_reset=False,
        horizon=5000,
        done_at_success=False,
        controller_configs=controller_configs,
        shield_type="PFL",
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.0, 0.0, 0.0],
        verbose=True,
        object_gripped_reward=-0.5,
        second_cube_at_target_reward=0,
        fourth_cube_at_target_reward=0.5,
        human_animation_freq=100,
    )

    env = ExpertObsWrapper(
        env=rsenv,
        agent_keys=[
            "object_gripped",
            "vec_eef_to_next_objective",
            "robot0_gripper_qpos",
            "robot0_gripper_qvel",
        ],
        expert_keys=[
            "object_gripped",
            "vec_eef_to_object",
            "vec_eef_to_target",
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
    expert = PickPlaceHumanCartExpert(
        observation_space=env.observation_space,
        action_space=env.action_space,
        signal_to_noise_ratio=0.99,
    )

    env = CartActionBasedExpertImitationRewardWrapper(
        env=env,
        expert=expert,
        alpha=0.,
        beta=0.95,
        iota_m=0.01,
        iota_g=0.01,
    )

    sc_agent = PickPlaceHumanCartExpert(
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
