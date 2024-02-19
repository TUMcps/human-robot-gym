"""This script shows an example of the Schunk robot being safely controlled in the `ReachHuman`.

For instance, this can be used with our provided training function to train a safe RL agent.

Contributors:
    Felix Trost (FT)

Changelog:
    16.06.23 FT Actions now provided by expert policy
"""

import robosuite as suite
import time
import numpy as np  # noqa: F401

from robosuite.controllers import load_controller_config
from robosuite.wrappers import GymWrapper

from human_robot_gym.demonstrations.experts import ReachHumanExpert
from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)
from human_robot_gym.models.robots.manipulators.kinova3_robot import load_kinova3pinn_models
load_kinova3pinn_models()
from human_robot_gym.wrappers.expert_obs_wrapper import ExpertObsWrapper
import hubo_models.environments

if __name__ == "__main__":
    # Notice how the environment is wrapped by the wrapper
    controller_config = dict()
    controller_conig_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    # robot_conig_path = file_path_completion("models/robots/config/schunk.json")
    robot_conig_path = file_path_completion("models/robots/config/kinova3.json")
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    controller_configs = [controller_config]

    env = GymWrapper(  # ExpertObsWrapper
        suite.make(
            "ReachHuman", #  ReachHuman
            robots="Kinova3Pinn",  # use Schunk robot Kinova3Pinn
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
            # base_human_pos_offset=[1.0, 0.0, 0.0],
            verbose=True,
            goal_dist=0.0001,
            # human_rand=[1.0, 0.5, 0.2]
        ),
        keys=[
            "object-state",
            "robot0_proprio-state",
            "goal_difference"
        ] #,  # agent_keys keys
        # expert_keys=[
        #     "goal_difference"
        # ]
    )

    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=2
    )

    env = VisualizationWrapper(env)

    # expert = ReachHumanExpert(
    #     observation_space=env.observation_space,
    #     action_space=env.action_space,
    #     signal_to_noise_ratio=0.99,
    # )
    #
    # expert_obs_wrapper = ExpertObsWrapper.get_from_wrapped_env(env=env)

    t_max = 100
    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(t_max):
            # expert_observation = expert_obs_wrapper.current_expert_observation
            # action = expert(expert_observation)
            # print("action: ", action, "\n")

            action = env.action_space.sample()
            pos = np.array([env.sim.data.qpos[x] for x in env.robots[0]._ref_joint_pos_indexes])
            goal = env.desired_goal
            action[:pos.shape[0]] = np.clip(goal-pos, -0.5, 0.5) #goal-pos  action[0:7]
            # print("goal: ", goal, "\n")
            print("action: ", action, "\n")
            observation, reward, done, info = env.step(action)
            # print("Reward: {}".format(reward))
            if done or t == t_max:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, t / (time.time() - t1)))
