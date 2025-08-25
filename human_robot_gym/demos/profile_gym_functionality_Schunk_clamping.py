"""This script shows an example of the Schunk robot being safely controlled in an human environment.

For instance, this can be used with our provided training function to train a safe RL agent.
"""
import cProfile
import robosuite as suite
import numpy as np  # noqa: F401

from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper  # noqa: F401
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)


def run_episode(env):
    env.reset()
    env.desired_goal = np.array([0, 1.5, -np.pi / 2 + 1.5, 0, -np.pi / 2, 0])
    for i in range(100):
        action = env.action_space.sample()
        pos = np.array([env.sim.data.qpos[x] for x in env.robots[0]._ref_joint_pos_indexes])
        goal = env.desired_goal
        action[:pos.shape[0]] = np.clip(goal-pos, -0.5, 0.5)
        observation, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        if done:
            break


if __name__ == "__main__":
    # Notice how the environment is wrapped by the wrapper
    controller_config = dict()
    controller_conig_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    robot_conig_path = file_path_completion("models/robots/config/schunk.json")
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
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
            reward_shaping=True,  # use dense rewards
            control_freq=5,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            horizon=1000,
            controller_configs=controller_configs,
            use_failsafe_controller=True,
            visualize_failsafe_controller=True,
            visualize_pinocchio=False,
            base_human_pos_offset=[1.3, -2.0, -0.50],
            verbose=True,
            goal_dist=0.0001,
            human_rand=[0.0, 0.0, 0.0],
            human_animation_names=["Test/test"],
            human_animation_freq=10
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

    cProfile.run("run_episode(env)", sort="cumtime")

"""
   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    5.257    5.257 {built-in method builtins.exec}
        1    0.000    0.000    5.257    5.257 <string>:1(<module>)
        1    0.002    0.002    5.257    5.257 profile_gym_functionality_Schunk_clamping.py:21(run_episode)
      100    0.001    0.000    5.206    0.052 collision_prevention_wrapper.py:39(step)
      100    0.001    0.000    5.152    0.052 gym_wrapper.py:96(step)
      100    0.000    0.000    5.150    0.052 reach_human_env.py:324(step)
      100    0.118    0.001    5.150    0.051 human_env.py:360(step)
    10104    1.993    0.000    1.993    0.000 {method 'forward' of 'mujoco_py.cymj.MjSim' objects}
     5000    0.051    0.000    1.662    0.000 robot_env.py:558(_pre_action)
     5000    0.075    0.000    1.405    0.000 single_arm.py:216(control)
     5000    0.913    0.000    0.913    0.000 {method 'step' of 'mujoco_py.cymj.MjSim' objects}
     5000    0.353    0.000    0.591    0.000 failsafe_controller.py:304(run_controller)
86658/54440    0.164    0.000    0.588    0.000 {built-in method numpy.core._multiarray_umath.implement_array_function}
    15500    0.026    0.000    0.487    0.000 <__array_function__ internals>:177(clip)
    15500    0.031    0.000    0.436    0.000 fromnumeric.py:2085(clip)
     5000    0.100    0.000    0.418    0.000 manipulator.py:16(grip_action)
    15603    0.012    0.000    0.405    0.000 fromnumeric.py:51(_wrapfunc)
    15500    0.027    0.000    0.389    0.000 {method 'clip' of 'numpy.ndarray' objects}
    15500    0.114    0.000    0.362    0.000 _methods.py:126(_clip)
    15002    0.026    0.000    0.278    0.000 robot.py:288(action_dim)
    15002    0.107    0.000    0.252    0.000 single_arm.py:338(action_limits)
     5000    0.067    0.000    0.251    0.000 rethink_gripper.py:43(format_action)
    31000    0.087    0.000    0.190    0.000 _methods.py:92(_clip_dep_is_scalar_nan)
     5000    0.009    0.000    0.171    0.000 base_controller.py:190(clip_torques)
     5001    0.094    0.000    0.157    0.000 base.py:328(_update_observables)
    31416    0.047    0.000    0.138    0.000 <__array_function__ internals>:177(concatenate)
    31000    0.038    0.000    0.102    0.000 <__array_function__ internals>:177(ndim)
     5000    0.018    0.000    0.101    0.000 human_env.py:1393(_human_measurement)
     5000    0.098    0.000    0.098    0.000 human_env.py:686(_collision_detection)
     5000    0.040    0.000    0.083    0.000 human_env.py:1395(<listcomp>)
     5000    0.016    0.000    0.063    0.000 human_env.py:1092(_set_human_measurement)
    70014    0.053    0.000    0.063    0.000 observables.py:214(update)
     5002    0.009    0.000    0.058    0.000 base.py:599(actuators)
5949/5531    0.013    0.000    0.053    0.000 base.py:274(correct_naming)
      100    0.000    0.000    0.053    0.001 collision_prevention_wrapper.py:47(action)
"""
