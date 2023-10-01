"""This script can be used to debug animation clips in an human environment.
This can be useful for example to select keyframes to pinpoint transitions between phases in the animation.

It allows to pause, rewind and fast forward the animation clips.

Hotkeys:
    - space: pause/resume animation
    - left arrow: step back 5 frame (when paused)
    - right arrow: step forward 5 frame (when paused)
    - ctrl + left arrow: step back 25 frames (when paused)
    - ctrl + right arrow: step forward 25 frames (when paused)
    - alt + left arrow: step back 1 frame (when paused)
    - alt + right arrow: step forward 1 frame (when paused)

The current animation time is displayed in the top right corner of the screen.

Author:
    Felix Trost

Changelog:
    01.10.2023 FT File creation
"""
import robosuite as suite
import time
from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

import glfw
import mujoco_py

import human_robot_gym.robots  # noqa: F401
from human_robot_gym.utils.cart_keyboard_controller import KeyboardControllerAgentCart
import human_robot_gym.environments.manipulation.human_env  # noqa: F401
from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper


if __name__ == "__main__":
    controller_config = dict()
    controller_conig_path = file_path_completion(
        "controllers/failsafe_controller/config/failsafe.json"
    )
    robot_conig_path = file_path_completion("models/robots/config/schunk.json")
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    controller_configs = [controller_config]

    # Notice how the environment is wrapped by the wrapper
    env = GymWrapper(
        suite.make(
            "ReachHuman",
            robots="Schunk",  # use Sawyer robot
            use_camera_obs=False,  # do not use pixel observations
            has_offscreen_renderer=False,  # not needed since not using pixel obs
            has_renderer=True,  # make sure we can render to the screen
            render_camera=None,
            control_freq=20,  # control should happen fast enough so that simulation looks smooth
            hard_reset=False,
            controller_configs=controller_configs,
            verbose=True,
            human_animation_names=[
                "CollaborativeHammering/12",
            ]
        )
    )

    env = VisualizationWrapper(env)

    controller = KeyboardControllerAgentCart(
        env=env,
        speed=0.1,
        gripper_torque_scale=1,
    )

    delayed_control_time = 0
    control_pressed = False
    alt_pressed = False
    control_time_delay_start = 0

    original_animation_time_fn = env.unwrapped._compute_animation_time
    original_progress_animation = env.unwrapped._progress_to_next_animation

    def animation_time_fn(control_time):
        global delayed_control_time
        global control_time_delay_start
        control_time_delay_start = control_time
        print(original_animation_time_fn(control_time - delayed_control_time))
        return int(original_animation_time_fn(control_time - delayed_control_time))

    def progress_animation_fn(animation_start_time):
        global delayed_control_time
        global control_time_delay_start
        control_time_delay_start = 0
        delayed_control_time = 0
        return original_progress_animation(animation_start_time)

    def static_animation_time_fn(control_time):
        global delayed_control_time
        global control_time_delay_start
        delayed_control_time += control_time - control_time_delay_start
        control_time_delay_start = control_time
        print(env.unwrapped.animation_time)
        return int(env.unwrapped.animation_time)

    def change_animation_time(offset):
        if env.unwrapped._compute_animation_time == static_animation_time_fn:
            if alt_pressed:
                adjusted_offset = offset
            elif control_pressed:
                adjusted_offset = 25 * offset
            else:
                adjusted_offset = 5 * offset
            env.unwrapped.animation_time += adjusted_offset

    env.unwrapped._compute_animation_time = animation_time_fn
    env.unwrapped._progress_to_next_animation = progress_animation_fn

    def toggle_animation_time_fn(*_):
        if env.unwrapped._compute_animation_time == static_animation_time_fn:
            env.unwrapped._compute_animation_time = animation_time_fn
        else:
            env.unwrapped._compute_animation_time = static_animation_time_fn

    def toggle_ctrl_pressed(value):
        global control_pressed
        control_pressed = value

    def toggle_alt_pressed(value):
        global alt_pressed
        alt_pressed = value

    controller.add_keypress_callback(glfw.KEY_SPACE, toggle_animation_time_fn)
    controller.add_keypress_callback(glfw.KEY_LEFT, lambda *_: change_animation_time(-1))
    controller.add_keypress_callback(glfw.KEY_RIGHT, lambda *_: change_animation_time(1))
    controller.add_keypress_callback(glfw.KEY_LEFT_CONTROL, lambda *_: toggle_ctrl_pressed(True))
    controller.add_keyup_callback(glfw.KEY_LEFT_CONTROL, lambda *_: toggle_ctrl_pressed(False))
    controller.add_keypress_callback(glfw.KEY_LEFT_ALT, lambda *_: toggle_alt_pressed(True))
    controller.add_keyup_callback(glfw.KEY_LEFT_ALT, lambda *_: toggle_alt_pressed(False))

    env.unwrapped.viewer.viewer._hide_overlay = False

    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(100000):
            env.unwrapped.viewer.viewer.add_overlay(
                mujoco_py.generated.const.GRID_TOPRIGHT,
                f"Animation Time_ {env.unwrapped.animation_time}", "")
            action = env.action_space.sample()  # np.array([0, 1, 0, 0, 0, 0, 0, 0])
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))
