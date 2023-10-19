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

The animation clips can be selected by passing their name as command line argument.

Example:
```
python human_robot_gym/demos/demo_animation_clips.py "CollaborativeLifting/0"
```

Author:
    Felix Trost

Changelog:
    01.10.2023 FT File creation
"""
import robosuite as suite
import time
from robosuite.wrappers import GymWrapper
from robosuite.controllers import load_controller_config

from argparse import ArgumentParser

import human_robot_gym.robots  # noqa: F401
from human_robot_gym.utils.cart_keyboard_controller import AnimationDebugKeyboardController
import human_robot_gym.environments.manipulation.human_env  # noqa: F401
from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument(
        "animation_name",
        type=str,
        nargs="+",
        help="Name(s) of the animation clip(s) to play",
    )
    args = parser.parse_args()

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
            human_animation_names=args.animation_name,
        )
    )

    env = VisualizationWrapper(env)

    # The controller adds monkey patches to the environment's _compute_animation_time and
    # _progress_to_next_animation functions to allow to pause, and step through the animation clips.
    controller = AnimationDebugKeyboardController(
        env=env,
    )

    for i_episode in range(20):
        observation = env.reset()
        t1 = time.time()
        for t in range(100000):
            action = env.action_space.sample()
            controller.add_animation_time_overlay()
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
        print("Episode {}, fps = {}".format(i_episode, 500 / (time.time() - t1)))
