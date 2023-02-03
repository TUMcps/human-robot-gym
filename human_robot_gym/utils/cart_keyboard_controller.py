"""This file defines keyboard controllers to conveniently define shortcuts.

Includes a keyboard controller agent for the carthesian action space

Author:
    Felix Trost (FT)
"""
import glfw
import numpy as np
from typing import Any, Callable, Literal, Union

from gym import Env
from robosuite.renderers.mujoco.mujoco_py_renderer import MujocoPyRenderer


class KeyboardController:
    """Keyboard controller base class.

    This class allows defining custom keyboard shortcuts.

    Args:
        env (Env): gym environment
    """
    def __init__(
        self,
        env: Env,
    ):
        self._env = env
        self._mj_renderer = self._get_mj_renderer(env)

    def _get_mj_renderer(self, env: Env) -> MujocoPyRenderer:
        """Extracts the MuJoCo renderer from the environment.

        Args:
            env (Env): gym environment containing the renderer

        Returns:
            MujocoPyRenderer: renderer
        """
        return env.unwrapped.viewer

    def add_keypress_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Registers a callback to the event when a key is pressed.

        Args:
            key (int | 'any'): The associated key
                Choosing 'any' disables the standard MjViewer hotkeys
                (see CustomMjViewer)
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute. The second and fourth argument
                are relevant, specifying the key and the action
                (press/release/repeat)
        """
        self._mj_renderer.add_keypress_callback(key, fn)

    def add_keyup_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Registers a callback to the event when a key is released.

        Args:
            key (int | 'any'): The associated key
                Choosing 'any' disables the standard MjViewer hotkeys
                (see CustomMjViewer)
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute. The second and fourth argument
                are relevant, specifying the key and the action
                (press/release/repeat)
        """
        self._mj_renderer.add_keyup_callback(key, fn)

    def add_keyrepeat_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Registers a callback to the event when a key is repeated.

        Args:
            key (int | 'any'): The associated key
                Choosing 'any' disables the standard MjViewer hotkeys
                (see CustomMjViewer)
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute. The second and fourth argument
                are relevant, specifying the key and the action
                (press/release/repeat)
        """
        self._mj_renderer.add_keyrepeat_callback(key, fn)


class KeyboardControllerAgentCart(KeyboardController):
    """Carthesian action space keyboard controller.

    This class allows manual control in cartesian action space.
    Usage disables standard MjViewer shortcuts due to overlaps.

    Shortcuts:
        w: move forward
        s: move backward
        a: move left
        d: move right
        r: move up
        f: move down
        t: open gripper
        g: close gripper

        q: reset environment

    Args:
        env (Env): The environment in which to act
        speed (float): Length of the 3D motion vector
        gripper_torque_scale (float): Magnitude of non-zero torque values
    """

    def __init__(
            self,
            env: Env,
            speed: float,
            gripper_torque_scale: float,
    ):
        super().__init__(env)
        self._speed = speed
        self._gripper_torque_scale = gripper_torque_scale
        self._dir = np.zeros(3)
        self._gripper_torque = np.zeros(1)
        self._add_key_callbacks()

    def _add_key_callbacks(self):
        """Register the keypress callbacks."""
        self.add_keypress_callback("any", self.motion_key_callback)
        self.add_keyup_callback("any", self.motion_key_callback)
        self.add_keyrepeat_callback("any", self.motion_key_callback)

        self.add_keypress_callback(
            glfw.KEY_Q,
            lambda *_: self._env.reset()
        )

    def motion_key_callback(
        self,
        window: Any,
        key: int,
        scancode: Any,
        action: int,
        mods: Any,
    ):
        """Describes how to handle motion key events.

        Args:
            key (int): the keycode from the key event
            action (int): the action associated with the event (press/release/repeat)
        """
        sign = 1 if action in {glfw.PRESS, glfw.REPEAT} else -1

        if key == glfw.KEY_W:
            self._dir[0] -= sign
        elif key == glfw.KEY_S:
            self._dir[0] += sign
        elif key == glfw.KEY_A:
            self._dir[1] -= sign
        elif key == glfw.KEY_D:
            self._dir[1] += sign
        elif key == glfw.KEY_F:
            self._dir[2] -= sign
        elif key == glfw.KEY_R:
            self._dir[2] += sign

        if key == glfw.KEY_T:
            self._gripper_torque -= sign
        elif key == glfw.KEY_G:
            self._gripper_torque += sign

        self._dir = np.clip(self._dir, -1, 1)
        self._gripper_torque = np.clip(self._gripper_torque, -1, 1)

    def __call__(self) -> np.ndarray:
        """Output the current action parameters.

        Returns:
            (np.ndarray) action parameters
        """
        scaled_speed = self._dir * self._speed
        scaled_gripper_torque = self._gripper_torque * self._gripper_torque_scale

        return np.concatenate([scaled_speed, scaled_gripper_torque])
