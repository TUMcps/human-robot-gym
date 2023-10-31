"""This file defines keyboard controllers to conveniently define shortcuts.

Includes a keyboard controller agent for the carthesian action space

Author:
    Felix Trost (FT)

Changelog:
    05.02.23 FT File creation
"""
import glfw
import numpy as np
from typing import Any, Callable, Literal, Union

import mujoco_py

from gym import Env
from robosuite.renderers.mujoco.mujoco_py_renderer import MujocoPyRenderer

from human_robot_gym.environments.manipulation.human_env import HumanEnv


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
        """Extract the MuJoCo renderer from the environment.

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
        """Register a callback to the event when a key is pressed.

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
        """Register a callback to the event when a key is released.

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
        """Register a callback to the event when a key is repeated.

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
            speed: float = 0.1,
            gripper_torque_scale: float = 1,
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
        """Describe how to handle motion key events.

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


class AnimationDebugKeyboardController(KeyboardController):
    """Keyboard controller for debugging animation clips.

    This class allows to pause and step through the animation clips.

    It applies monkey patches to the environment's `_compute_animation_time` and `_progress_to_next_animation` functions
    that wrap the original functions and allow to pause and step through the animation clips.

    Shortcuts:
        space: toggle between play and pause
        left: step backward during pause
        right: step forward during pause
        ctrl + left: step fast backward during pause
        ctrl + right: step fast forward during pause
        alt + left: step a single frame backward during pause
        alt + right: step a single frame forward during pause

    Args:
        env (Env): The environment in which to act. Must be a (wrapped) `HumanEnv`
        default_step_size (int): Number of frames to step when using the arrow keys during pause
        ctrl_step_size (int): Number of frames to step when using the arrow keys and holding ctrl during pause

    Raises:
        [AssertionError: "The environment must be a (wrapped) HumanEnv"]
    """
    def __init__(
        self,
        env: Env,
        default_step_size: int = 5,
        ctrl_step_size: int = 25,
    ):
        assert isinstance(env.unwrapped, HumanEnv), "The environment must be a (wrapped) HumanEnv"

        super().__init__(
            env=env,
        )

        self._left_ctrl_pressed = False
        self._right_ctrl_pressed = False
        self._left_alt_pressed = False
        self._right_alt_pressed = False

        self._default_step_size = default_step_size
        self._ctrl_step_size = ctrl_step_size

        self._paused = False

        # Show the overlay to display the current animation time
        self._env.unwrapped.viewer.viewer._hide_overlay = False

        self._env_compute_animation_time_fn = env.unwrapped._compute_animation_time
        self._env_progress_animation_fn = env.unwrapped._progress_to_next_animation

        # Monkey patch to enable pausing and resuming the animation
        self._env.unwrapped._compute_animation_time = self._compute_animation_time
        self._env.unwrapped._progress_to_next_animation = self._progress_to_next_animation

        self._control_time_delay = 0
        self._control_time_delay_start = 0

        self._add_key_callbacks()

    @property
    def ctrl_pressed(self):
        """Check if one of the ctrl keys is pressed."""
        return self._left_ctrl_pressed or self._right_ctrl_pressed

    @property
    def alt_pressed(self):
        """Check if one of the alt keys is pressed."""
        return self._left_alt_pressed or self._right_alt_pressed

    def _add_key_callbacks(self):
        """Add callbacks for the defined keyboard shortcuts."""
        # Callbacks for ctrl and alt keys
        self.add_keypress_callback(
            glfw.KEY_LEFT_CONTROL,
            lambda *_: self._on_left_ctrl_state_changed(True)
        )
        self.add_keyup_callback(
            glfw.KEY_LEFT_CONTROL,
            lambda *_: self._on_left_ctrl_state_changed(False)
        )
        self.add_keypress_callback(
            glfw.KEY_RIGHT_CONTROL,
            lambda *_: self._on_right_ctrl_state_changed(True)
        )
        self.add_keyup_callback(
            glfw.KEY_RIGHT_CONTROL,
            lambda *_: self._on_right_ctrl_state_changed(False)
        )
        self.add_keypress_callback(
            glfw.KEY_LEFT_ALT,
            lambda *_: self._on_left_alt_state_changed(True)
        )
        self.add_keyup_callback(
            glfw.KEY_LEFT_ALT,
            lambda *_: self._on_left_alt_state_changed(False)
        )
        self.add_keypress_callback(
            glfw.KEY_RIGHT_ALT,
            lambda *_: self._on_right_alt_state_changed(True)
        )
        self.add_keyup_callback(
            glfw.KEY_RIGHT_ALT,
            lambda *_: self._on_right_alt_state_changed(False)
        )

        # Toggle between play and pause
        self.add_keypress_callback(
            glfw.KEY_SPACE,
            lambda *_: self._toggle_paused()
        )

        # Step forward and backward during pause
        self.add_keypress_callback(
            glfw.KEY_LEFT,
            lambda *_: self._modify_animation_time(-1)
        )
        self.add_keypress_callback(
            glfw.KEY_RIGHT,
            lambda *_: self._modify_animation_time(1)
        )

    def _on_left_ctrl_state_changed(self, pressed: bool):
        """Update the state of the left ctrl key."""
        self._left_ctrl_pressed = pressed

    def _on_right_ctrl_state_changed(self, pressed: bool):
        """Update the state of the right ctrl key."""
        self._right_ctrl_pressed = pressed

    def _on_left_alt_state_changed(self, pressed: bool):
        """Update the state of the left alt key."""
        self._left_alt_pressed = pressed

    def _on_right_alt_state_changed(self, pressed: bool):
        """Update the state of the right alt key."""
        self._right_alt_pressed = pressed

    def _toggle_paused(self):
        """Toggle between play and pause."""
        self._paused = not self._paused

    def _compute_animation_time(self, control_time: int):
        """Wrapper function to compute the animation time.

        This function replaces the original function of the environment
        and calls the original function with a modified time argument.

        This allows to pause and step through the animation clips.

        Args:
            control_time (int): The current control time
        """
        if self._paused:
            self._control_time_delay += control_time - self._control_time_delay_start

        self._control_time_delay_start = control_time

        return int(
            self._env_compute_animation_time_fn(
                (control_time - self._control_time_delay) % self._env.unwrapped.human_animation_length
            )
        )

    def _progress_to_next_animation(self, animation_start_time: int):
        """Wrapper function to update internal variables when a new animation is selected."""
        self._control_time_delay = 0
        self._control_time_delay_start = animation_start_time
        return self._env_progress_animation_fn(animation_start_time)

    def _modify_animation_time(self, forward: bool):
        """Apply an offset to internal variables to modify the animation time during pause."""
        offset = 1 if forward else -1

        if self._paused:
            if self.alt_pressed:
                adjusted_offset = offset
            elif self.ctrl_pressed:
                adjusted_offset = self._ctrl_step_size * offset
            else:
                adjusted_offset = self._default_step_size * offset
            self._control_time_delay -= adjusted_offset

    def add_animation_time_overlay(self):
        """Display the current animation time in the top right corner of the screen."""
        self._env.unwrapped.viewer.viewer.add_overlay(
            mujoco_py.generated.const.GRID_TOPRIGHT,
            f"Animation Time_ {self._env.unwrapped.animation_time}",
            "",
        )
