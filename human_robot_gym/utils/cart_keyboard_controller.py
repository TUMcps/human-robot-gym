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

import mujoco

from gymnasium import Env
from robosuite.renderers.mjviewer.mjviewer_renderer import MjviewerRenderer

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
        self.env = env
        self._key_callbacks = {}
        self._setup_key_callback()

    @property
    def _mj_renderer(self) -> MjviewerRenderer:
        """Extract the MuJoCo renderer from the environment.

        Args:
            env (Env): gym environment containing the renderer

        Returns:
            MjviewerRenderer: renderer
        """
        return self.env.unwrapped.viewer

    @property
    def _viewer(self):
        """Extract the MuJoCo viewer from the environment.

        Args:
            env (Env): gym environment containing the viewer

        Returns:
            mujoco.viewer.MjViewer: viewer
        """
        return self._mj_renderer.viewer

    def _setup_key_callback(self):
        """Set up the unified key callback system for the new MuJoCo viewer API."""
        # Track key states since MuJoCo viewer only provides press events
        if not isinstance(self._mj_renderer, MjviewerRenderer):
            raise ValueError("The keyboard controller is only supported for MjviewerRenderer. \
                              Set renderer='mjviewer' when creating the environment.")
        self._key_states = {}

        def unified_key_callback(keycode):
            """Unified callback that handles all key events and dispatches to registered callbacks."""
            # For movement controls, we need to simulate press/release behavior
            # Since MuJoCo viewer only calls on key press, we'll use PRESS for all events
            # and let the motion callback handle the state changes
            action = glfw.PRESS

            # Check for specific key callbacks
            if keycode in self._key_callbacks:
                for callback in self._key_callbacks[keycode]:
                    callback(None, keycode, None, action, None)

            # Check for "any" key callbacks
            if "any" in self._key_callbacks:
                for callback in self._key_callbacks["any"]:
                    callback(None, keycode, None, action, None)

        # Store the unified callback for later use
        self._unified_key_callback = unified_key_callback

        # Patch the renderer's update method to include key callback on first viewer creation
        if not hasattr(self._mj_renderer, "_original_update"):
            self._mj_renderer._original_update = self._mj_renderer.update

        def patched_update():
            if self._viewer is not None and not hasattr(self._viewer, "custom_key_callback_initialized"):
                self._viewer.close()
                self._mj_renderer.viewer = None  # Force re-creation of the viewer with custom callback
            if self._viewer is None:
                # Create the viewer with our key callback
                self._mj_renderer.viewer = mujoco.viewer.launch_passive(
                    self._mj_renderer.env.sim.model._model,
                    self._mj_renderer.env.sim.data._data,
                    show_left_ui=False,
                    show_right_ui=False,
                    key_callback=self._unified_key_callback,
                )

                # Disable built-in viewer shortcuts that might interfere
                if hasattr(self._viewer, "enable_keyboard_shortcuts"):
                    self._viewer.enable_keyboard_shortcuts = False

                # Apply the same configuration as the original update method
                self._viewer.opt.geomgroup[0] = 0

                if self._mj_renderer.camera_config is not None:
                    self._viewer.cam.lookat = self._mj_renderer.camera_config["lookat"]
                    self._viewer.cam.distance = self._mj_renderer.camera_config["distance"]
                    self._viewer.cam.azimuth = self._mj_renderer.camera_config["azimuth"]
                    self._viewer.cam.elevation = self._mj_renderer.camera_config["elevation"]

                if self._mj_renderer.camera_id is not None:
                    if self._mj_renderer.camera_id >= 0:
                        self._viewer.cam.type = 2
                        self._viewer.cam.fixedcamid = self._mj_renderer.camera_id
                    else:
                        self._viewer.cam.type = 0
                self._viewer.custom_key_callback_initialized = True

            # Always call sync (this is the main part of every update call)
            self._viewer.sync()

        # Replace the update method with our patched version
        self._mj_renderer.update = patched_update
        self._mj_renderer.update()

    def update(self):
        """Update the renderer initialization if necessary."""
        if self._viewer is None or not hasattr(self._viewer, "custom_key_callback_initialized"):
            self._setup_key_callback()

    def add_keypress_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Register a callback to the event when a key is pressed.

        Args:
            key (int | 'any'): The associated key
                Choosing 'any' disables the standard MjViewer hotkeys
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute. The second and fourth argument
                are relevant, specifying the key and the action
                (press/release/repeat)
        """
        if key not in self._key_callbacks:
            self._key_callbacks[key] = []
        self._key_callbacks[key].append(fn)

    def add_keyup_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Register a callback to the event when a key is released.

        Note: In the new MuJoCo viewer API, keyup events are handled
        as keypress events. This method exists for backward compatibility.

        Args:
            key (int | 'any'): The associated key
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute
        """
        # For backward compatibility, treat keyup as keypress
        self.add_keypress_callback(key, fn)

    def add_keyrepeat_callback(
        self,
        key: Union[int, Literal["any"]],
        fn: Callable[[Any, int, Any, int, Any], None],
    ):
        """Register a callback to the event when a key is repeated.

        Note: In the new MuJoCo viewer API, key repeat events are handled
        as keypress events. This method exists for backward compatibility.

        Args:
            key (int | 'any'): The associated key
            fn ((Any, int, Any, int, Any)-> None):
                the callback to execute
        """
        # For backward compatibility, treat keyrepeat as keypress
        self.add_keypress_callback(key, fn)


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

        # Track currently pressed keys for continuous movement
        self._pressed_keys = set()

        self._add_key_callbacks()

    def _add_key_callbacks(self):
        """Register the keypress callbacks."""
        self.add_keypress_callback("any", self.motion_key_callback)
        self.add_keyup_callback("any", self.motion_key_callback)
        self.add_keyrepeat_callback("any", self.motion_key_callback)

        self.add_keypress_callback(
            glfw.KEY_Q,
            lambda *_: self.env.reset()
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
        super().update()

        self._dir = np.clip(self._dir, -1, 1)
        self._gripper_torque = np.clip(self._gripper_torque, -1, 1)
        scaled_speed = self._dir * self._speed
        scaled_gripper_torque = self._gripper_torque * self._gripper_torque_scale

        # Reset direction and gripper torque for next call
        self._dir = np.zeros(3)
        self._gripper_torque = np.zeros(1)

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
        self.env.unwrapped.viewer.viewer._hide_overlay = False

        self.env_compute_animation_time_fn = env.unwrapped._compute_animation_time
        self.env_progress_animation_fn = env.unwrapped._progress_to_next_animation

        # Monkey patch to enable pausing and resuming the animation
        self.env.unwrapped._compute_animation_time = self._compute_animation_time
        self.env.unwrapped._progress_to_next_animation = self._progress_to_next_animation

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
        def left_ctrl_press_callback(window, key, scancode, action, mods):
            self._on_left_ctrl_state_changed(True)

        def left_ctrl_release_callback(window, key, scancode, action, mods):
            self._on_left_ctrl_state_changed(False)

        def right_ctrl_press_callback(window, key, scancode, action, mods):
            self._on_right_ctrl_state_changed(True)

        def right_ctrl_release_callback(window, key, scancode, action, mods):
            self._on_right_ctrl_state_changed(False)

        def left_alt_press_callback(window, key, scancode, action, mods):
            self._on_left_alt_state_changed(True)

        def left_alt_release_callback(window, key, scancode, action, mods):
            self._on_left_alt_state_changed(False)

        def right_alt_press_callback(window, key, scancode, action, mods):
            self._on_right_alt_state_changed(True)

        def right_alt_release_callback(window, key, scancode, action, mods):
            self._on_right_alt_state_changed(False)

        self.add_keypress_callback(glfw.KEY_LEFT_CONTROL, left_ctrl_press_callback)
        self.add_keyup_callback(glfw.KEY_LEFT_CONTROL, left_ctrl_release_callback)
        self.add_keypress_callback(glfw.KEY_RIGHT_CONTROL, right_ctrl_press_callback)
        self.add_keyup_callback(glfw.KEY_RIGHT_CONTROL, right_ctrl_release_callback)
        self.add_keypress_callback(glfw.KEY_LEFT_ALT, left_alt_press_callback)
        self.add_keyup_callback(glfw.KEY_LEFT_ALT, left_alt_release_callback)
        self.add_keypress_callback(glfw.KEY_RIGHT_ALT, right_alt_press_callback)
        self.add_keyup_callback(glfw.KEY_RIGHT_ALT, right_alt_release_callback)

        # Toggle between play and pause
        def toggle_pause_callback(window, key, scancode, action, mods):
            self._toggle_paused()

        self.add_keypress_callback(glfw.KEY_SPACE, toggle_pause_callback)

        # Step forward and backward during pause
        def step_backward_callback(window, key, scancode, action, mods):
            self._modify_animation_time(False)

        def step_forward_callback(window, key, scancode, action, mods):
            self._modify_animation_time(True)

        self.add_keypress_callback(glfw.KEY_LEFT, step_backward_callback)
        self.add_keypress_callback(glfw.KEY_RIGHT, step_forward_callback)

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
            self.env_compute_animation_time_fn(
                (control_time - self._control_time_delay) % self.env.unwrapped.human_animation_length
            )
        )

    def _progress_to_next_animation(self, animation_start_time: int):
        """Wrapper function to update internal variables when a new animation is selected."""
        self._control_time_delay = 0
        self._control_time_delay_start = animation_start_time
        return self.env_progress_animation_fn(animation_start_time)

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
        self.env.unwrapped.viewer.viewer.add_overlay(
            mujoco.mjtGridPos.mjGRID_TOPRIGHT.value,
            f"Animation Time_ {self.env.unwrapped.animation_time}",
            "",
        )
