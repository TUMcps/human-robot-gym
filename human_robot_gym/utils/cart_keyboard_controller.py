import glfw
import numpy as np

from gym import Env
from robosuite.renderers.mujoco.mujoco_py_renderer import MujocoPyRenderer

class CartKeyboardController:
    def __init__(
            self, 
            env: Env,
            speed: float, 
            gripper_torque_scale: float,
    ):
        self._speed = speed
        self._gripper_torque_scale = gripper_torque_scale
        self._dir = np.zeros(3)
        self._gripper_torque = np.zeros(1)
        self._env = env
        self._mj_renderer = self._get_mj_renderer(env)
        self._add_key_events()    

    def _get_mj_renderer(self, env: Env)-> MujocoPyRenderer:
        return env.unwrapped.viewer

    def _add_key_events(self):
        self._mj_renderer.add_keypress_callback("any", self.key_event)
        self._mj_renderer.add_keyup_callback("any", self.key_event)
        self._mj_renderer.add_keyrepeat_callback("any", self.key_event)

        self._mj_renderer.add_keypress_callback(
            glfw.KEY_Q, 
            lambda *_: self._env.reset()
        )

    def key_event(self, window, key, scancode, action, mods):
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

    def add_keypress_callback(self, key, fn):
        self._mj_renderer.add_keypress_callback(key, fn)

    def add_keyup_callback(self, key, fn):
        self._mj_renderer.add_keyup_callback(key, fn)

    def add_keyrepeat_callback(self, key, fn):
        self._mj_renderer.add_keyrepeat_callback(key, fn)

    def __call__(self) -> np.ndarray:
        scaled_speed = self._dir * self._speed
        scaled_gripper_torque = self._gripper_torque * self._gripper_torque_scale

        return np.concatenate([scaled_speed, scaled_gripper_torque])
