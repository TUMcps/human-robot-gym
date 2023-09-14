"""This file defines a collaborative stacking task for a single robot arm in a human environment.

The objective of this task is to build a stack of cubes in collaboration with a human partner.
The human partner will place the first cube on the table, from there on robot and human will alternate placing cubes.
The task is divided into 6 phases:
    1. Approach: The human approaches the table.
    2. Place first: The human places the first cube on the table.
    3. Wait for second: The human waits for the robot to place the second cube.
    4. Place third: The human places the third cube on the stack.
    5. Wait for fourth: The human waits for the robot to place the fourth cube.
    6. Retreat: The human retreats from the table.

When using a fixed horizon, these phases are repeated until the horizon is reached.
Otherwise, the episode ends when the animation is finished.

Reward is given once the task is done, i.e. the animation of the human is finished.
Optionally, the sparse reward may be augmented by sub-objective rewards for:
    - An object being gripped
    - The second cube being at its target
    - The fourth cube being at its target

Tasks are terminated if the stack is toppled over or if a collision occurs (only if `done_at_collision = True`).

Author:
    Felix Trost (FT)

Changelog:
    12.07.23 FT File created
"""
from typing import Any, Dict, List, Optional, OrderedDict, Tuple, Union
from enum import Enum
from dataclasses import asdict, dataclass

import numpy as np

from scipy.spatial.transform import Rotation

import mujoco_py

import xml.etree.ElementTree as ET

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import ObjectPositionSampler

from human_robot_gym.environments.manipulation.human_env import COLLISION_TYPE
from human_robot_gym.environments.manipulation.human_env import HumanEnv, HumanEnvState
from human_robot_gym.utils.mjcf_utils import quat_to_rot, rot_to_quat, xml_path_completion
from human_robot_gym.utils.animation_utils import layered_sin_modulations, sample_animation_loop_properties


class CollaborativeStackingPhase(Enum):
    """Enum for the different phases of the collaborative stacking task."""
    APPROACH = 0
    PLACE_FIRST = 1
    WAIT_FOR_SECOND = 2
    PLACE_THIRD = 3
    WAIT_FOR_FOURTH = 4
    RETREAT = 5
    COMPLETE = 6


@dataclass
class CollaborativeStackingEnvState(HumanEnvState):
    """Dataclass for encapsulating the state of the `CollaborativeStackingCart` environment.

    Extends the `HumanEnvState` dataclass to include all variables necessary
    to restore the state of the `CollaborativeStackingCart` environment.

    Attributes:
        sim_state (np.ndarray): State of the mujoco simulation
        human_animation_ids (np.ndarray): List of sampled human animation ids. During the episode this list is
            iterated over and the corresponding human animation is played.
        human_animation_ids_index (int): Index of the current human animation id in the list of human animation ids.
        animation_start_time (int): Start time of the current human animation.
        animation_time (int): Current time of the current human animation.
        low_level_time (int): Current time of the low level controller.
        human_pos_offset (List[float]): Offset of the human position.
        human_rot_offset (List[float]): Offset of the human rotation.
        object_placements_list (List[List[Tuple[str, np.ndarray]]]): List of object placements.
            Stores joint name and joint position for each object for each placement.
            During the episode this list is iterated over and the corresponding object joint position is used.
        object_placements_list_index (int): Index of the current object joint position in the list of object joint
            positions.
        task_phase_value (int): Value corresponding to the current task phase.
        n_delayed_timesteps (Tuple[int, int]): Number of timesteps the current animation is delayed
            because of the loop phases. Contains two values: delays for the present and wait phase.
        animation_loop_properties (List[Dict[str, Tuple[float, float]]]): Loop amplitudes and speed modifiers
            for all layered sines for all human animations sampled for the current episode.
    """
    object_placements_list: List[List[Tuple[str, np.ndarray]]]
    object_placements_list_index: int
    task_phase_value: int
    n_delayed_timesteps: Tuple[int, int]
    animation_loop_properties: List[Dict[str, Tuple[List[float], List[float]]]]
    object_stack_body_ids: List[int]


class CollaborativeStackingCart(HumanEnv):
    """This class corresponds to a collaborative stacking task for a single robot arm in a human environment.

    The objective of this task is to build a stack of cubes in collaboration with a human partner.
    The human partner will place the first cube on the table, from there on robot and human will place alternately.
    The task is divided into 6 phases:
        1. Approach: The human approaches the table.
        2. Place first: The human places the first cube on the table.
        3. Wait for second: The human waits for the robot to place the second cube.
        4. Place third: The human places the third cube on the stack.
        5. Wait for fourth: The human waits for the robot to place the fourth cube.
        6. Retreat: The human retreats from the table.

    When using a fixed horizon (`done_at_success = False`), these phases are repeated until the horizon is reached.
    Otherwise, the episode ends when the animation is finished.

    Reward is given once the task is done, i.e. the animation of the human is finished.
    Optionally, the sparse reward may be augmented by sub-objective rewards for:
        - An object being gripped (`object_gripped_reward`)
        - The second cube being at its target (`second_cube_at_target_reward`)
        - The fourth cube being at its target (`fourth_cube_at_target_reward`)

    Reward penalties may be given for
        - An illegal collision occurring (`collision_reward`)
        - The stack being toppled over (`stack_toppled_reward`)

    Tasks are terminated if the stack is toppled over
    or if an illegal collision occurs (only if `done_at_collision = True`).

    Args:
        robots (str | List[str]): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: `"Sawyer"` would generate one arm; `["Panda", "Panda", "Sawyer"]` would generate three robot arms)
            Note: Must be a single single-arm robot!

        robot_base_offset (None | List[float] | List[List[float]]): Offset (x, y, z) of the robot bases.
            If more than one robot is loaded provide a list of doubles, one for each robot.
            Specify `None` for an offset of (0, 0, 0) for each robot.

        env_configuration (str): Specifies how to position the robots within the environment (default is `"default"`).
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (None | str | List[Dict[str, Any]]): If set, contains relevant controller parameters
            for creating a custom controller. Else, uses the default controller for this specific task.
            Should either be single dict if same controller is to be used for all robots or else it should be
            a list of the same length as `robots` param

        gripper_types (str | List[str]): type of gripper, used to instantiate
            gripper models from gripper factory. Default is `"default"`, which is the default grippers(s) associated
            with the robot(s) the `robots` specification. `None` removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single `str` if same gripper type is to be used for all
            robots or else it should be a list of the same length as the `robots` param

        initialization_noise (Dict[str, Any] | List[Dict[str, Any]]): Dict containing the initialization noise
            parameters. The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or `0.0` results in no noise being applied.
                If `"gaussian"` type of noise is applied then this magnitude scales the standard deviation applied,
                If `"uniform"` type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify `"gaussian"` or `"uniform"`

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as `robots` param

            :Note: Specifying `"default"` will automatically use the default noise settings.
                Specifying `None` will automatically create the required dict with `"magnitude"` set to `0.0`.

        table_full_size (Tuple[float, float, float]): x, y, and z dimensions of the table.

        table_friction (Tuple[float, float, float]): the three mujoco friction parameters for
            the table.

        object_full_size (Tuple[float, float, float]): x, y, and z dimensions of the cube objects that should be moved.

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (None | float): Scales the normalized reward function by the amount specified.
            If `None`, environment reward remains unnormalized

        reward_shaping (bool): if `True`, use dense rewards, else use sparse rewards.

        goal_dist (float): Distance threshold for reaching the goal.

        n_object_placements_sampled_per_100_steps (int): How many object placements to sample at resets
            per 100 steps in the horizon.
            After all objects of the list have been placed, restart from the first in the list.
            This is done to ensure the same list of objects can be placed when loading the env state from a file.

        collision_reward (float): Reward to be given in the case of a collision.

        stack_toppled_reward (float): Reward to be given in the case the stack is toppled over.

        task_reward (float): Reward to be given in the case of completing the task (i.e. finishing the animation).

        second_cube_at_target_reward (float): Reward if one cube of the robot is within `goal_dist` of its target.

        fourth_cube_at_target_reward (float): Reward if both cubes of the robotare within `goal_dist` of their targets.

        object_gripped_reward (float): Additional reward for gripping a cube when `reward_shaping=False`.

        object_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a `UniformRandomSampler`
            is used by default.
            Objects are elements that can and should be manipulated.

        obstacle_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place obstacles on every reset, else a `UniformRandomSampler`
            is used by default.
            Obstacles are elements that should be avoided.

        has_renderer (bool): If `True`, render the simulation state in
            a viewer instead of headless mode.

        has_offscreen_renderer (bool): `True` if using off-screen rendering

        render_camera (str): Name of camera to render if `has_renderer` is `True`. Setting this value to `None`
            will resul` in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse

        render_collision_mesh (bool): `True` if rendering collision meshes in camera. `False` otherwise.

        render_visual_mesh (bool): `True` if rendering visual meshes in camera. `False` otherwise.

        render_gpu_device_id (int): corresponds to the GPU device id to use for offscreen rendering.
            Defaults to `-1`, in which case the device will be inferred from environment variables
            (`GPUS` or `CUDA_VISIBLE_DEVICES`).

        control_freq (float): how many control signals to receive in every second. This sets the amount of
            simulation time that passes between every action input.

        horizon (int): Every episode lasts for exactly `horizon` action steps.

        ignore_done (bool): `True` if never terminating the environment (ignore `horizon`).

        hard_reset (bool): If `True`, re-loads model, sim, and render object upon a `reset` call, else,
            only calls `self.sim.reset` and resets all robosuite-internal variables

        camera_names (str | List[str]): name of camera to be rendered. Should either be single `str` if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if `use_camera_obs` is `True`.

            :Note: To render all robots' cameras of a certain type (e.g.: `"robotview"` or `"eye_in_hand"`), use the
                convention `"all-{name}"` (e.g.: `"all-robotview"`) to automatically render all camera images from each
                robot's camera list).

        camera_heights (int | List[int]): height of camera frame. Should either be single `int` if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            `camera_names` param.

        camera_widths (int | List[int]): width of camera frame. Should either be single `int` if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            `camera_names` param.

        camera_depths (bool | List[bool]): `True` if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            `camera_names` param.

        camera_segmentations (None | str | List[str] | List[List[str]]): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not `None`, multiple types of segmentations can be specified. A [List[str] / str | None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A List[List[str]] specifies per-camera
            segmentation setting(s) to use.

        renderer (str): string for the renderer to use

        renderer_config (dict): dictionary for the renderer configurations

        shield_type (str): Shield type to use. Valid options are: "OFF", "SSM", and "PFL"

        visualize_failsafe_controller (bool): Whether or not the reachable sets of the failsafe controller should be
            visualized

        visualize_pinocchio (bool): Whether or not pinocchio (collision prevention static env) should be visualized

        control_sample_time (float): Control frequency of the failsafe controller

        human_animation_names (List[str]): Human animations to play

        base_human_pos_offset (List[float]): Base human animation offset

        human_animation_freq (float): Speed of the human animation in fps.

        human_rand (List[float]): Max. randomization of the human [x-pos, y-pos, z-angle]

        n_animations_sampled_per_100_steps (int): How many animations to sample at resets per 100 steps in the horizon.
            After all animations of the list have been played, restart from the first animation in the list.
            This is done to ensure the same list of animations can be played when loading the env state from a file.

        safe_vel (float): Safe cartesian velocity. The robot is allowed to move with this velocity in the vicinity of
            humans.

        self_collision_safety (float): Safe distance for self collision detection

        seed (int): Random seed for `np.random`

        verbose (bool): If `True`, print out debug information

        done_at_collision (bool): If `True`, the episode is terminated when a collision occurs

        done_at_success (bool): If `True`, the episode is terminated when the goal is reached

    Raises:
        AssertionError: [Invalid number of robots specified]
    """
    def __init__(
        self,
        robots: Union[str, List[str]],
        robot_base_offset: Optional[Union[List[float], List[List[float]]]] = None,
        env_configuration: str = "default",
        controller_configs: Optional[Union[str, List[Dict[str, Any]]]] = None,
        gripper_types: Union[str, List[str]] = "default",
        initialization_noise: Union[str, List[str], List[Dict[str, Any]]] = "default",
        table_full_size: Tuple[float, float, float] = (1.2, 2.0, 0.05),
        table_friction: Tuple[float, float, float] = (1.0, 5e-3, 1e-4),
        object_full_size: Tuple[float, float, float] = (0.045, 0.045, 0.045),
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        reward_shaping: bool = False,
        goal_dist: float = 0.025,
        n_object_placements_sampled_per_100_steps: int = 2,
        collision_reward: float = -10,
        stack_toppled_reward: float = -10,
        task_reward: float = 1,
        second_cube_at_target_reward: float = -1,
        fourth_cube_at_target_reward: float = -1,
        object_gripped_reward: float = -1,
        object_placement_initializer: Optional[ObjectPositionSampler] = None,
        obstacle_placement_initializer: Optional[ObjectPositionSampler] = None,
        has_renderer: bool = False,
        has_offscreen_renderer: bool = True,
        render_camera: str = "frontview",
        render_collision_mesh: bool = False,
        render_visual_mesh: bool = True,
        render_gpu_device_id: int = -1,
        control_freq: float = 10,
        horizon: int = 1000,
        ignore_done: bool = False,
        hard_reset: bool = True,
        camera_names: Union[str, List[str]] = "frontview",
        camera_heights: Union[int, List[int]] = 256,
        camera_widths: Union[int, List[int]] = 256,
        camera_depths: Union[bool, List[bool]] = False,
        camera_segmentations: Optional[Union[str, List[str], List[List[str]]]] = None,
        renderer: str = "mujoco",
        renderer_config: Dict[str, Any] = None,
        shield_type: str = "SSM",
        visualize_failsafe_controller: bool = False,
        visualize_pinocchio: bool = False,
        control_sample_time: float = 0.004,
        human_animation_names: List[str] = [
            "CollaborativeStacking/0",
            "CollaborativeStacking/1",
            "CollaborativeStacking/2",
            "CollaborativeStacking/3",
            "CollaborativeStacking/4",
            "CollaborativeStacking/5",
            "CollaborativeStacking/6",
            "CollaborativeStacking/7",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 30,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        n_animations_sampled_per_100_steps: int = 2,
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
    ):
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.object_full_size = object_full_size
        # settings for table top (hardcoded since it's not an essential part of the environment)
        self.table_offset = np.array((0.0, 0.0, 0.82))
        # reward configuration
        self.stack_toppled_reward = stack_toppled_reward
        self.second_cube_at_target_reward = second_cube_at_target_reward
        self.fourth_cube_at_target_reward = fourth_cube_at_target_reward
        self.object_gripped_reward = object_gripped_reward
        self.goal_dist = goal_dist
        self._object_placements_list = None
        self._object_placements_list_index = 0
        self._n_objects_to_sample_at_resets = int(
            horizon * n_object_placements_sampled_per_100_steps / 100
        )

        self._object_stack_body_ids = None
        self._max_stack_height = 0

        # object placement initializer
        self.object_placement_initializer = object_placement_initializer
        self.obstacle_placement_initializer = obstacle_placement_initializer

        # if run should stop at collision
        self.done_at_collision = done_at_collision
        self.done_at_success = done_at_success

        self.task_phase: CollaborativeStackingPhase = CollaborativeStackingPhase.COMPLETE
        self._n_delayed_timesteps = None

        self._animation_loop_properties = None

        self._l_mocap_body_name = "lh_mocap_object"
        self._r_mocap_body_name = "rh_mocap_object"

        self._l_weld_eq_name = "lh_weld_eq"
        self._r_weld_eq_name = "rh_weld_eq"

        self._l_weld_eq_id = None
        self._r_weld_eq_id = None

        self.l_cube = None
        self.r_cube = None

        self._l_cube_body_id = None
        self._r_cube_body_id = None

        self.manipulation_objects = None
        self._manipulation_objects_body_ids = None

        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            reward_scale=reward_scale,
            reward_shaping=reward_shaping,
            collision_reward=collision_reward,
            task_reward=task_reward,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
            shield_type=shield_type,
            visualize_failsafe_controller=visualize_failsafe_controller,
            visualize_pinocchio=visualize_pinocchio,
            control_sample_time=control_sample_time,
            human_animation_names=human_animation_names,
            base_human_pos_offset=base_human_pos_offset,
            human_animation_freq=human_animation_freq,
            human_rand=human_rand,
            n_animations_sampled_per_100_steps=n_animations_sampled_per_100_steps,
            safe_vel=safe_vel,
            self_collision_safety=self_collision_safety,
            seed=seed,
            verbose=verbose,
        )

    @property
    def object_placements(self) -> List[Tuple[str, np.ndarray]]:
        """List of object placements.

        Returns:
            List[Tuple[str, np.ndarray]]: List of object placements.
                Stores joint name and joint position for each object.
        """
        return self._object_placements_list[self._object_placements_list_index]

    @property
    def wait_for_second_cube_loop_amplitudes(self) -> Dict[str, List[float]]:
        """Loop amplitudes for all layered sines of the first wait phase for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index]["wait_for_second"][0]

    @property
    def wait_for_second_cube_loop_speeds(self) -> Dict[str, List[float]]:
        """Loop speed modifiers for all layered sines of the first wait phase for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index]["wait_for_second"][1]

    @property
    def wait_for_fourth_cube_loop_amplitudes(self) -> Dict[str, List[float]]:
        """Loop amplitudes for all layered sines of the second wait phase for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index]["wait_for_fourth"][0]

    @property
    def wait_for_fourth_cube_loop_speeds(self) -> Dict[str, List[float]]:
        """Loop speed modifiers for all layered sines of the second wait phase for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index]["wait_for_fourth"][1]

    @property
    def keyframes(self) -> List[int]:
        """List of keyframes in the current human animation."""
        return self.human_animation_data[self.human_animation_id][1]["keyframes"]

    @property
    def first_placing_hand(self) -> str:
        """Hand with which the first cube is placed. Either `"left"` or `"right"`."""
        return self.human_animation_data[self.human_animation_id][1]["first_placing_hand"]

    @property
    def next_target_position(self) -> Optional[np.ndarray]:
        """Position of the target for the next cube the robot should place.

        Returns:
            Optional[np.ndarray]: Position of the target for the next cube the robot should place.
                `None` if the robot should not place a cube next
                (i.e. outside the WAIT_FOR_SECOND and WAIT_FOR_FOURTH phases)
        """
        pos = None

        # Obtain location of the cube last placed by the human
        if (self.task_phase, self.first_placing_hand) in [
            (CollaborativeStackingPhase.WAIT_FOR_SECOND, "left"),
            (CollaborativeStackingPhase.WAIT_FOR_FOURTH, "right"),
        ]:
            pos = self.sim.data.body_xpos[self._l_cube_body_id]
        elif (self.task_phase, self.first_placing_hand) in [
            (CollaborativeStackingPhase.WAIT_FOR_SECOND, "right"),
            (CollaborativeStackingPhase.WAIT_FOR_FOURTH, "left"),
        ]:
            pos = self.sim.data.body_xpos[self._r_cube_body_id]

        # Get location one cube size above the last placed cube
        if pos is not None:
            return pos + np.array([0, 0, self.object_full_size[2]])

        return pos

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Override super `step` method to manage the animation phase state machine."""
        obs, rew, done, info = super().step(action)

        object_gripped = obs["object_gripped"]

        if self.goal_reached:
            self._on_goal_reached()
            self.goal_reached = False
        if self.has_renderer:
            self._visualize()

        if self.task_phase == CollaborativeStackingPhase.PLACE_FIRST and self.animation_time > self.keyframes[1]:
            self._human_place_first_object()
            self.task_phase = CollaborativeStackingPhase.WAIT_FOR_SECOND
        elif (
            self.task_phase == CollaborativeStackingPhase.WAIT_FOR_SECOND and
            (body_id := self._id_of_cube_at_target()) is not None and
            not object_gripped
        ):
            self.task_phase = CollaborativeStackingPhase.PLACE_THIRD
            self._object_stack_body_ids.append((body_id))
        elif (
            self.task_phase == CollaborativeStackingPhase.PLACE_THIRD and
            self.animation_time > self.keyframes[3]
        ):
            self._human_place_third_object()
            self.task_phase = CollaborativeStackingPhase.WAIT_FOR_FOURTH
        elif (
            self.task_phase == CollaborativeStackingPhase.WAIT_FOR_FOURTH and
            self._id_of_cube_at_target() is not None and
            not object_gripped
        ):
            self.task_phase = CollaborativeStackingPhase.RETREAT

        self._max_stack_height = max(self._max_stack_height, len(self._object_stack_body_ids))

        return obs, rew, done, info

    def _id_of_cube_at_target(
        self,
    ) -> Optional[int]:
        target = self.next_target_position

        if target is None:
            return None

        for body_id in self._manipulation_objects_body_ids:
            if self._object_to_target_dist(target, self.sim.data.body_xpos[body_id]) < self.goal_dist:
                return body_id

        return None

    def _object_to_target_dist(
        self,
        target: np.ndarray,
        object_pos: np.ndarray,
    ) -> float:
        """Distance between object and target. We use a cube distance metric."""
        return np.max(np.abs(target - object_pos))

    def _check_first_manipulation_object_in_target_zone(
        self,
        achieved_goal: List[float],
        desired_goal: List[float]
    ) -> Optional[int]:
        object_a_pos = achieved_goal[1:4]
        object_b_pos = achieved_goal[4:7]

        if len(self._object_stack_body_ids) < 1:
            return False

        bottom_object_pos = self.sim.data.body_xpos[self._object_stack_body_ids[0]]
        second_object_target_pos = bottom_object_pos + np.array([0, 0, self.object_full_size[2]])

        if len(self._object_stack_body_ids) < 2:
            # Both manipulation objects may be placed first
            if np.max(np.abs(second_object_target_pos - object_a_pos)) < self.goal_dist:
                return self._manipulation_objects_body_ids[0]
            elif np.max(np.abs(second_object_target_pos - object_b_pos)) < self.goal_dist:
                return self._manipulation_objects_body_ids[1]
            else:
                return None

        if np.max(np.abs(
            second_object_target_pos - self.sim.data.body_xpos[self._object_stack_body_ids[1]]
        )) < self.goal_dist:
            return self._object_stack_body_ids[1]
        else:
            return None

    def _check_second_manipulation_object_in_target_zone(
        self,
        achieved_goal: List[float],
        desired_goal: List[float]
    ) -> Optional[int]:
        if len(self._object_stack_body_ids) < 3:
            return False

        fourth_object_target_pos = self.sim.data.body_xpos[
            self._object_stack_body_ids[2]
        ] + np.array([0, 0, self.object_full_size[2]])

        manipulation_object_body_id = (
            self._manipulation_objects_body_ids[0]
            if self._object_stack_body_ids[2] == self._manipulation_objects_body_ids[1]
            else self._manipulation_objects_body_ids[1]
        )

        manipulation_object_pos = self.sim.data.body_xpos[manipulation_object_body_id]

        if np.max(np.abs(fourth_object_target_pos - manipulation_object_pos)) < self.goal_dist:
            return manipulation_object_body_id
        else:
            return None

    def _check_stack_toppled(self) -> bool:
        if len(self._object_stack_body_ids) < 2:
            return False
        else:
            bottom_object_id = self._object_stack_body_ids[0]
            min_height = self.sim.data.body_xpos[bottom_object_id][2] + self.object_full_size[2] / 2
            return any([
                self.sim.data.body_xpos[body_id][2] < min_height
                for body_id in self._object_stack_body_ids[1:]
            ])

    def _get_info(self) -> Dict[str, Any]:
        info = super()._get_info()

        info["max_stack_height"] = self._max_stack_height

        return info

    def reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        if self.reward_shaping:
            reward = self._shaped_reward(
                achieved_goal=achieved_goal,
                desired_goal=desired_goal,
                info=info,
            )
        else:
            reward = self._sparse_reward(
                achieved_goal=achieved_goal,
                desired_goal=desired_goal,
                info=info,
            )

        if COLLISION_TYPE(info["collision_type"]) not in (COLLISION_TYPE.NULL | COLLISION_TYPE.ALLOWED):
            reward += self.collision_reward

        if self._check_stack_toppled():
            reward += self.stack_toppled_reward

        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

    def _shaped_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        # TODO
        return 0

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        if self._check_success(achieved_goal, desired_goal):
            return self.task_reward

        reward = -1

        if self._check_second_manipulation_object_in_target_zone(achieved_goal, desired_goal):
            reward = self.fourth_cube_at_target_reward
        elif self._check_first_manipulation_object_in_target_zone(achieved_goal, desired_goal):
            reward = self.second_cube_at_target_reward

        object_gripped = achieved_goal[0]

        if object_gripped:
            reward += self.object_gripped_reward

        return reward

    def _check_success(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
    ) -> bool:
        """Override the super class success condition to check if the animation is complete.

        If the robot never graspes the object, the animation remains in the `PRESENT` phase.
        If the robot does not manage to place the object to the target, the animation does not exceed the `WAIT` phase.
        """
        return self.task_phase == CollaborativeStackingPhase.COMPLETE

    def _check_done(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> bool:
        """Compute the done flag based on the achieved goal, the desired goal, and the info dict.

        This function can only be called for one sample.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collision
        Returns:
            bool: done flag
        """
        if self.done_at_collision and COLLISION_TYPE(info["collision_type"]) not in (
            COLLISION_TYPE.NULL | COLLISION_TYPE.ALLOWED
        ):
            return True

        if self._check_stack_toppled():
            return True

        if self.done_at_success and self._check_success(achieved_goal, desired_goal):
            return True

        return False

    def _get_achieved_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        return np.concatenate(
            [
                [observation["object_gripped"]],
                observation["all_objects_pos"],
                observation[f"{self.robots[0].robot_model.naming_prefix}eef_pos"],
            ]
        ).tolist()

    def _get_desired_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        return np.concatenate(
            [
                observation["next_target_pos"]
            ]
        ).tolist()

    def _compute_animation_time(self, control_time: int) -> int:
        animation_time = super()._compute_animation_time(control_time=control_time)
        classic_animation_time = animation_time

        animation_length = self.human_animation_data[self.human_animation_id][0]["Pelvis_pos_x"].shape[0]

        if animation_time > self.keyframes[0] and self.task_phase == CollaborativeStackingPhase.APPROACH:
            self.task_phase = CollaborativeStackingPhase.PLACE_FIRST

        elif self.task_phase == CollaborativeStackingPhase.WAIT_FOR_SECOND:
            modulation_start_time = self.keyframes[2]

            if animation_time >= modulation_start_time:
                animation_time = int(
                    layered_sin_modulations(
                        classic_animation_time=classic_animation_time,
                        modulation_start_time=modulation_start_time,
                        amplitudes=self.wait_for_second_cube_loop_amplitudes,
                        speeds=self.wait_for_second_cube_loop_speeds,
                    )
                )

            self._n_delayed_timesteps = (classic_animation_time - animation_time, 0)

        elif self.task_phase == CollaborativeStackingPhase.PLACE_THIRD:
            animation_time = classic_animation_time - self._n_delayed_timesteps[0]

        elif self.task_phase == CollaborativeStackingPhase.WAIT_FOR_FOURTH:
            modulation_start_time = self.keyframes[4]
            animation_time = classic_animation_time - self._n_delayed_timesteps[0]

            if animation_time >= modulation_start_time:
                animation_time = int(
                    layered_sin_modulations(
                        classic_animation_time=animation_time,
                        modulation_start_time=modulation_start_time,
                        amplitudes=self.wait_for_fourth_cube_loop_amplitudes,
                        speeds=self.wait_for_fourth_cube_loop_speeds,
                    )
                )

            self._n_delayed_timesteps = (self._n_delayed_timesteps[0], classic_animation_time - animation_time)

        elif self.task_phase == CollaborativeStackingPhase.RETREAT:
            animation_time = classic_animation_time - self._n_delayed_timesteps[1]

        if animation_time >= animation_length - 1:
            self.task_phase = CollaborativeStackingPhase.COMPLETE
            animation_time = animation_length - 1

        return animation_time

    def _control_human(self, force_update: bool = True):
        """Augment super class method to keep the mocap object's position at the human's hand."""
        super()._control_human(force_update=True)
        self.sim.step()
        self._update_mocap_body_transforms()

    def _update_mocap_body_transforms(self):
        """Update the mocap body's position and orientation to be at the human's extended hand."""
        l_hand_rot = quat_to_rot(self.sim.data.get_body_xquat(self.human.left_hand))
        r_hand_rot = quat_to_rot(self.sim.data.get_body_xquat(self.human.right_hand))

        l_pos_offset_towards_thumb, r_pos_offset_towards_thumb = [
            hand_rot.apply(np.array([0, 0, 1])) * 0.03
            for hand_rot in [l_hand_rot, r_hand_rot]
        ]

        l_hand_rot *= Rotation.from_euler("y", -np.pi / 2)
        r_hand_rot *= Rotation.from_euler("y", np.pi / 2)

        self.sim.data.set_mocap_pos(
            self._l_mocap_body_name,
            self.sim.data.get_site_xpos(self.human.left_hand) + l_pos_offset_towards_thumb
        )

        self.sim.data.set_mocap_pos(
            self._r_mocap_body_name,
            self.sim.data.get_site_xpos(self.human.right_hand) + r_pos_offset_towards_thumb
        )

        self.sim.data.set_mocap_quat(
            self._l_mocap_body_name,
            rot_to_quat(l_hand_rot),
        )

        self.sim.data.set_mocap_quat(
            self._r_mocap_body_name,
            rot_to_quat(r_hand_rot),
        )

    def _reset_internal(self):
        """Reset the environment's internal state."""
        self.robots[0].init_qpos = np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, np.pi / 4])

        super()._reset_internal()
        self._reset_animation()
        self._control_human()

        self._animation_loop_properties = [
            sample_animation_loop_properties(
                animation_info=self.human_animation_data[human_animation_id][1],
            )
            for human_animation_id in self._human_animation_ids
        ]

        self._object_placements_list = [
            [
                (obj.joints[0], np.concatenate([obj_pos, obj_quat]))
                for obj_pos, obj_quat, obj in self.object_placement_initializer.sample().values()
            ] for _ in range(self._n_objects_to_sample_at_resets)
        ]
        self._object_placements_list_index = 0

    def _on_goal_reached(self):
        """Callback function that is called when the goal is reached.

        Samples a new position for the object and a new target position.
        """
        if self.done_at_success:
            return

        self._object_placements_list_index = (
            (self._object_placements_list_index + 1) % self._n_objects_to_sample_at_resets
        )
        for joint_name, joint_qpos in self.object_placements:
            self.sim.data.set_joint_qpos(joint_name, joint_qpos)

        self._progress_to_next_animation(
            animation_start_time=int(self.low_level_time / self.human_animation_step_length)
        )

    def _progress_to_next_animation(self, animation_start_time: float):
        """Pick a new animation during an episode.

        Called when the current animation is complete and `self.done_at_success` is set to `False`.

        Args:
            animation_start_time (float): The time at which the new animation should start.
        """
        super()._progress_to_next_animation(animation_start_time=animation_start_time)
        self._reset_animation()
        self._control_human()

    def _reset_animation(self):
        """Reset animation-dependent internal variables"""
        self.task_phase = CollaborativeStackingPhase.APPROACH
        self._n_delayed_timesteps = (0, 0)
        self._object_stack_body_ids = []
        self._max_stack_height = 0

        self._human_pickup_objects()

    def _set_equality_status(self, equality_id: int, status: bool):
        self.sim.model.eq_active[equality_id] = int(status)

    def _human_place_first_object(self):
        if self.first_placing_hand == "left":
            self._object_stack_body_ids.append(self._l_cube_body_id)
            self._human_drop_left_object()
        else:
            self._object_stack_body_ids.append(self._r_cube_body_id)
            self._human_drop_right_object()

    def _human_place_third_object(self):
        cube = None

        if self.first_placing_hand == "left":
            cube = self.r_cube
            self._human_drop_right_object()
        else:
            cube = self.l_cube
            self._human_drop_left_object()

        self.sim.data.set_joint_qpos(
            cube.joints[0],
            np.concatenate(
                [
                    self.sim.data.body_xpos[
                        self._object_stack_body_ids[1]
                    ] + np.array([0, 0, self.object_full_size[2]]),
                    [1, 0, 0, 0],
                ]
            )
        )

        self.sim.data.set_joint_qvel(
            cube.joints[0],
            np.array([0, 0, 0, 0, 0, 0])
        )

        self._object_stack_body_ids.append(self.sim.model.body_name2id(cube.root_body))

    def _human_drop_left_object(self):
        """Drop the left object."""
        self._set_equality_status(self._l_weld_eq_id, False)

    def _human_drop_right_object(self):
        """Drop the right object."""
        self._set_equality_status(self._r_weld_eq_id, False)

    def _human_pickup_objects(self):
        """Pick up both objects."""
        self._set_equality_status(self._l_weld_eq_id, True)
        self._set_equality_status(self._r_weld_eq_id, True)

    def _get_default_object_bin_boundaries(self) -> Tuple[float, float, float, float]:
        """Get the x and y boundaries of the target sampling space.

        Returns:
            Tuple[float, float, float, float]:
                Boundaries of sampling space in the form (xmin, xmax, ymin, ymax)
        """
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05

        return (
            bin_x_half * 0.5,
            bin_x_half * 0.8,
            -bin_y_half * 0.15,
            bin_y_half * 0.15,
        )

    def _visualize_next_target_location(self):
        if (pos := self.next_target_position) is not None:
            self.viewer.viewer.add_marker(
                pos=pos,
                type=6,
                size=[self.goal_dist, self.goal_dist, self.goal_dist],
                rgba=[0, 1, 0, 0.3],
                label="",
                shininess=0.0,
            )

    def _visualize_object_sample_space(self):
        """Draw a box to display the object sampling space"""
        self.draw_box(
            self._get_default_object_bin_boundaries() + (  # Add z boundaries
                self.table_offset[2] - 0.05,
                self.table_offset[2] + 0.05,
            ),
            (1.0, 0.0, 0.0, 0.3),
        )

    def _visualize(self):
        self._visualize_object_sample_space()
        self._visualize_next_target_location()

    def draw_box(
        self,
        boundaries: Tuple[float, float, float, float, float, float],
        color: Tuple[float, float, float, float],
    ):
        """Render a box in the scene.

        Args:
            boundaries (Tuple[float, float, float, float, float, float]):
                Box boundaries in the form (xmin, xmax, ymin, ymax, zmin, zmax)
            color (Tuple[float, float, float, float]):
                Color in the form (r, g, b, a)
        """
        # Box (type 2)
        self.viewer.viewer.add_marker(
            pos=np.array([
                (boundaries[0] + boundaries[1]) / 2,
                (boundaries[2] + boundaries[3]) / 2,
                (boundaries[5] + boundaries[4]) / 2,
            ]),
            type=6,
            size=[
                (boundaries[1] - boundaries[0]) * 0.5,
                (boundaries[3] - boundaries[2]) * 0.5,
                (boundaries[5] - boundaries[4]) * 0.5,
            ],
            rgba=color,
            label="",
            shininess=0.0,
        )

    def _setup_arena(self):
        """Setup the mujoco arena.

        Must define `self.mujoco_arena`.
        Defines `self.objects` and `self.obstacles`.
        """
        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_offset=self.table_offset,
            xml=xml_path_completion("arenas/table_arena.xml"),
        )

        self._set_origin()

        self._set_mujoco_camera()

        box_size = np.array(self.object_full_size)
        self.manipulation_objects = [
            BoxObject(
                name="manipulation_object_a",
                size=box_size * 0.5,
                rgba=[0.1, 0.7, 0.3, 1],
            ),
            BoxObject(
                name="manipulation_object_b",
                size=box_size * 0.5,
                rgba=[0.1, 0.7, 0.3, 1],
            ),
        ]

        self.l_cube = BoxObject(
            name="human_l_cube",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )

        self.r_cube = BoxObject(
            name="human_r_cube",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )

        self.objects = [
            *self.manipulation_objects,
            self.l_cube,
            self.r_cube,
        ]
        object_bin_boundaries = self._get_default_object_bin_boundaries()

        # object_bin_boundaries = self._get_default_object_bin_boundaries()
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=[*self.manipulation_objects],
            x_range=[object_bin_boundaries[0], object_bin_boundaries[1]],
            y_range=[object_bin_boundaries[2], object_bin_boundaries[3]],
        )

        # << OBSTACLES >>
        self._setup_collision_objects(
            add_table=True,
            add_base=True,
            safety_margin=0.0
        )
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def _postprocess_model(self):
        """Extend super class method to add additional elements to the model before creating the sim object."""
        super()._postprocess_model()

        # Object at the human hand (position and rotation), handover object may be welded to it
        self._add_mocap_bodies_to_model()

        # Connection between a motion capture object at the human's hand and the root body of the object.
        # Can be activated to simulate the human grasping the object.
        self._add_weld_equalities_to_model()

    def _add_mocap_bodies_to_model(self, visualize: bool = False) -> Tuple[ET.Element, ET.Element]:
        """Add a mocap body to the model.

        This body is a direct child of the world body and has no joints.
        Its position and rotation can be controlled by calling `self.sim.data.set_mocap_pos` and
        `self.sim.data.set_mocap_quat`.

        Args:
            visualize (bool): If `True`, visualize the mocap body with a cube geom. Defaults to `False`.

        Returns:
            ET.Element: The created mocap body xml tree element.
        """
        mocap_objects = (
            ET.Element(
                "body", name=name, pos="0 0 0", quat="0 0 0 1", mocap="true"
            )
            for name in [self._l_mocap_body_name, self._r_mocap_body_name]
        )

        if visualize:
            for mocap_object in mocap_objects:
                mocap_object.append(
                    ET.Element(
                        "geom",
                        name=f"{mocap_object.get('name')}_vis",
                        size="0.1 0.1 0.1",
                        rgba="0.8 0.2 0.2 0.7",
                        type="box",
                        contype="0",
                        conaffinity="0",
                        group="1",
                    )
                )

        self.model.worldbody.extend(mocap_objects)

        return mocap_objects

    def _add_weld_equalities_to_model(self) -> Tuple[ET.Element, ET.Element]:
        """Add a weld equality to the model between the mocap object and the manipulation object.

        This equality can be deactivated to simulate the human letting go of the object.

        Returns:
            ET.Element: The created weld equality xml tree element.
        """
        equalities = (
            ET.Element(
                "weld",
                name=self._l_weld_eq_name,
                body1=self.l_cube.root_body,
                body2=self._l_mocap_body_name,
                relpose="0 0.045 0.0 1 0 0 0",
                # solref="-700 -100",
            ),
            ET.Element(
                "weld",
                name=self._r_weld_eq_name,
                body1=self.r_cube.root_body,
                body2=self._r_mocap_body_name,
                relpose="0 0.045 0.0 1 0 0 0",
                # solref="-700 -100",
            ),
        )

        self.model.equality.extend(equalities)

        return equalities

    def _setup_references(self):
        """Set up references to important components."""
        super()._setup_references()

        self._manipulation_objects_body_ids = [
            self.sim.model.body_name2id(manipulation_object.root_body)
            for manipulation_object in self.manipulation_objects
        ]

        self._l_cube_body_id = self.sim.model.body_name2id(self.l_cube.root_body)
        self._r_cube_body_id = self.sim.model.body_name2id(self.r_cube.root_body)

        self._l_weld_eq_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, self._l_weld_eq_name,
        )

        self._r_weld_eq_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, self._r_weld_eq_name,
        )

    def _setup_observables(self) -> OrderedDict[str, Observable]:
        """Set up observables to be used for this environment.

        Creates object-based observables if enabled.

        Returns:
            OrderedDict[str, Observable]: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()
        # robot joint pos
        prefix = self.robots[0].robot_model.naming_prefix
        if prefix + "joint_pos" in observables:
            observables[prefix + "joint_pos"].set_active(False)
        if prefix + "joint_vel" in observables:
            observables[prefix + "joint_vel"].set_active(False)
        if prefix + "eef_pos" in observables:
            observables[prefix + "eef_pos"].set_active(True)
        if "human_joint_pos" in observables:
            observables["human_joint_pos"].set_active(True)
        if prefix + "joint_pos_cos" in observables:
            observables[prefix + "joint_pos_cos"].set_active(False)
        if prefix + "joint_pos_sin" in observables:
            observables[prefix + "joint_pos_sin"].set_active(False)
        if prefix + "gripper_qpos" in observables:
            observables[prefix + "gripper_qpos"].set_active(True)
        if prefix + "gripper_qvel" in observables:
            observables[prefix + "gripper_qvel"].set_active(True)
        if prefix + "eef_quat" in observables:
            observables[prefix + "eef_quat"].set_active(False)
        if "gripper_pos" in observables:
            observables["gripper_pos"].set_active(False)
        if "gripper_aperture" in observables:
            observables["gripper_aperture"].set_active(True)

        # low-level object information
        goal_mod = "goal"
        obj_mod = "object"
        pf = self.robots[0].robot_model.naming_prefix

        # Absolute coordinates of the next target position
        # If there is currently no next target position, the current eef position is returned
        @sensor(modality=goal_mod)
        def next_target_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            if (next_target_pos := self.next_target_position) is not None:
                return next_target_pos
            elif f"{pf}eef_pos" in obs_cache:
                return obs_cache[f"{pf}eef_pos"]
            else:
                return np.zeros(3)

        # Absolute coordinates of object position
        @sensor(modality=obj_mod)
        def object_a_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xpos[self._manipulation_objects_body_ids[0]]

        @sensor(modality=obj_mod)
        def object_b_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xpos[self._manipulation_objects_body_ids[1]]

        @sensor(modality=obj_mod)
        def object_l_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xpos[self._l_cube_body_id]

        @sensor(modality=obj_mod)
        def object_r_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xpos[self._r_cube_body_id]

        @sensor(modality=obj_mod)
        def all_objects_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            if set(["object_a_pos", "object_b_pos", "object_l_pos", "object_r_pos"]).issubset(obs_cache.keys()):
                return np.concatenate(
                    [
                        obs_cache["object_a_pos"],
                        obs_cache["object_b_pos"],
                        obs_cache["object_l_pos"],
                        obs_cache["object_r_pos"],
                    ]
                )
            else:
                return np.zeros(12)

        @sensor(modality=obj_mod)
        def vec_eef_to_object_a(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["object_a_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "object_a_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=obj_mod)
        def vec_eef_to_object_b(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["object_b_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "object_b_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=obj_mod)
        def vec_eef_to_object_l(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["object_l_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "object_l_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=obj_mod)
        def vec_eef_to_object_r(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["object_r_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "object_r_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=obj_mod)
        def vec_eef_to_all_objects(obs_cache: Dict[str, Any]) -> np.ndarray:
            if set(
                ["vec_eef_to_object_a", "vec_eef_to_object_b", "vec_eef_to_object_l", "vec_eef_to_object_r"]
            ).issubset(obs_cache.keys()):
                return np.concatenate(
                    [
                        obs_cache["vec_eef_to_object_a"],
                        obs_cache["vec_eef_to_object_b"],
                        obs_cache["vec_eef_to_object_l"],
                        obs_cache["vec_eef_to_object_r"],
                    ]
                )
            else:
                return np.zeros(12)

        @sensor(modality=goal_mod)
        def vec_eef_to_object(obs_cache: Dict[str, Any]) -> np.ndarray:
            if set(
                ["vec_eef_to_object_a", "vec_eef_to_object_b"]
            ).issubset(obs_cache.keys()):
                if len(self._object_stack_body_ids) >= 2:
                    return obs_cache["vec_eef_to_object_a"]
                else:
                    return obs_cache["vec_eef_to_object_b"]
            else:
                return np.zeros(3)

        @sensor(modality=goal_mod)
        def vec_eef_to_target(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["next_target_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "next_target_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Boolean value to check if an object is gripped that is not yet successfully stacked
        @sensor(modality="object")
        def object_gripped(obs_cache: Dict[str, Any]) -> bool:
            obj_gripped = any(
                [
                    self._check_grasp(
                        gripper=self.robots[0].gripper,
                        object_geoms=manipulation_object
                    )
                    for manipulation_object in self.manipulation_objects
                    if manipulation_object.root_body not in self._object_stack_body_ids
                ]
            )

            return obj_gripped

        @sensor(modality=goal_mod)
        def vec_eef_to_next_objective(obs_cache: Dict[str, Any]) -> np.ndarray:
            if all(
                [key in obs_cache for key in ["vec_eef_to_free_object", "vec_eef_to_next_target", "object_gripped"]]
            ):
                return np.array(
                    obs_cache["vec_eef_to_next_target"]
                ) if obs_cache["object_gripped"] else np.array(
                    obs_cache["vec_eef_to_free_object"]
                )
            else:
                return np.zeros(3)

        sensors = [
            next_target_pos,
            object_a_pos,
            object_b_pos,
            object_l_pos,
            object_r_pos,
            all_objects_pos,
            vec_eef_to_object_a,
            vec_eef_to_object_b,
            vec_eef_to_object_l,
            vec_eef_to_object_r,
            vec_eef_to_all_objects,
            vec_eef_to_object,
            vec_eef_to_target,
            object_gripped,
            vec_eef_to_next_objective,
        ]

        names = [s.__name__ for s in sensors]

        # Create observables
        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )

        return observables

    def _setup_collision_info(self):
        """Extend the super method by white-listing the manipulation object for collision detection."""
        super()._setup_collision_info()
        self.whitelisted_collision_geoms = self.whitelisted_collision_geoms.union(
            {
                self.sim.model.geom_name2id(geom_name)
                for manipulation_object in self.manipulation_objects
                for geom_name in manipulation_object.contact_geoms
            }
        )

        self.whitelisted_collision_geoms = self.whitelisted_collision_geoms.union(
            {
                self.sim.model.geom_name2id(geom_name)
                for geom_name in self.l_cube.contact_geoms
            }
        )

        self.whitelisted_collision_geoms = self.whitelisted_collision_geoms.union(
            {
                self.sim.model.geom_name2id(geom_name)
                for geom_name in self.r_cube.contact_geoms
            }
        )

    def get_environment_state(self) -> CollaborativeStackingEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            state (CollaborativeStackingEnvState): The current state of the environment.
        """
        human_env_state = super().get_environment_state()
        return CollaborativeStackingEnvState(
            task_phase_value=self.task_phase.value,
            n_delayed_timesteps=self._n_delayed_timesteps,
            animation_loop_properties=self._animation_loop_properties,
            object_placements_list=self._object_placements_list,
            object_placements_list_index=self._object_placements_list_index,
            object_stack_body_ids=self._object_stack_body_ids.copy(),
            **asdict(human_env_state),
        )

    def set_environment_state(self, state: CollaborativeStackingEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            CollaborativeStackingEnvState: The state to be set.
        """
        super().set_environment_state(state)
        self.task_phase = CollaborativeStackingPhase(state.task_phase_value)
        self._n_delayed_timesteps = state.n_delayed_timesteps
        self._animation_loop_properties = state.animation_loop_properties
        self._object_placements_list = state.object_placements_list
        self._object_placements_list_index = state.object_placements_list_index
        self._object_stack_body_ids = state.object_stack_body_ids.copy()

        # Manage equalities
        if self.task_phase in [
            CollaborativeStackingPhase.WAIT_FOR_FOURTH,
            CollaborativeStackingPhase.RETREAT,
            CollaborativeStackingPhase.COMPLETE,
        ]:
            self._human_drop_left_object()
            self._human_drop_right_object()
        elif self.task_phase in [
            CollaborativeStackingPhase.PLACE_THIRD,
            CollaborativeStackingPhase.WAIT_FOR_SECOND,
        ]:
            if self.first_placing_hand == "left":
                self._human_drop_left_object()
            else:
                self._human_drop_right_object()

        if self.has_renderer:
            self._visualize()
