"""This file describes a collaborative hammering task for a single robot arm in a human environment.

The objective of this task is to hammer a nail into a board held by the human,
It is divided into three phases:
    1. Approach: The human approaches the robot holding the board.
    2. Present: The human presents the board to the robot. The robot should hammer the nail into the board.
    3. Retreat: Once the nail is hammered in, the human moves back to their starting position.

When setting `done_at_success` to `True`, the episode is terminated when animation of the human is finished.
Otherwise, these four phases are looped.

Reward is given once the task is completed, i.e., the animation is finished,
Optionally, this sparse reward can be augmented by a step step reward for having the nail driven sufficiently far
into the board.

Author
    Felix Trost (FT)

Changelog:
    15.09.23 FT File creation
"""
from enum import Enum
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, OrderedDict, Tuple, Union

import xml.etree.ElementTree as ET

import mujoco_py

import numpy as np
from robosuite.utils.observables import Observable, sensor
from scipy.spatial.transform import Rotation

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.models.objects.composite import HammerObject
from robosuite.utils.placement_samplers import ObjectPositionSampler
import robosuite.utils.transform_utils as T
from robosuite.utils.mjcf_utils import find_elements

from human_robot_gym.environments.manipulation.human_env import HumanEnv, HumanEnvState
from human_robot_gym.utils.mjcf_utils import xml_path_completion, rot_to_quat, quat_to_rot
from human_robot_gym.utils.animation_utils import (
    sample_animation_loop_properties, layered_sin_modulations
)


class CollaborativeHammeringPhase(Enum):
    """Enum for the different phases of the collaborative hammering task."""
    APPROACH = 0
    PRESENT = 1
    RETREAT = 3
    COMPLETE = 4


@dataclass
class CollaborativeHammeringEnvState(HumanEnvState):
    """Dataclass for encapsulating the state of the `CollaborativeHammeringCart` environment.

    Extends the `HumanEnvState` dataclass to include all variables necessary
    to restore the state of the `CollaborativeHammeringCart` environment.

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
        nail_placements (List[np.ndarray]): List of sampled positions for the nail in the board.
            During the episode this list is iterated over and the corresponding nail position is used.
        nail_placements_index (int): Index of the current nail position in the list of nail positions.
        task_phase_value (int): Value corresponding to the current task phase.
        n_delayed_timesteps (int): Number of timesteps the current animation is delayed
            because of the presenting loop phases.
        animation_loop_properties (List[Dict[str, Tuple[float, float]]]): Loop amplitudes and speed modifiers
            for all layered sines for all human animations sampled for the current episode.
    """
    nail_placements: List[np.ndarray]
    nail_placements_index: int
    task_phase_value: int
    n_delayed_timesteps: int
    animation_loop_properties: List[Tuple[List[float], List[float]]]


class CollaborativeHammeringCart(HumanEnv):
    """This class corresponds to a collaborative manufacturing task for a single robot arm in a human environment
    where the robot should hammer a nail into a board.

    It is divided into three phases:
        1. Approach: The human approaches the robot holding the board.
        2. Present: The human presents the board to the robot. The robot should hammer the nail into the board.
        3. Retreat: Once the nail is hammered in, the human moves back to their starting position.

    When setting `done_at_success` to `True`, the episode is terminated when animation of the human is finished.
    Otherwise, these four phases are looped.

    Reward is given once the task is completed, i.e., the animation is finished,
    Optionally, this sparse reward can be augmented by a step step reward for having the nail driven sufficiently far
    into the board.

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

        board_full_size (Tuple[float, float, float]): x, y, and z dimensions of the board in which the nail sticks.

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (None | float): Scales the normalized reward function by the amount specified.
            If `None`, environment reward remains unnormalized

        reward_shaping (bool): if `True`, use dense rewards, else use sparse rewards.

        goal_tolerance (float): Maximum distance the nail can stick out of the board for the goal to be achieved
            (relative to the initial distance)

        n_nail_placements_sampled_per_100_steps (int): How many nail placements to sample at resets per 100 steps in
            the horizon. After all placements of the list have been tried, restart from the first placement in the list.

        collision_reward (float): Reward to be given in the case of a collision.

        hammer_gripped_reward_bonus (float): Additional reward for having the hammer gripped.

        nail_hammered_in_reward (float): Additional reward for having the nail hammered in.

        task_reward (float): Reward to be given in the case of reaching the goal.

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

        collision_debounce_delay (float): Time in seconds after a human collision before new collisions may be detected.
            This is done to ensure no critical collisions are detected erraneously.

        seed (int): Random seed for `np.random`

        verbose (bool): If `True`, print out debug information

        done_at_collision (bool): If `True`, the episode is terminated when a collision occurs

        done_at_success (bool): If `True`, the episode is terminated when the goal is reached

        gripper_controllable (bool): If `True`, the gripper may be controlled by the agent. Otherwise, the gripper
            action is replaced by closing the gripper in any case. In the latter case, the hammer cannot be dropped.
            Defaults to `False`.

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
        table_full_size: Tuple[float, float, float] = (1.5, 2.0, 0.05),
        table_friction: Tuple[float, float, float] = (1.0, 5e-3, 1e-4),
        board_full_size: Tuple[float, float, float] = (1.0, 0.4, 0.03),
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        reward_shaping: bool = False,
        n_nail_placements_sampled_per_100_steps: int = 3,
        goal_tolerance: float = 0.05,
        collision_reward: float = -10,
        hammer_gripped_reward_bonus: float = 0,
        nail_hammered_in_reward: float = -1,
        task_reward: float = 1,
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
            "CollaborativeHammering/0",
            "CollaborativeHammering/1",
            "CollaborativeHammering/2",
            "CollaborativeHammering/3",
            "CollaborativeHammering/4",
            "CollaborativeHammering/5",
            "CollaborativeHammering/6",
            "CollaborativeHammering/7",
            "CollaborativeHammering/8",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 100,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        n_animations_sampled_per_100_steps: int = 1,
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        collision_debounce_delay: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
        gripper_controllable: bool = False,
    ):
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.board_full_size = board_full_size

        self.table_offset = np.array([0.0, 0.0, 0.82])

        self.hammer_gripped_reward_bonus = hammer_gripped_reward_bonus
        self.nail_hammered_in_reward = nail_hammered_in_reward

        self.goal_tolerance = goal_tolerance

        self.object_placement_initializer = None  # TODO
        self.nail_placement_initializer = None
        self._nail_placements = None
        self._nail_placements_index = 0
        self._n_nail_placements_to_sample_at_resets = max(
            int(horizon * n_nail_placements_sampled_per_100_steps / 100),
            1,
        )
        self.obstacle_placement_initializer = obstacle_placement_initializer
        self.board = None
        self.board_body_id = None
        self.hammer = None
        self.hammer_body_id = None

        self.task_phase: CollaborativeHammeringPhase = CollaborativeHammeringPhase.COMPLETE
        self._n_delayed_timesteps = None

        self.gripper_controllable = gripper_controllable

        self._animation_loop_properties = None

        self._lh_mocap_body_name = "lh_mocap"
        self._rh_mocap_body_name = "rh_mocap"

        self._lh_grip_body_name = "lh_grip"
        self._rh_grip_body_name = "rh_grip"

        self._lh_eq_name = "lh_eq"
        self._rh_eq_name = "rh_eq"
        # On PRESENT, replace the weld by a connect eq
        self._rh_connect_eq_name = "rh_backup_eq"

        self._lh_eq_id = None
        self._rh_eq_id = None
        self._rh_connect_eq_id = None

        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            done_at_collision=done_at_collision,
            done_at_success=done_at_success,
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
            collision_debounce_delay=collision_debounce_delay,
            seed=seed,
            verbose=verbose,
        )

    @property
    def animation_loop_amplitudes(self) -> List[float]:
        """Loop amplitudes for all layered sines for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index][0]

    @property
    def animation_loop_speeds(self) -> List[float]:
        """Loop speed modifiers for all layered sines for the current human animation."""
        return self._animation_loop_properties[self._human_animation_ids_index][1]

    @property
    def keyframes(self) -> List[int]:
        """Keyframes for the current human animation."""
        return self.human_animation_data[self.human_animation_id][1]["keyframes"]

    @property
    def animation_length(self) -> int:
        """Length of the current human animation."""
        return self.human_animation_data[self.human_animation_id][0]["Pelvis_pos_x"].shape[0]

    @property
    def nail_placement(self) -> np.ndarray:
        """Current nail placement."""
        return self._nail_placements[self._nail_placements_index]

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Override super `step` method to enter the `RETREAT` phase once the nail is hammered in.

        Args:
            action (np.ndarray): Action to be executed by the environment.

        Returns:
            Tuple[np.ndarray, float, bool, Dict[str, Any]]: The observation, reward, done, info
        """
        if not self.gripper_controllable:
            action[-1] = 1  # Always close the gripper

        obs, rew, done, info = super().step(action)

        if self.goal_reached:
            self._on_goal_reached()
            self.goal_reached = False

        if self.has_renderer:
            self._visualize()

        if self.task_phase == CollaborativeHammeringPhase.PRESENT and self._check_nail_hammered_in(
            achieved_goal=self._get_achieved_goal_from_obs(obs),
            desired_goal=self._get_desired_goal_from_obs(obs),
        ):
            self.task_phase = CollaborativeHammeringPhase.RETREAT
            self._human_take_board_from_table()

        return obs, rew, done, info

    def _check_nail_hammered_in(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
    ) -> bool:
        """Check if the nail is driven sufficiently far into the board. Threshold: `self.goal_tolerance`.

        Args:
            achieved_goal (List[float]): Achieved goal.
            desired_goal (List[float]): Desired goal.

        Returns:
            bool: whether the nail is driven sufficiently far into the board.
        """
        nail_hammering_progress = achieved_goal[1]
        return 1 - nail_hammering_progress < self.goal_tolerance

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Override super method to add the subobjective reward of the collaborative hammering task.

        The sparse reward function yields
            - `self.task_reward` when the animation is complete
            - `self.nail_hammered_in_reward` when the nail is driven sufficiently far into the board
            - `-1` otherwise
        with a bonus of
            - `self.hammer_gripped_reward` for having the hammer gripped.

        Args:
            achieved_goal (List[float]): Part of the robot state observation relevant for the goal.
            desired_goal (List[float]): The desired goal.
            info (Dict[str, Any]): The info dict.

        Returns:
            float: The sparse reward.
        """
        if self._check_success(achieved_goal=achieved_goal, desired_goal=desired_goal):
            return self.task_reward

        if self._check_nail_hammered_in(achieved_goal=achieved_goal, desired_goal=desired_goal):
            reward = self.nail_hammered_in_reward
        else:
            reward = -1.0

        hammer_gripped = bool(achieved_goal[0])

        if hammer_gripped:
            reward += self.hammer_gripped_reward_bonus

        return reward

    def _dense_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute the dense reward based on the achieved goal, the desired goal, and the info dict.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: dense environment reward
        """
        return 0  # TODO

    def _check_success(
        self,
        achieved_goal: List[float],
        desired_goal: List[float]
    ) -> bool:
        """Override the super class success condition to check if the animation is complete.

        If the robot does not manage to hammer the nail into the board,
        the animation does not exceed the `PRESENT` phase.
        """
        return self.task_phase == CollaborativeHammeringPhase.COMPLETE

    def _get_achieved_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        """Extract the achieved goal from the observation.

        The achieved goal includes
            - Whether the hammer is gripped.
            - How far the nail is hammered into the board.

        Args:
            observation (OrderedDict[str, Any]): The observation after the action is executed

        Returns:
            List[float]: The achieved goal
        """
        # TODO
        return np.concatenate(
            [
                [observation["hammer_gripped"]],
                [observation["nail_hammering_progress"]],
            ]
        ).tolist()

    def _get_desired_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        """Extract the desired goal from the observation.

        The desired goal contains:
            - the position of the nail.

        Args:
            observation (OrderedDict[str, Any]): The observation after the action is executed

        Returns:
            List[float]: The desired goal
        """
        return np.concatenate(
            [
                [observation["nail_pos"]],
            ]
        ).tolist()

    def _compute_animation_time(self, control_time: int) -> int:
        """Compute the current animation time.

        The human should perform an idle animation when waiting for the robot to hammer the nail into the board.
        To achieve this, we use multiple layered sine funtions to loop some frames around the first keyframe
        back and forth. The frequency and amplitudes of the sine functions are set according to the animation info
        json files with some randomization (see `sample_animation_loop_properties`).

        The number of frames the animation is delayed because of the loop phase
        is stored in `self._n_delayed_timesteps`.
        This value is used after the beginning of the object inspection to achieve a smooth transition.

        Args:
            control_time (float): The current control time.
        Returns:
            float: The current animation time.
        """
        animation_time = super()._compute_animation_time(control_time)
        classic_animation_time = animation_time

        # Progress to the `PRESENT` phase automatically depending on the animation
        if self.task_phase == CollaborativeHammeringPhase.APPROACH and animation_time > self.keyframes[0]:
            self.task_phase = CollaborativeHammeringPhase.PRESENT
            # self._human_drop_board_onto_table()
        # When in the `PRESENT` phase, loop the animation until the nail is hammered into the board
        elif (
            self.task_phase == CollaborativeHammeringPhase.PRESENT and
            animation_time > (self.keyframes[0] + self.keyframes[1]) / 2
        ):
            animation_time = int(
                layered_sin_modulations(
                    classic_animation_time=classic_animation_time,
                    modulation_start_time=(self.keyframes[0] + self.keyframes[1]) / 2,
                    amplitudes=self.animation_loop_amplitudes,
                    speeds=self.animation_loop_speeds,
                )
            )

            self._n_delayed_timesteps = classic_animation_time - animation_time
        # Subtract the delay from the present phase
        elif self.task_phase == CollaborativeHammeringPhase.RETREAT:
            animation_time -= self._n_delayed_timesteps

        # Once the animation is complete, freeze the animation time at the last frame
        if animation_time >= self.human_animation_length - 1:
            self.task_phase = CollaborativeHammeringPhase.COMPLETE
            animation_time = self.human_animation_length - 1

        return animation_time

    def _control_human(self, force_update: bool = True):
        """Override super method to update the position of the mocap objects."""
        super()._control_human(force_update=True)
        self.sim.forward()
        self._update_mocap_body_transforms()

    def _update_mocap_body_transforms(self):
        """Adjust position and rotation of the mocap objects to be at the human's hands."""
        lh_rot = quat_to_rot(self.sim.data.get_body_xquat(self.human.left_hand))
        rh_rot = quat_to_rot(self.sim.data.get_body_xquat(self.human.right_hand))

        lh_quat = rot_to_quat(lh_rot * Rotation.from_euler("y", -np.pi / 2))
        rh_quat = rot_to_quat(rh_rot * Rotation.from_euler("y", np.pi / 2))

        self.sim.data.set_mocap_pos(
            self._lh_mocap_body_name,
            self.sim.data.get_site_xpos(self.human.left_hand)  # + np.array([0, 0, -0.2])
        )

        self.sim.data.set_mocap_pos(
            self._rh_mocap_body_name,
            self.sim.data.get_site_xpos(self.human.right_hand)  # + np.array([0, -0.05, 0]),
        )

        self.sim.data.set_mocap_quat(
            self._lh_mocap_body_name,
            lh_quat,
        )

        self.sim.data.set_mocap_quat(
            self._rh_mocap_body_name,
            rh_quat,
        )

    def _reset_internal(self):
        """Reset the internal configuration of the environment."""
        # Set the desired new initial joint angles before resetting the robot.
        self.robots[0].init_qpos = np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, np.pi / 4])

        super()._reset_internal()

        self._control_human()
        self._reset_animation()
        self._put_hammer_into_gripper()

        self._animation_loop_properties = [
            sample_animation_loop_properties(
                animation_info=self.human_animation_data[human_animation_id][1]
            )
            for human_animation_id in self._human_animation_ids
        ]

        self._nail_placements = [
            np.array(self.nail_placement_initializer.sample()["nail_dummy"][0])
            for _ in range(self._n_nail_placements_to_sample_at_resets)
        ]
        self._nail_placements_index = 0

        self._reset_board()
        self._reset_nail()

    def _on_goal_reached(self):
        """Callback function that is called when the goal is reached.

        If episodes are not terminated on task completion,
        select a new placement for the nail and prepare a new task instance.
        """
        if self.done_at_success:
            return

        self._nail_placements_index = (
            (self._nail_placements_index + 1) % self._n_nail_placements_to_sample_at_resets
        )

        self._reset_board()
        self._reset_nail()

        self._progress_to_next_animation(
            animation_start_time=int(self.low_level_time / self.human_animation_step_length)
        )

    def _progress_to_next_animation(self, animation_start_time: float):
        """Extend super method to reset animation-specific internal variables."""
        super()._progress_to_next_animation(animation_start_time=animation_start_time)
        self._control_human()
        self._reset_animation()

    def _reset_animation(self):
        """Reset the task phase and the animation_specific internal variables."""
        self.task_phase = CollaborativeHammeringPhase.APPROACH
        self._n_delayed_timesteps = 0
        self._human_take_board_from_table()

    def _reset_board(self):
        """Return the board to its initial position."""
        self.sim.data.set_joint_qpos(
            self.board.joints[0],
            np.concatenate(
                [
                    self.table_offset + np.array([-0.5, 0, 0.5 * self.board_full_size[2]]),
                    [1, 0, 0, 0],
                ]
            )
        )

    def _reset_nail(self):
        """Move the nail to the placement position and pull it out of the board."""
        self.sim.model.body_pos[self.sim.model.body_name2id("nail_base")] = self.nail_placement
        self.sim.data.set_joint_qpos(
            "nail_head_joint0",
            self.sim.model.jnt_range[self.sim.model.joint_name2id("nail_head_joint0")][0]
        )

    def _put_hammer_into_gripper(self):
        """Securely grasp the hammer with the robot gripper."""
        rotation_quat = rot_to_quat(Rotation.from_euler("y", np.pi / 2))
        sim_time = self.sim.data.time

        for _ in range(100):
            self.sim.data.set_joint_qpos(
                "hammer_joint0",
                np.concatenate(
                    [
                        self._eef_xpos,
                        rotation_quat,
                    ]
                )
            )
            # Execute no-op action with gravity compensation
            self.sim.data.ctrl[self.robots[0]._ref_joint_actuator_indexes] = self.robots[
                0
            ].controller.torque_compensation
            self.robots[0].grip_action(gripper=self.robots[0].gripper, gripper_action=[1])
            self.sim.step()

        # Reset time value
        self.sim.data.time = sim_time

    def _human_drop_board_onto_table(self):
        """Make the human release the board with one hand so it lays flat on the table"""
        self.sim.model.eq_active[self.lh_eq_id] = 0
        self.sim.model.eq_active[self.rh_eq_id] = 0
        self.sim.model.eq_active[self.rh_connect_eq_id] = 1
        self.sim.forward()

    def _human_take_board_from_table(self):
        """Make the human pick up the board with the second hand."""
        self.sim.model.eq_active[self.lh_eq_id] = 1
        self.sim.model.eq_active[self.rh_eq_id] = 1
        self.sim.model.eq_active[self.rh_connect_eq_id] = 0

    def _get_default_nail_sample_space_boundaries(self) -> Tuple[float, float, float, float]:
        """Get the x and y boundaries of the nail sampling space.

        Returns:
            Tuple[float, float, float, float]:
                Boundaries of sampling space in the form (xmin, xmax, ymin, ymax)
        """
        board_x_half = self.board_full_size[0] * 0.5
        board_y_half = self.board_full_size[1] * 0.5

        return (
            board_x_half * 0.1,
            board_x_half * 0.9,
            board_y_half * -0.9,
            board_y_half * 0.9,
        )

    def _visualize(self):
        """Visualize the goal space and the sampling space of initial object positions."""
        self._visualize_nail_sample_space()

    def _visualize_nail_sample_space(self):
        """Draw a red box to indicate the sampling space of nail placements on the board."""
        boundaries = self._get_default_nail_sample_space_boundaries()

        offset = quat_to_rot(self.sim.data.body_xquat[self.board_body_id]).apply(
            np.array(
                [
                    (boundaries[1] + boundaries[0]) * 0.5,
                    (boundaries[3] + boundaries[2]) * 0.5,
                    0
                ]
            )
        )

        self.viewer.viewer.add_marker(
            pos=self.sim.data.body_xpos[self.board_body_id] + offset,
            type=6,
            size=[
                (boundaries[1] - boundaries[0]) * 0.5,
                (boundaries[3] - boundaries[2]) * 0.5,
                0.05,
            ],
            mat=quat_to_rot(
                self.sim.data.body_xquat[self.board_body_id]
            ).as_matrix(),
            label="",
            shininess=0,
            rgba=[1, 0, 0, 0.2],
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

        board_size = np.array(self.board_full_size) * 0.5

        self.board = BoxObject(
            name="board",
            size=board_size,
            rgba=[0.6, 0.6, 0.0, 1],
        )

        self.hammer = HammerObject(
            "hammer",
        )

        self.objects = [
            self.board,
            self.hammer,
        ]

        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=[],
        )

        nail_dummy = BoxObject(
            name="nail_dummy",
            size=np.array([0.01, 0.01, 0.01]),
        )

        nail_sampling_boundaries = self._get_default_nail_sample_space_boundaries()

        self.nail_placement_initializer = self._setup_placement_initializer(
            name="NailSampler",
            initializer=self.nail_placement_initializer,
            objects=[nail_dummy],
            x_range=[nail_sampling_boundaries[0], nail_sampling_boundaries[1]],
            y_range=[nail_sampling_boundaries[2], nail_sampling_boundaries[3]],
            reference_pos=[0, 0, 0],
            z_offset=self.board_full_size[2] * 0.5 + 0.001,
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
        """Add the mocap bodies, nail, and equalities to the model before the sim is created."""
        super()._postprocess_model()

        r_anchor = "-0.5 -0.2 0"
        l_anchor = "-0.1 0.2 0"

        self._add_mocap_bodies_to_model(l_anchor_pos=l_anchor, r_anchor_pos=r_anchor, visualize=False)
        self._add_grips_to_board(l_anchor_pos=l_anchor, r_anchor_pos=r_anchor, visualize=False)
        self._add_nail_to_board()
        self._add_equalities_to_model()

    def _add_mocap_bodies_to_model(
        self,
        l_anchor_pos: str,
        r_anchor_pos: str,
        visualize: bool = False
    ) -> Tuple[ET.Element, ET.Element]:
        """Add two mocap objects to the model. These are used to keep the board at the human's hands.

        Args:
            l_anchor_pos (str): The position attribute of the left mocap object.
            r_anchor_pos (str): The position attribute of the right mocap object.
            visualize (bool): Whether or not to visualize the mocap objects. Defaults to False.

        Returns:
            Tuple[ET.Element, ET.Element]: The nodes for both mocap objects.
        """
        lh_mocap_object = ET.Element(
            "body",
            name=self._lh_mocap_body_name,
            pos=l_anchor_pos,
            quat="0 0 0 1",
            mocap="true",
        )

        rh_mocap_object = ET.Element(
            "body",
            name=self._rh_mocap_body_name,
            pos=r_anchor_pos,
            quat="0 0 0 1",
            mocap="true",
        )

        if visualize:
            lh_mocap_object.append(ET.Element(
                "geom",
                name=f"{self._lh_mocap_body_name}_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="0 1 1 0.5",
            ))

            rh_mocap_object.append(ET.Element(
                "geom",
                name=f"{self._rh_mocap_body_name}_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="1 0 1 0.5",
            ))

        self.model.worldbody.extend(
            [
                lh_mocap_object,
                rh_mocap_object,
            ]
        )

        return lh_mocap_object, rh_mocap_object

    def _add_grips_to_board(
        self,
        l_anchor_pos: str,
        r_anchor_pos: str,
        visualize: bool = False
    ) -> Tuple[ET.Element, ET.Element]:
        """Add two bodies to the board that are connected to the mocap objects at the human's hands.

        Args:
            l_anchor_pos (str): The position attribute of the left grip.
            r_anchor_pos (str): The position attribute of the right grip.
            visualize (bool): Whether or not to visualize the grips. Defaults to False.

        Returns:
            Tuple[ET.Element, ET.Element]: The nodes for both grips.
        """
        board_elem = find_elements(
            root=self.model.root,
            tags="body",
            attribs={"name": "board_main"},
            return_first=True
        )

        l_grip = ET.Element(
            "body",
            name=self._lh_grip_body_name,
            pos=l_anchor_pos,
        )

        r_grip = ET.Element(
            "body",
            name=self._rh_grip_body_name,
            pos=r_anchor_pos,
        )

        if visualize:
            l_grip.append(ET.Element(
                "geom",
                name=f"{self._lh_grip_body_name}_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="0 1 0 0.5",
            ))

            r_grip.append(ET.Element(
                "geom",
                name=f"{self._rh_grip_body_name}_g0_vis",
                type="box",
                size="0.05 0.05 0.05",
                contype="0",
                conaffinity="0",
                group="1",
                rgba="1 0 0 0.5",
            ))

        board_elem.append(l_grip)
        board_elem.append(r_grip)

        return l_grip, r_grip

    def _add_nail_to_board(self) -> ET.Element:
        """Add the nail to the model from an xml file."""
        board_elem = find_elements(
            root=self.model.root,
            tags="body",
            attribs={"name": "board_main"},
            return_first=True
        )

        nail = ET.parse(xml_path_completion("objects/nail.xml")).getroot()

        board_elem.append(nail)

        return nail

    def _add_equalities_to_model(self) -> Tuple[ET.Element, ET.Element, ET.Element]:
        """Add the equality constraints to the model that should connect the mocap bodies at the human's hands
        with the grip bodies at the board.

        Adds three equalities:
            - A connect equality for the left hand that is deactivated once the human lays the board onto the table
            - A weld equality for the right hand that is replaced by a
            - connect equality for the right hand when the human lays the board onto the table
        """
        equalities = [
            ET.Element(
                "connect",
                name=self._lh_eq_name,
                body1=self._lh_grip_body_name,
                body2=self._lh_mocap_body_name,
                anchor="0 0 0",
                active="true",
                # solref="0.01 1",
            ),
            ET.Element(
                "weld",
                name=self._rh_eq_name,
                body1=self._rh_grip_body_name,
                body2=self._rh_mocap_body_name,
                relpose="0 0 0 0 0 0 1",
                active="true",
                # solref="0.01 1",
            ),
            ET.Element(
                "connect",
                name=self._rh_connect_eq_name,
                body1=self._rh_grip_body_name,
                body2=self._rh_mocap_body_name,
                anchor="0 0 0",
                active="false",
                # solref="0.01 1",
            ),
        ]

        self.model.equality.extend(equalities)

        return tuple(equalities)

    def _setup_references(self):
        """Add references to task-specific objects."""
        super()._setup_references()

        self.sim.model.opt.noslip_iterations = 20
        self.board_body_id = self.sim.model.body_name2id(self.board.root_body)
        self.hammer_body_id = self.sim.model.body_name2id(self.hammer.root_body)

        self.lh_eq_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, self._lh_eq_name,
        )

        self.rh_eq_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, self._rh_eq_name,
        )

        self.rh_connect_eq_id = mujoco_py.functions.mj_name2id(
            self.sim.model, mujoco_py.const.OBJ_EQUALITY, self._rh_connect_eq_name,
        )

        assert self.lh_eq_id != -1
        assert self.rh_eq_id != -1
        assert self.rh_connect_eq_id != -1

    def _setup_observables(self) -> OrderedDict[str, Observable]:
        """Setup environment-specific observation values."""
        observables = super()._setup_observables()

        robot_prefix = self.robots[0].robot_model.naming_prefix

        if (obs_key := f"{robot_prefix}joint_pos") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}joint_vel") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}eef_pos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := "human_joint_pos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}joint_pos_cos") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}joint_pos_sin") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := f"{robot_prefix}gripper_qpos") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}gripper_qvel") in observables:
            observables[obs_key].set_active(True)
        if (obs_key := f"{robot_prefix}eef_quat") in observables:
            observables[obs_key].set_active(False)
        if (obs_key := "gripper_pos") in observables:
            observables[obs_key].set_active(False)

        goal_mod = "goal"
        obj_mod = "object"

        # Absolute position of the nail in Cartesian space
        @sensor(modality=goal_mod)
        def nail_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.get_body_xpos("nail_head")

        # Vector from end-effector to the nail
        @sensor(modality=goal_mod)
        def vec_eef_to_nail(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["nail_pos"] - np.array(obs_cache[f"{robot_prefix}eef_pos"]))
                if "nail_pos" in obs_cache and f"{robot_prefix}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Absolute position of the hammer in Cartesian space
        @sensor(modality=obj_mod)
        def hammer_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xpos[self.hammer_body_id]

        # Rotation quaternion of the hammer
        @sensor(modality=obj_mod)
        def hammer_quat(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.sim.data.body_xquat[self.hammer_body_id]

        # Vector from the end-effector to the hammer
        @sensor(modality=obj_mod)
        def vec_eef_to_hammer(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["hammer_pos"] - np.array(obs_cache[f"{robot_prefix}eef_pos"]))
                if "hammer_pos" in obs_cache and f"{robot_prefix}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Rotation quaternion to represent the rotation difference of end-effector and hammer
        @sensor(modality=obj_mod)
        def quat_eef_to_hammer(obs_cache: Dict[str, Any]) -> np.ndarray:
            if "robot0_eef_quat" not in obs_cache or "object_quat" not in obs_cache:
                return np.zeros(4)

            quat = rot_to_quat(
                quat_to_rot(obs_cache["object_quat"]) * quat_to_rot(obs_cache["robot0_eef_quat"]).inv()
            )
            return T.convert_quat(np.array(quat), "xyzw")

        # Absolute position of the board in Cartesian space
        @sensor(modality=obj_mod)
        def board_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return np.array(self.sim.data.body_xpos[self.board_body_id])

        # Rotation quaternion of the board
        @sensor(modality=obj_mod)
        def board_quat(obs_cache: Dict[str, Any]) -> np.ndarray:
            return T.convert_quat(self.sim.data.body_xquat[self.board_body_id], to="xyzw")

        # Vector from end-effector to board
        @sensor(modality=obj_mod)
        def vec_eef_to_board(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["board_pos"] - np.array(obs_cache[f"{robot_prefix}eef_pos"]))
                if "board_pos" in obs_cache and f"{robot_prefix}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Rotation quaternion to represent the rotation difference of end-effector and board
        @sensor(modality=obj_mod)
        def quat_eef_to_board(obs_cache: Dict[str, Any]) -> np.ndarray:
            if "robot0_eef_quat" not in obs_cache or "object_quat" not in obs_cache:
                return np.zeros(4)

            quat = rot_to_quat(
                quat_to_rot(obs_cache["object_quat"]) * quat_to_rot(obs_cache["robot0_eef_quat"]).inv()
            )
            return T.convert_quat(np.array(quat), "xyzw")

        # Boolean value reflecting whether the hammer is within the gripper
        @sensor(modality=obj_mod)
        def hammer_gripped(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self._check_grasp(
                gripper=self.robots[0].gripper,
                object_geoms=self.hammer,
            )

        # Scalar value reflecting how far the nail is driven into the board. Normalized to [0,1]
        @sensor(modality=goal_mod)
        def nail_hammering_progress(obs_cache: Dict[str, Any]) -> np.ndarray:
            jointpos = self.sim.data.get_joint_qpos("nail_head_joint0")

            limits = self.sim.model.jnt_range[
                self.sim.model.joint_name2id("nail_head_joint0")
            ]

            jointpos = (jointpos - limits[0]) / (limits[1] - limits[0])
            jointpos = np.clip(jointpos, 0, 1)

            return jointpos

        sensors = [
            nail_pos,
            vec_eef_to_nail,
            hammer_pos,
            hammer_quat,
            vec_eef_to_hammer,
            quat_eef_to_hammer,
            board_pos,
            board_quat,
            vec_eef_to_board,
            quat_eef_to_board,
            hammer_gripped,
            nail_hammering_progress,
        ]

        names = [s.__name__ for s in sensors]

        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )

        return observables

    def _setup_collision_info(self):
        """Extend the super method by white-listing the hammer for collision detection.
        Note that the board is not white-listed.
        """
        super()._setup_collision_info()
        self.whitelisted_collision_geoms = self.whitelisted_collision_geoms.union(
            {
                self.sim.model.geom_name2id(geom_name)
                for geom_name in self.hammer.contact_geoms
            }
        )

    def get_environment_state(self) -> CollaborativeHammeringEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            state (CollaborativeHammeringEnvState): The current state of the environment.
        """
        human_env_state = super().get_environment_state()
        return CollaborativeHammeringEnvState(
            task_phase_value=self.task_phase.value,
            n_delayed_timesteps=self._n_delayed_timesteps,
            animation_loop_properties=self._animation_loop_properties,
            nail_placements=self._nail_placements,
            nail_placements_index=self._nail_placements_index,
            **asdict(human_env_state),
        )

    def set_environment_state(self, state: CollaborativeHammeringEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            CollaborativeHammeringEnvState: The state to be set.
        """
        super().set_environment_state(state)
        self.task_phase = CollaborativeHammeringEnvState(state.task_phase_value)
        self._n_delayed_timesteps = state.n_delayed_timesteps
        self._animation_loop_properties = state.animation_loop_properties
        self._nail_placements = state.nail_placements
        self._nail_placements_index = state.nail_placements_index

        self.sim.model.body_pos[self.sim.model.body_name2id("nail_base")] = self.nail_placement

        if self.has_renderer:
            self._visualize()
