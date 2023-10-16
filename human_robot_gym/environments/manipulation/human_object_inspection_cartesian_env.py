"""This file describes a lift task for a single robot arm in a human environment.

The objective of this task is to lift the object to be in front of the human's head for inspection.
It is divided into four phases:
    1. Approach: The human walks to the table
    2. Ready: The human waits for the robot to raise the object
    3. Inspection: When the object has reached the target zone, the human inspects the object.
    4. Retreat: The human walks back to the starting position.

If the object exits the target zone in the inspection phase, the human returns to the ready phase and
the inspection has to be restarted.

When using a fixed horizon, these four phases are looped until the horizon is reached.
Otherwise, the episode ends with the animation.

Reward is given once the task is done, i.e. the animation is finished.
Optionally, this sparse reward can be reformulated as a step reward to yield additional gratification
for grabbing the object and bringing it to the target zone or as a dense reward incorporating distances
to the object and the target zone.

Author: Felix Trost

Changelog:
    23.05.23 FT File created
"""
from typing import Any, Dict, List, Optional, Tuple, Union
from enum import Enum
from dataclasses import asdict, dataclass

import numpy as np

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.placement_samplers import ObjectPositionSampler

from human_robot_gym.environments.manipulation.pick_place_human_cartesian_env import (
    PickPlaceHumanCart, PickPlaceHumanCartEnvState
)
from human_robot_gym.utils.mjcf_utils import xml_path_completion
from human_robot_gym.utils.animation_utils import layered_sin_modulations, sample_animation_loop_properties


class ObjectInspectionPhase(Enum):
    """Enum for the different phases of the object inspection task."""
    APPROACH = 0
    READY = 1
    INSPECTION = 2
    RETREAT = 3
    COMPLETE = 4


@dataclass
class HumanObjectInspectionCartEnvState(PickPlaceHumanCartEnvState):
    """Dataclass for encapsulating the state of the `HumanObjectInspectionCart` environment.

    Extends the `PickPlaceHumanCartEnvState` dataclass to include all variables necessary
    to restore a `HumanObjectInspectionCart` environment state.

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
        target_positions (List[np.ndarray]): List of target positions. During the episode this list is
            iterated over and the corresponding target_position is used.
        target_positions_index (int): Index of the current target position in the list of target_position.
        task_phase (int): Value corresponding to the current task phase.
        n_delayed_timesteps (int): Number of timesteps the current animation is delayed because of the loop phase.
        animation_loop_properties (List[Tuple[float, float]]):  Loop amplitudes and speed modifiers
            for all layered sines for all human animations sampled for the current episode.
    """
    task_phase_value: int
    n_delayed_timesteps: int
    animation_loop_properties: List[Tuple[List[float], List[float]]]


class HumanObjectInspectionCart(PickPlaceHumanCart):
    """This class corresponds to a lift task for a single robot arm in a human environment
    where the robot should lift the object to be in front of the human's head for inspection.

    It is divided into four phases:
        1. Approach: The human walks to the table
        2. Ready: The human waits for the robot to raise the object
        3. Inspection: When the object has reached the target zone, the human inspects the object.
            Reward is given for every step the object is in the target zone.
        4. Retreat: The human walks back to the starting position.

    If the object exits the target zone in the inspection phase, the human returns to the ready phase and
    the inspection has to be restarted.

    Reward is given once the task is done, i.e. the animation is finished.
    Optionally, this sparse reward can be reformulated as a step reward to yield additional gratification
    for grabbing the object and bringing it to the target zone or as a dense reward incorporating distances
    to the object and the target zone.

    When using a fixed horizon, these four phases are looped until the horizon is reached.
    Otherwise, the episode ends with the animation.

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

        object_full_size (Tuple[float, float, float]): x, y, and z dimensions of the cube object that should be moved.

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (None | float): Scales the normalized reward function by the amount specified.
            If None, environment reward remains unnormalized

        reward_shaping (bool): if `True`, use dense rewards, else use sparse rewards.

        goal_dist (float): Distance threshold for reaching the goal.

        n_object_placements_sampled_per_100_steps (int): How many object placements to sample at resets
            per 100 steps in the horizon.
            After all objects of the list have been placed, restart from the first in the list.
            This is done to ensure the same list of objects can be placed when loading the env state from a file.

        goal_exit_tolerance (float): Absolute tolerance by which the goal distance is increased once the goal zone is
            entered. This is used to avoid oscillating rapidly between WAIT and INSPECTION phases.

        collision_reward (float): Reward to be given in the case of a collision.

        task_reward (float): Reward to be given in the case of completing the task (i.e. finishing the animation).

        object_at_target_reward (float): Reward if the object is within `goal_dist` of the target position.
            Applied when `reward_shaping=False`.

        object_gripped_reward (float): Additional reward for gripping the object when `reward_shaping=False`.
            If object is not gripped: `reward = -1`.
            If object gripped but not at the target: `object_gripped_reward`.
            If object is at the target: `reward = object_at_target_reward`.
            If task completed (animation finished): `reward = task_reward`.
            `object_gripped_reward` defaults to `-1`.

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

        collision_debounce_delay (float): Time in seconds after a human collision before new collisions may be detected.
            This is done to ensure no critical collisions are detected erraneously.

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
        table_full_size: Tuple[float, float, float] = (1.5, 2.0, 0.05),
        table_friction: Tuple[float, float, float] = (1.0, 5e-3, 1e-4),
        object_full_size: Tuple[float, float, float] = (0.04, 0.04, 0.04),
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        reward_shaping: bool = False,
        goal_dist: float = 0.15,
        n_object_placements_sampled_per_100_steps: int = 3,
        goal_exit_tolerance: float = 0.02,
        collision_reward: float = -10,
        task_reward: float = 1,
        object_at_target_reward: float = -1,
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
            "ObjectInspection/0",
            "ObjectInspection/1",
            "ObjectInspection/2",
            "ObjectInspection/3",
            "ObjectInspection/4",
            "ObjectInspection/5",
            "ObjectInspection/6",
            "ObjectInspection/7",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 30,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        n_animations_sampled_per_100_steps: int = 2,
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        collision_debounce_delay: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
    ):
        self.task_phase: ObjectInspectionPhase = ObjectInspectionPhase.COMPLETE
        # Number of timesteps the animation is delayed because of the loop phase
        self._n_delayed_timesteps = None
        # Characteristics of the loop phase, set according to the animation info json files
        # with some randomization
        self._animation_loop_properties = None

        self.object_at_target_reward = object_at_target_reward
        self.goal_exit_tolerance = goal_exit_tolerance

        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            table_full_size=table_full_size,
            table_friction=table_friction,
            object_full_size=object_full_size,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            reward_scale=reward_scale,
            reward_shaping=reward_shaping,
            goal_dist=goal_dist,
            n_object_placements_sampled_per_100_steps=n_object_placements_sampled_per_100_steps,
            n_targets_sampled_per_100_steps=0,
            collision_reward=collision_reward,
            task_reward=task_reward,
            object_gripped_reward=object_gripped_reward,
            object_placement_initializer=object_placement_initializer,
            target_placement_initializer=None,
            obstacle_placement_initializer=obstacle_placement_initializer,
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
            done_at_collision=done_at_collision,
            done_at_success=done_at_success,
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
    def target_pos(self) -> np.ndarray:
        """Current position of the target.

        Returns a position specified in the info json file of the current animation.

        Returns:
            np.ndarray: The current target position.
        """
        target_pos = np.array(self.human_animation_data[self.human_animation_id][1]["target_pos"])

        target_pos += self.human_pos_offset

        return target_pos

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Override super `step` method to enter the `INSPECTION` phase when the object first is in the target zone.

        if the object exits the target zone in the inspection phase, the human returns to the ready phase
        and the inspection has to be restarted.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            Tuple[np.ndarray, float, bool, Dict[str, Any]]: The observation, reward, done flag, and info dict.
        """
        obs, reward, done, info = super().step(action)

        achieved_goal = self._get_achieved_goal_from_obs(obs)
        desired_goal = self._get_desired_goal_from_obs(obs)

        if self.task_phase == ObjectInspectionPhase.READY and self._check_object_in_target_zone(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
        ):
            self.task_phase = ObjectInspectionPhase.INSPECTION
        elif self.task_phase == ObjectInspectionPhase.INSPECTION and not self._check_object_in_target_zone(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
            tolerance=self.goal_exit_tolerance,  # Add a tolerance to avoid oscillating between WAIT and INSPECTION
        ):
            self.task_phase = ObjectInspectionPhase.READY

        return obs, reward, done, info

    def _on_goal_reached(self):
        """Select a new animation and sample new object locations when the goal is reached."""
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
        self.manipulation_object = BoxObject(
            name="manipulation_object",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )
        self.objects = [self.manipulation_object]
        object_bin_boundaries = self._get_default_object_bin_boundaries()
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=self.objects,
            x_range=[object_bin_boundaries[0], object_bin_boundaries[1]],
            y_range=[object_bin_boundaries[2], object_bin_boundaries[3]],
        )

        # << OBSTACLES >>
        self._setup_collision_objects(
            add_table=True,
            add_base=True,
            safety_margin=0.00
        )
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Override super method to change the success condition to whether or not the animation is complete.
        If the object never reaches the target zone the inspection phase is never entered
        and the episode is not successful.

        Args:
            achieved_goal (List[float]): Part of the robot state observation relevant for the goal.
            desired_goal (List[float]): The desired goal.
        Returns:
            bool: Whether or not the episode is successful.
        """
        return self.task_phase == ObjectInspectionPhase.COMPLETE

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Override super method to add the subobjective rewards of the object inspection task.

        The sparse reward function yields
            - `self.task_reward` when the animation is complete
            - `self.object_at_target_reward` when the object is in the target zone
            - `self.object_gripped_reward` when the object is gripped
            - `-1` otherwise

        Args:
            achieved_goal (List[float]): Part of the robot state observation relevant for the goal.
            desired_goal (List[float]): The desired goal.
            info (Dict[str, Any]): The info dict.

        Returns:
            float: The sparse reward.
        """
        object_gripped = bool(achieved_goal[6])

        if self._check_success(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal
        ):
            return self.task_reward
        elif self._check_object_in_target_zone(achieved_goal=achieved_goal, desired_goal=desired_goal):
            return self.object_at_target_reward
        elif object_gripped:
            return self.object_gripped_reward
        else:
            return -1

    def _compute_animation_time(self, control_time: float) -> float:
        """Compute the current animation time.

        The human should perform an idle animation when waiting for the object to enter the target zone.
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

        keyframes = self.human_animation_data[self.human_animation_id][1]["keyframes"]

        # Enter the `READY` phase when past the first keyframe
        if animation_time > keyframes[0] and self.task_phase == ObjectInspectionPhase.APPROACH:
            self.task_phase = ObjectInspectionPhase.READY

        # When in the `READY` phase, loop the animation until the object enters the target zone
        if self.task_phase == ObjectInspectionPhase.READY:
            animation_time = int(
                layered_sin_modulations(
                    classic_animation_time=animation_time,
                    modulation_start_time=keyframes[0],
                    amplitudes=self.animation_loop_amplitudes,
                    speeds=self.animation_loop_speeds,
                )
            )

            self._n_delayed_timesteps = classic_animation_time - animation_time
        else:
            animation_time -= self._n_delayed_timesteps

        # Enter the `RETREAT` phase when past the second keyframe
        if animation_time > keyframes[1]:
            self.task_phase = ObjectInspectionPhase.RETREAT

        # Once the animation is complete, freeze the animation time at the last frame
        if animation_time >= self.human_animation_length - 1:
            self.task_phase = ObjectInspectionPhase.COMPLETE
            animation_time = self.human_animation_length - 1

        return animation_time

    def _control_human(self, force_update: bool = True):
        """Override super method to force update the human every frame to avoid glitches in the wait animation."""
        super()._control_human(force_update=True)

    def _reset_animation(self):
        """Reset the task phase and the animation-specific internal variables."""
        self.task_phase = ObjectInspectionPhase.APPROACH
        self._n_delayed_timesteps = 0

    def _progress_to_next_animation(self, animation_start_time: int):
        """Extend super method to reset animation-specific internal variables.

        Args:
            animation_start_time (int): The current control time to set the animation start time to.
        """
        super()._progress_to_next_animation(animation_start_time)
        self._reset_animation()

    def _reset_internal(self):
        """Extend super method to reset internal variables concerning task and animation."""
        super()._reset_internal()
        self._reset_animation()

        self._animation_loop_properties = [
            sample_animation_loop_properties(
                animation_info=self.human_animation_data[human_animation_id][1],
            )
            for human_animation_id in self._human_animation_ids
        ]

    def _sample_target_pos(self) -> np.ndarray:
        """Override the parent function to return the current target position.

        In contrast to the basic pick place environment, the target position is not sampled but
        specified in the animation info json file.

        Returns:
            np.ndarray: The current target position.
        """
        return self.target_pos

    def _get_default_object_bin_boundaries(self) -> Tuple[float, float, float, float]:
        """Get the x and y boundaries of the object sampling space.

        Returns:
            Tuple[float, float, float, float]:
                Boundaries of sampling space in the form (xmin, xmax, ymin, ymax)
        """
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05

        return (
            bin_x_half * 0.35,
            bin_x_half * 0.75,
            -bin_y_half * 0.15,
            bin_y_half * 0.15,
        )

    def _visualize(self):
        """Visualize the goal space and the sampling space of initial object positions.

        The goal is only visualized during the inspection phase or .
        """
        if self.task_phase in [ObjectInspectionPhase.READY, ObjectInspectionPhase.INSPECTION]:
            self._visualize_goal()
        self._visualize_object_sample_space()

    def get_environment_state(self) -> HumanObjectInspectionCartEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            state (HumanObjectInspectionCartEnvState): The current state of the environment.
        """
        pick_place_env_state = super().get_environment_state()
        return HumanObjectInspectionCartEnvState(
            task_phase_value=self.task_phase.value,
            n_delayed_timesteps=self._n_delayed_timesteps,
            animation_loop_properties=self._animation_loop_properties,
            **asdict(pick_place_env_state),
        )

    def set_environment_state(self, state: HumanObjectInspectionCartEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            HumanObjectInspectionCartEnvState: The state to be set.
        """
        super().set_environment_state(state)
        self.task_phase = ObjectInspectionPhase(state.task_phase_value)
        self._n_delayed_timesteps = state.n_delayed_timesteps
        self._animation_loop_properties = state.animation_loop_properties
