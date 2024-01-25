"""This file describes a pick-and-place task with cartesian action space
for a single robot with a human doing tasks nearby.

This class is based on the human environment.

Author:
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
    16.05.23 FT Formatted docstrings
"""
from typing import Any, Dict, List, Optional, OrderedDict, Tuple, Union
from dataclasses import asdict, dataclass

import numpy as np

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import ObjectPositionSampler

from human_robot_gym.utils.mjcf_utils import xml_path_completion
from human_robot_gym.environments.manipulation.human_env import HumanEnv, HumanEnvState


@dataclass
class PickPlaceHumanCartEnvState(HumanEnvState):
    """Dataclass for encapsulating the state of the PickPlaceHumanCart environment.
    Extends the `HumanEnvState` dataclass to include all variables necessary
    to restore a `PickPlaceHumanCart` environment state.

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
    """
    object_placements_list: List[List[Tuple[str, np.ndarray]]]
    object_placements_list_index: int
    target_positions: List[np.ndarray]
    target_positions_index: int


class PickPlaceHumanCart(HumanEnv):
    """This class corresponds to the pick place task for a single robot arm in a human environment.

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
            If `None`, environment reward remains unnormalized

        reward_shaping (bool): if `True`, use dense rewards, else use sparse rewards.

        goal_dist (float): Distance threshold for reaching the goal.

        n_object_placements_sampled_per_100_steps (int): How many object placements to sample at resets
            per 100 steps in the horizon.
            After all objects of the list have been placed, restart from the first in the list.
            This is done to ensure the same list of objects can be placed when loading the env state from a file.

        n_targets_sampled_per_100_steps (int): How many targets to sample at resets per 100 steps in the horizon.
            After all goals of the list have been reached, restart from the first in the list.
            This is done to ensure the same list of goals can be played when loading the env state from a file.
            Setting `n_targets_sampled_per_100_steps != n_object_placements_sampled_per_100_steps`
            yields more possible object-goal combinations.

        collision_reward (float): Reward to be given in the case of a collision.

        task_reward (float): Reward to be given in the case of completing the task.

        object_gripped_reward (float): Additional reward for gripping the object when `reward_shaping=False`.
            If object is not gripped: `reward = -1`.
            If object gripped but not at the target: `object_gripped_reward`.
            If object is at the target: `reward = task_reward`.
            `object_gripped_reward` defaults to `-1`.

        object_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a `UniformRandomSampler`
            is used by default.
            Objects are elements that can and should be manipulated.

        target_placement_initializer (ObjectPositionSampler): if provided, will
            be used to generate target locations every time the previous target was reached
            and on resets. If not set, a `UniformRandomSampler` is used by default.
            Targets specify the coordinates to which the object should be moved.

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
        goal_dist: float = 0.1,
        n_object_placements_sampled_per_100_steps: int = 3,
        n_targets_sampled_per_100_steps: int = 3,
        collision_reward: float = -10,
        task_reward: float = 1,
        object_gripped_reward: float = -1,
        object_placement_initializer: Optional[ObjectPositionSampler] = None,
        target_placement_initializer: Optional[ObjectPositionSampler] = None,
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
            "CMU/62_01",
            "CMU/62_03",
            "CMU/62_04",
            "CMU/62_07",
            "CMU/62_09",
            "CMU/62_10",
            "CMU/62_12",
            "CMU/62_13",
            "CMU/62_14",
            "CMU/62_15",
            "CMU/62_16",
            "CMU/62_18",
            "CMU/62_19",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 120,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        n_animations_sampled_per_100_steps: int = 5,
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        collision_debounce_delay: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
    ):  # noqa: D107
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.object_full_size = object_full_size
        # settings for table top (hardcoded since it's not an essential part of the environment)
        self.table_offset = np.array((0.0, 0.0, 0.82))
        self.object_gripped_reward = object_gripped_reward
        self.goal_dist = goal_dist
        self._object_placements_list = None
        self._object_placements_list_index = 0
        self._n_objects_to_sample_at_resets = max(
            int(horizon * n_object_placements_sampled_per_100_steps / 100),
            1,
        )
        self._target_positions = None
        self._target_positions_index = 0
        self._n_targets_to_sample_at_resets = max(
            int(horizon * n_targets_sampled_per_100_steps / 100),
            1,
        )
        # object placement initializer
        self.object_placement_initializer = object_placement_initializer
        self.target_placement_initializer = target_placement_initializer
        self.obstacle_placement_initializer = obstacle_placement_initializer

        self.manipulation_object = None
        self.manipulation_object_body_id = None
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
            done_at_collision=done_at_collision,
            done_at_success=done_at_success,
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
    def object_placements(self) -> List[Tuple[str, np.ndarray]]:
        """Get the name and current initial object joint positions"""
        return self._object_placements_list[self._object_placements_list_index]

    @property
    def target_pos(self) -> np.ndarray:
        """Get the current target position in the list of target positions."""
        return self._target_positions[self._target_positions_index]

    def step(self, action: np.ndarray) -> Tuple[OrderedDict[str, Any], float, bool, Dict[str, Any]]:
        """Override base step function.

        Adds the target position as a sphere to the visualizer.
        The object and target sampling spaces are shown as red and blue boxes.

        Args:
            action (np.ndarray): Action to execute within the environment
        Returns:
            4-tuple:
                - (OrderedDict[str, Any]) observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        Raises:
            ValueError: [Steps past episode termination]
        """
        obs, reward, done, info = super().step(action)
        if self.goal_reached:
            self._on_goal_reached()
            self.goal_reached = False
        if self.has_renderer:
            self._visualize()

        return obs, reward, done, info

    def _on_goal_reached(self):
        """Callback function that is called when the goal is reached.

        Samples a new position for the object and a new target position.
        """
        # if goal is reached, calculate a new goal.
        self._target_positions_index = (
            (self._target_positions_index + 1) % self._n_targets_to_sample_at_resets
        )
        self._object_placements_list_index = (
            (self._object_placements_list_index + 1) % self._n_objects_to_sample_at_resets
        )
        for joint_name, joint_qpos in self.object_placements:
            self.sim.data.set_joint_qpos(joint_name, joint_qpos)

    def _get_info(self) -> Dict[str, Any]:
        """Return the info dictionary of this step.

        Returns
            info dict containing:
                * collision: whether there was a collision
                * collision_type: type of collision
                * timeout: whether timeout was reached
                * failsafe_intervention: if the failsafe controller intervened
                    in this step
        """
        info = super()._get_info()
        # Add more info if wanted (do not forget to pass this to the tensorboard callback)
        # info["my_cool_info"] = 0
        return info

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute the sparse reward based on the achieved goal, the desired goal, and the info dict.

        The sparse reward function yields
            - `self.task_reward` if the target is reached,
            - `self.object_gripped_reward` if the object is gripped but the target is not reached, and
            - `-1` otherwise.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: sparse environment reward
        """
        object_gripped = bool(achieved_goal[6])

        # sparse reward
        if self._check_success(achieved_goal, desired_goal):
            return self.task_reward
        elif object_gripped:
            return self.object_gripped_reward
        else:
            return -1

    def _dense_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a dense guidance reward based on the achieved goal, the desired goal, and the info dict.

        The dense reward incorporates the Euclidean distances between robot end effector and object,
        and between object and target.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: dense environment reward
        """
        eef_pos = np.array(achieved_goal[:3])
        obj_pos = np.array(achieved_goal[3:6])

        eef_2_obj = np.linalg.norm(obj_pos - eef_pos)
        obj_2_target = np.linalg.norm(np.array(desired_goal) - obj_pos)
        return -(eef_2_obj * 0.2 + obj_2_target) * 0.1

    def _check_success(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
    ) -> bool:
        """Check if the desired goal was reached.

        Checks if the object is at the target position.
        The distance metric is a RMSE and the threshold is `self.goal_dist`.
        This function can only be called for one sample.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for goal
            desired_goal (List[float]): the desired goal
        Returns:
            bool: whether the goal was reached
        """
        return self._check_object_in_target_zone(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
        )

    def _check_object_in_target_zone(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        tolerance: Optional[float] = None,
    ) -> bool:
        """Check if the object is within the target zone.
        The distance metric is a RMSE and the threshold is `self.goal_dist`.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for goal
            desired_goal (List[float]): the desired goal
            tolerance (Optional[float]): optional tolerance to `self.goal_dist`. Allows to specify a larger
                target zone. Defaults to `None`.
        Returns:
            bool: whether the object is within the target zone
        """
        dist = np.linalg.norm(np.array(achieved_goal[3:6]) - np.array(desired_goal))

        if tolerance is not None:
            dist -= tolerance

        return dist <= self.goal_dist

    def _get_achieved_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        """Extract the achieved goal from the observation.

        The achieved goal includes the end effector position, object position, and whether the object is gripped.

        Args:
            observation (OrderedDict[str, Any]): The observation after the action is executed

        Returns:
            List[float]: The achieved goal
        """
        pf = self.robots[0].robot_model.naming_prefix
        return np.concatenate(
            [
                observation[f"{pf}eef_pos"],
                observation["object_pos"],
                [observation["object_gripped"]],
            ]
        ).tolist()

    def _get_desired_goal_from_obs(
        self,
        observation: OrderedDict[str, Any],
    ) -> List[float]:
        """Extract the desired goal from the observation.

        The desired goal is the target position for the object.

        Args:
            observation (OrderedDict[str, Any]): The observation after the action is executed

        Returns:
            List[float]: The desired goal
        """
        return list(observation["target_pos"])

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        # Set the desired new initial joint angles before resetting the robot.
        self.robots[0].init_qpos = np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, np.pi / 4])

        super()._reset_internal()
        self._target_positions = [self._sample_target_pos() for _ in range(self._n_targets_to_sample_at_resets)]
        self._target_positions_index = 0
        self._object_placements_list = [
            [
                (obj.joints[0], np.concatenate([obj_pos, obj_quat]))
                for obj_pos, obj_quat, obj in self.object_placement_initializer.sample().values()
            ] for _ in range(self._n_objects_to_sample_at_resets)
        ]
        self._object_placements_list_index = 0

    def _sample_target_pos(self) -> np.ndarray:
        """Sample a new target location from the defined space.

        Returns:b
            np.ndarray: a new target location
        """
        return np.array(self.target_placement_initializer.sample()['target'][0])

    def _setup_arena(self):
        """Set up the mujoco arena.

        Must define self.mujoco_arena.
        Defines object, target, and obstacles.
        """
        # load model for table top workspace
        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_offset=self.table_offset,
            xml=xml_path_completion("arenas/table_arena.xml")
        )

        # Arena always gets set to zero origin
        self._set_origin()

        # Modify default agentview camera
        self._set_mujoco_camera()

        # << OBJECTS >>
        # Objects are elements that can be moved around and manipulated.
        # Create objects
        # Box example
        box_size = np.array(self.object_full_size)
        self.manipulation_object = BoxObject(
            name="manipulation_object",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )
        self.objects = [self.manipulation_object]
        # Placement sampler for objects
        object_bin_boundaries = self._get_default_object_bin_boundaries()

        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=self.objects,
            x_range=[object_bin_boundaries[0], object_bin_boundaries[1]],
            y_range=[object_bin_boundaries[2], object_bin_boundaries[3]],
        )

        # << TARGETS >>
        # Targets specify the coordinates to which the object should be moved.
        target = BoxObject(
            name="target",
            size=box_size * 0.5,
            rgba=[0.9, 0.1, 0.1, 1],
        )

        target_bin_boundaries = self._get_default_target_bin_boundaries()
        self.target_placement_initializer = self._setup_placement_initializer(
            name="TargetSampler",
            initializer=self.target_placement_initializer,
            objects=[target],
            x_range=[target_bin_boundaries[0], target_bin_boundaries[1]],
            y_range=[target_bin_boundaries[2], target_bin_boundaries[3]],
            z_offset=box_size[2] * 0.5,
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

    def _setup_collision_info(self):
        """Extend the super method by white-listing the manipulation object for collision detection."""
        super()._setup_collision_info()
        self.whitelisted_collision_geoms = self.whitelisted_collision_geoms.union(
            {
                self.sim.model.geom_name2id(geom_name) for geom_name in self.manipulation_object.contact_geoms
            }
        )

    def _setup_references(self):
        """Set up references to important components."""
        super()._setup_references()

        assert len(self.objects) == 1
        self.manipulation_object_body_id = self.sim.model.body_name2id(self.objects[0].root_body)

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

        # Absolute coordinates of goal position
        @sensor(modality=goal_mod)
        def target_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return self.target_pos

        # Absolute coordinates of object position
        @sensor(modality=obj_mod)
        def object_pos(obs_cache: Dict[str, Any]) -> np.ndarray:
            return np.array(self.sim.data.body_xpos[self.manipulation_object_body_id])

        # Vector from robot end-effector to object
        @sensor(modality=obj_mod)
        def vec_eef_to_object(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["object_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "object_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Vector from object to target
        @sensor(modality=goal_mod)
        def vec_object_to_target(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["target_pos"]) - np.array(obs_cache["object_pos"])
                if "target_pos" in obs_cache and "object_pos" in obs_cache
                else np.zeros(3)
            )

        @sensor(modality=goal_mod)
        def vec_eef_to_target(obs_cache: Dict[str, Any]) -> np.ndarray:
            return (
                np.array(obs_cache["target_pos"]) - np.array(obs_cache[f"{pf}eef_pos"])
                if "target_pos" in obs_cache and f"{pf}eef_pos" in obs_cache
                else np.zeros(3)
            )

        # Boolean value if the object is gripped
        # Checks if both finger pads are in contact with the object
        @sensor(modality="object")
        def object_gripped(obs_cache: Dict[str, Any]) -> bool:
            return self._check_grasp(
                gripper=self.robots[0].gripper,
                object_geoms=self.manipulation_object,
            )

        @sensor(modality=goal_mod)
        def vec_eef_to_next_objective(obs_cache: Dict[str, Any]) -> np.ndarray:
            if all([key in obs_cache for key in ["vec_eef_to_object", "vec_eef_to_target", "object_gripped"]]):
                return np.array(
                    obs_cache["vec_eef_to_target"]
                ) if obs_cache["object_gripped"] else np.array(
                    obs_cache["vec_eef_to_object"]
                )
            else:
                return np.zeros(3)

        sensors = [
            target_pos,
            object_pos,
            vec_eef_to_object,
            vec_object_to_target,
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
            bin_x_half * 0.6,
            bin_y_half * 0.25,
            bin_y_half * 0.45,
        )

    def _get_default_target_bin_boundaries(self) -> Tuple[float, float, float, float]:
        """Get the x and y boundaries of the target sampling space.

        Returns:
            Tuple[float, float, float, float]:
                Boundaries of the sampling space in the form (xmin, xmax, ymin, ymax)
        """
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05

        return (
            bin_x_half * 0.35,
            bin_x_half * 0.6,
            bin_y_half * -0.45,
            bin_y_half * -0.25,
        )

    def _visualize(self):
        """Visualize goal, object sampling space, and target sampling space."""
        self._visualize_goal()
        self._visualize_object_sample_space()
        self._visualize_target_sample_space()

    def _visualize_goal(self):
        """Draw a sphere at the target location."""
        # sphere (type 2)
        self.viewer.viewer.add_marker(
            pos=self.target_pos,
            type=2,
            size=[self.goal_dist, self.goal_dist, self.goal_dist],
            rgba=[0.0, 1.0, 0.0, 0.7],
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

    def _visualize_target_sample_space(self):
        """Draw a box to display the target sampling space"""
        self.draw_box(
            self._get_default_target_bin_boundaries() + (  # Add z boundaries
                self.target_pos[2] - 0.05,
                self.target_pos[2] + 0.05
            ),
            (0.0, 0.0, 1.0, 0.3),
        )

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

    def get_environment_state(self) -> PickPlaceHumanCartEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            state (PickPlaceHumanCartEnvState): The current state of the environment.
        """
        human_env_state = super().get_environment_state()
        return PickPlaceHumanCartEnvState(
            object_placements_list=self._object_placements_list,
            object_placements_list_index=self._object_placements_list_index,
            target_positions=self._target_positions,
            target_positions_index=self._target_positions_index,
            **asdict(human_env_state),
        )

    def set_environment_state(self, state: PickPlaceHumanCartEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            PickPlaceHumanCartEnvState: The state to be set.
        """
        super().set_environment_state(state)
        self._object_placements_list = state.object_placements_list
        self._object_placements_list_index = state.object_placements_list_index
        self._target_positions = state.target_positions
        self._target_positions_index = state.target_positions_index

        if self.has_renderer:
            self._visualize()
