"""This file describes a variant for the pick place task
where the human points to a spot on the table where the object should be placed.

Author
    Felix Trost (FT)

Changelog:
    06.02.23 FT File creation
"""
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.placement_samplers import ObjectPositionSampler
from robosuite.utils.observables import Observable, sensor

from human_robot_gym.environments.manipulation.pick_place_human_cartesian_env import PickPlaceHumanCart
from human_robot_gym.utils.mjcf_utils import xml_path_completion


class PickPlacePointingHumanCart(PickPlaceHumanCart):
    """This class corresponds to the pick place task for a single robot arm in a human environment
    where the robot should place the object to a spot on the table the human is pointing at.

    Args:
        robots (str | List[str]): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: "Sawyer" would generate one arm; ["Panda", "Panda", "Sawyer"] would generate three robot arms)
            Note: Must be a single single-arm robot!

        robot_base_offset (None | List[float] or List[List[float]]): Offset (x, y, z) of the robot bases.
            If more than one robot is loaded provide a list of doubles, one for each robot.
            Specify None for an offset of (0, 0, 0) for each robot.

        env_configuration (str): Specifies how to position the robots within the environment (default is "default").
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (None | str | List[Dict[str, Any]]): If set, contains relevant controller parameters
            for creating a custom controller. Else, uses the default controller for this specific task.
            Should either be single dict if same controller is to be used for all robots or else it should be
            a list of the same length as "robots" param

        gripper_types (str | List[str]): type of gripper, used to instantiate
            gripper models from gripper factory. Default is "default", which is the default grippers(s) associated
            with the robot(s) the 'robots' specification. None removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single str if same gripper type is to be used for all
            robots or else it should be a list of the same length as "robots" param

        initialization_noise (Dict[str, Any] | List[Dict[str, Any]]): Dict containing the initialization noise
            parameters. The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or 0.0 results in no noise being applied.
                If "gaussian" type of noise is applied then this magnitude scales the standard deviation applied,
                If "uniform" type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify "gaussian" or "uniform"

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as "robots" param

            :Note: Specifying "default" will automatically use the default noise settings.
                Specifying None will automatically create the required dict with "magnitude" set to 0.0.

        table_full_size (Tuple[float, float, float]): x, y, and z dimensions of the table.

        table_friction (Tuple[float, float, float]): the three mujoco friction parameters for
            the table.

        use_camera_obs (bool): if True, every observation includes rendered image(s)

        use_object_obs (bool): if True, include object information in the observation.

        reward_scale (None | float): Scales the normalized reward function by the amount specified.
            If None, environment reward remains unnormalized

        reward_shaping (bool): if True, use dense rewards, else use sparse rewards.

        goal_dist (float): Distance threshold for reaching the goal.

        collision_reward (float): Reward to be given in the case of a collision.

        goal_reward (float): Reward to be given in the case of reaching the goal.

        object_gripped_reward (float): Additional reward for gripping the object when `reward_shaping=False`.
            If object is not gripped: `reward = -1`.
            If object gripped but not at the target: `object_gripped_reward`.
            If object is at the target: `reward = goal_reward`.
            `object_gripped_reward` defaults to -1.

        object_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a UniformRandomSampler
            is used by default.
            Objects are elements that can and should be manipulated.

        obstacle_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place obstacles on every reset, else a UniformRandomSampler
            is used by default.
            Obstacles are elements that should be avoided.

        has_renderer (bool): If true, render the simulation state in
            a viewer instead of headless mode.

        has_offscreen_renderer (bool): True if using off-screen rendering

        render_camera (str): Name of camera to render if `has_renderer` is True. Setting this value to 'None'
            will result in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse

        render_collision_mesh (bool): True if rendering collision meshes in camera. False otherwise.

        render_visual_mesh (bool): True if rendering visual meshes in camera. False otherwise.

        render_gpu_device_id (int): corresponds to the GPU device id to use for offscreen rendering.
            Defaults to -1, in which case the device will be inferred from environment variables
            (GPUS or CUDA_VISIBLE_DEVICES).

        control_freq (float): how many control signals to receive in every second. This sets the amount of
            simulation time that passes between every action input.

        horizon (int): Every episode lasts for exactly @horizon action steps.

        ignore_done (bool): True if never terminating the environment (ignore @horizon).

        hard_reset (bool): If True, re-loads model, sim, and render object upon a reset call, else,
            only calls self.sim.reset and resets all robosuite-internal variables

        camera_names (str | List[str]): name of camera to be rendered. Should either be single str if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if @use_camera_obs is True.

            :Note: To render all robots' cameras of a certain type (e.g.: "robotview" or "eye_in_hand"), use the
                convention "all-{name}" (e.g.: "all-robotview") to automatically render all camera images from each
                robot's camera list).

        camera_heights (int | List[int]): height of camera frame. Should either be single int if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_widths (int | List[int]): width of camera frame. Should either be single int if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_depths (bool | List[bool]): True if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            "camera names" param.

        camera_segmentations (None | str | List[str] | List[List[str]]): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not None, multiple types of segmentations can be specified. A [list of str / str or None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A list of list of str specifies per-camera
            segmentation setting(s) to use.

        renderer (str): string for the renderer to use

        renderer_config (Dict[str, Any]): dictionary for the renderer configurations

        use_failsafe_controller (bool): Whether or not the safety shield / failsafe controller should be active

        visualize_failsafe_controller (bool): Whether or not the reachable sets of the failsafe controller should be
            visualized

        visualize_pinocchio (bool): Whether or pinocchios (collision prevention static env) should be visualized

        control_sample_time (float): Control frequency of the failsafe controller

        human_animation_names (List[str]): Human animations to play

        base_human_pos_offset (List[float]): Base human animation offset

        human_animation_freq (float): Speed of the human animation in fps.

        human_rand (List[float]): Max. randomization of the human [x-pos, y-pos, z-angle]

        safe_vel (float): Safe cartesian velocity. The robot is allowed to move with this velocity in the vacinity of
            humans.

        self_collision_safety (float): Safe distance for self collision detection

        seed (int): Random seed for np.random

        verbose (bool): If True, print out debug information

        done_at_collision (bool): If True, the episode is terminated when a collision occurs

        done_at_success (bool): If True, the episode is terminated when the goal is reached

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
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        reward_shaping: bool = False,
        goal_dist: float = 0.1,
        collision_reward: float = -10,
        goal_reward: float = 1,
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
        use_failsafe_controller: bool = True,
        visualize_failsafe_controller: bool = False,
        visualize_pinocchio: bool = False,
        control_sample_time: float = 0.004,
        human_animation_names: List[str] = [
            "PickPlacePointingHuman/0",
            "PickPlacePointingHuman/1",
        ],
        base_human_pos_offset: List[float] = [0.0, 0.0, 0.0],
        human_animation_freq: float = 30,
        human_rand: List[float] = [0.0, 0.0, 0.0],
        safe_vel: float = 0.001,
        self_collision_safety: float = 0.01,
        seed: int = 0,
        verbose: bool = False,
        done_at_collision: bool = False,
        done_at_success: bool = False,
    ):
        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            table_full_size=table_full_size,
            table_friction=table_friction,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            reward_scale=reward_scale,
            reward_shaping=reward_shaping,
            goal_dist=goal_dist,
            collision_reward=collision_reward,
            goal_reward=goal_reward,
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
            use_failsafe_controller=use_failsafe_controller,
            visualize_failsafe_controller=visualize_failsafe_controller,
            visualize_pinocchio=visualize_pinocchio,
            control_sample_time=control_sample_time,
            human_animation_names=human_animation_names,
            base_human_pos_offset=base_human_pos_offset,
            human_animation_freq=human_animation_freq,
            human_rand=human_rand,
            safe_vel=safe_vel,
            self_collision_safety=self_collision_safety,
            seed=seed,
            verbose=verbose,
            done_at_collision=done_at_collision,
            done_at_success=done_at_success,
        )

    def _on_goal_reached(self):
        object_placements = self.object_placement_initializer.sample()
        for obj_pos, obj_quat, obj in object_placements.values():
            self.sim.data.set_joint_qpos(
                obj.joints[0],
                np.concatenate([obj_pos, obj_quat])
            )

    def _visualize(self):
        self._visualize_goal()
        self._visualize_object_sample_space()

    def _setup_arena(self):
        self.mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_offset=self.table_offset,
            xml=xml_path_completion("arenas/table_arena.xml"),
        )

        self._set_origin()

        self._set_mujoco_camera()

        box_size = np.array([0.04, 0.04, 0.04])
        box = BoxObject(
            name="smallBox",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )
        self.objects = [box]
        object_bin_boundaries = self._get_object_bin_boundaries()
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

    def _obtain_goal_pos(self) -> np.ndarray:
        if self.human_animation_data[self.human_animation_id][1]["pointing_hand"] == "right":
            desired_goal = self.sim.data.get_site_xpos(self.human.right_hand)
        elif self.human_animation_data[self.human_animation_id][1]["pointing_hand"] == "left":
            desired_goal = self.sim.data.get_site_xpos(self.human.left_hand)

        head_pos = self.sim.data.get_site_xpos(self.human.head)
        xz_offset = desired_goal - head_pos
        xz_offset = 0.15 * xz_offset / np.linalg.norm(xz_offset)

        thumb_correction = np.cross(-np.abs(xz_offset), np.array([0, 0, 1]))
        thumb_correction = 0.1 * thumb_correction / np.linalg.norm(thumb_correction)
        xz_offset += thumb_correction
        desired_goal += xz_offset
        desired_goal[2] = self.table_offset[2]
        return desired_goal

    def _setup_observables(self):
        observables = super()._setup_observables()

        # Absolute coordinates of goal position
        @sensor(modality="goal")
        def target_pos(obs_cache) -> np.ndarray:
            self.desired_goal = self._obtain_goal_pos()

            return self.desired_goal

        sensors = [
            target_pos,
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
