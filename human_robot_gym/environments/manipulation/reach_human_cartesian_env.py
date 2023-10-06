"""This file describes a cartesian position only reach task for a single robot with a human doing tasks nearby.

This class is based on the reach human environment.

Owner:
    Rafael Cabral

Contributors:
    Felix Trost (FT)

Changelog:
    16.05.23: FT Formatted docstrings
"""
from typing import Any, Dict, Union, List, Optional, Tuple

import numpy as np

from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import ObjectPositionSampler

from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman
from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)


class ReachHumanCart(ReachHuman):
    """
    This class extends corresponds to the cartesian reaching task for a single robot arm in a human environment.

    The arguments differ from ReachHuman in the following two ways:
        Randomized initial joint positions are not allowed.
        Added init_joint_pos to arguments.

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

        table_full_size (3-tuple): x, y, and z dimensions of the table.

        table_friction (3-tuple): the three mujoco friction parameters for
            the table.

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (None or float): Scales the normalized reward function by the amount specified.
            If `None`, environment reward remains unnormalized

        reward_shaping (bool): if `True`, use dense rewards, else use sparse rewards.

        goal_dist (float): Distance threshold for reaching the goal.

        n_goals_sampled_per_100_steps (int): Length of the list of desired goals to sample at resets.
            After all goals of the list have been reached, restart from the first in the list.
            This is done to ensure the same list of goals can be played when loading the env state from a file.

        collision_reward (float): Reward to be given in the case of a collision.

        task_reward (float): Reward to be given in the case of reaching the goal.

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

        init_joint_pos (np.array): initial joint configuration of the robot

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
        n_goals_sampled_per_100_steps: int = 8,
        collision_reward: float = -10,
        task_reward: float = 1,
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
        init_joint_pos: np.ndarray = np.array([0, 0.0, -np.pi / 2, 0, -np.pi / 2, np.pi / 4]),
    ):  # noqa: D107
        self.init_joint_pos = init_joint_pos
        self.sampling_space = np.array([[0.1, -0.5, 0.8], [0.5, 0.5, 1.3]])
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
            n_goals_sampled_per_100_steps=n_goals_sampled_per_100_steps,
            collision_reward=collision_reward,
            task_reward=task_reward,
            object_placement_initializer=object_placement_initializer,
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
            camera_segmentations=camera_segmentations,  # {None, instance, class, element}
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
            done_at_success=done_at_success
        )

    @property
    def desired_goal(self):
        """Return the desired goal."""
        return self.goal_marker_trans

    def step(self, action):
        """Override base step function.

        Changes the goal position to the Cartesian end-effector position.

        Args:
            action (np.array): Action to execute within the environment
        Returns:
            4-tuple:
                - (OrderedDict) observations from the environment
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) misc information
        Raises:
            ValueError: [Steps past episode termination]
        """
        obs, reward, done, info = super().step(action)
        # We have to set this in every step since the goal can change.
        return obs, reward, done, info

    def _get_achieved_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """
        Extract the achieved goal from the observation.

        The achieved goal is the new position of the end-effector.

        Args:
            observation: The observation after the action is executed

        Returns:
            The achieved goal
        """
        prefix = self.robots[0].robot_model.naming_prefix
        return observation[prefix + "eef_pos"]

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        self.robots[0].init_qpos = self.init_joint_pos
        super()._reset_internal()

    def _setup_observables(self):
        """Set up observables to be used for this environment.

        Creates object-based observables if enabled.

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()

        prefix = self.robots[0].robot_model.naming_prefix
        if prefix + "joint_pos" in observables:
            observables[prefix + "joint_pos"].set_active(False)
        if prefix + "joint_vel" in observables:
            observables[prefix + "joint_vel"].set_active(False)
        if prefix + "eef_pos" in observables:
            observables[prefix + "eef_pos"].set_active(True)
        if prefix + "eef_velp" in observables:
            observables[prefix + "eef_velp"].set_active(True)

        _eef_velp = self.sim.data.site_xvelp[self.robots[0].eef_site_id]

        # define observables modality
        modality = f"{prefix}proprio"

        @sensor(modality=modality)
        def eef_velp(obs_cache):
            return _eef_velp

        eef_velp.__name__ = f"{prefix}eef_velp"

        # Override goal difference observable
        modality = "goal"

        @sensor(modality=modality)
        def goal_difference(obs_cache):
            return self.desired_goal - np.array(self.sim.data.site_xpos[self.robots[0].eef_site_id])

        sensors = [eef_velp, goal_difference]
        names = [s.__name__ for s in sensors]

        # Create observables
        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )

        return observables

    def _sample_valid_pos(self):
        """Randomly sample a new valid joint configuration
            without self-collisions or collisions with the static environment.

        The end effector position of the valid pos lies in a box in front of the robot.

        Returns:
            joint configuration (np.array)
        """
        robot = self.robots[0]
        pos_limits = np.array(robot.controller.position_limits)
        goal = self.init_joint_pos
        for i in range(20):
            rand = np.random.rand(pos_limits.shape[1])
            goal = pos_limits[0] + (pos_limits[1] - pos_limits[0]) * rand
            if isinstance(robot.robot_model, PinocchioManipulatorModel):
                if not self._check_action_safety(robot.robot_model, goal):
                    goal = self.init_joint_pos
                    if self.visualize_pinocchio:
                        self.visualize_pin(self.pin_viz)
                else:
                    eef_goal_pos, _ = robot.robot_model.get_eef_transformation(goal)
                    if np.all(eef_goal_pos >= self.sampling_space[0]) and\
                       np.all(eef_goal_pos <= self.sampling_space[1]):
                        break
                    else:
                        goal = self.init_joint_pos
            else:
                break
        return goal

    def _visualize_goal(self):
        """Draw a sphere at the target location."""
        # sphere (type 2)
        self.viewer.viewer.add_marker(
            pos=self.goal_marker_trans,
            type=2,
            size=[self.goal_dist, self.goal_dist, self.goal_dist],
            rgba=[0.0, 1.0, 0.0, 0.7],
            label="",
            shininess=0.0,
        )
