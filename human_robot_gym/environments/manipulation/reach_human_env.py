"""This file describes a reach task for a single robot with a human doing tasks nearby.

This class is based on the human environment.

Owner:
    Jakob Thumm (JT)

Contributors:
    Julian Balletshofer JB
    Felix Trost FT
Changelog:
    2.5.22 JT Formatted docstrings
    15.7.22 JB added optional stop at collision
    16.05.23 FT Formatted docstrings
"""
from typing import Any, Dict, Union, List, Optional, Tuple
from dataclasses import asdict, dataclass

import numpy as np

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import ObjectPositionSampler

from human_robot_gym.utils.mjcf_utils import xml_path_completion
from human_robot_gym.environments.manipulation.human_env import HumanEnv, HumanEnvState
from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)


@dataclass
class ReachHumanEnvState(HumanEnvState):
    """Dataclass for encapsulating the state of the ReachHuman environment.
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
        desired_goals (List[np.ndarray]): List of desired goals. During the episode this list is
            iterated over and the corresponding target_position is used.
        desired_goals_index (int): Index of the current desired goal in the list of target_position.
    """
    desired_goals: List[np.ndarray]
    desired_goals_index: int


class ReachHuman(HumanEnv):
    """
    This class corresponds to the reaching task for a single robot arm in a human environment.

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

        randomize_initial_pos (bool): If `True`: Use random initial joint position for robot.
            Otherwise, do not override initial position.

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
        randomize_initial_pos=False,
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
        # settings for table top (hardcoded since it's not an essential part of the environment)
        self.table_offset = np.array((0.0, 0.0, 0.82))
        # reward configuration
        self.goal_dist = goal_dist
        self._desired_goals = None
        self._desired_goals_index = 0
        self._n_goals_to_sample_at_resets = max(
            int(horizon * n_goals_sampled_per_100_steps / 100),
            1,
        )

        self.goal_marker_trans = None
        self.goal_marker_rot = None
        # object placement initializer
        self.object_placement_initializer = object_placement_initializer
        self.obstacle_placement_initializer = obstacle_placement_initializer
        self.randomize_initial_pos = randomize_initial_pos

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
    def desired_goal(self):
        """Return the current desired goal in the list of sampled goals."""
        return self._desired_goals[self._desired_goals_index]

    def step(self, action):
        """Override base step function.

        Adds the goal position as an arrow to the visualizer.

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
        if self.goal_reached:
            # if goal is reached, calculate a new goal.
            self._desired_goals_index = (self._desired_goals_index + 1) % self._n_goals_to_sample_at_resets
            if isinstance(self.robots[0].robot_model, PinocchioManipulatorModel):
                (self.goal_marker_trans, self.goal_marker_rot) = self.robots[
                    0
                ].robot_model.get_eef_transformation(self._desired_goals[self._desired_goals_index])
            self.goal_reached = False
        if self.has_renderer:
            self._visualize_goal()
        return obs, reward, done, info

    def reset(self):
        """
        Resets the environment.

        Returns:
            Observation
        """
        return super().reset()

    def _get_info(self) -> Dict:
        """Return the info dictionary of this step.

        Returns
            info dict containing of
                * collision: if there was a collision or not
                * collision_type: type of collision
                * timeout: if timeout was reached
                * failsafe_intervention: if the failsafe controller intervened
                    in this step or not
        """
        info = super()._get_info()
        # Add more info if wanted (do not forget to pass this to the tensorboard callback)
        # info["my_cool_info"] = 0
        return info

    def _dense_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a dense guidance reward based on the achieved goal, the desired goal, and the info dict.

        The dense reward is proportional to the L2-distance between the achieved goal and the desired goal.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: dense environment reward
        """
        return -0.1 * np.sqrt(np.sum((np.array(achieved_goal) - np.array(desired_goal))**2))

    def _check_success(
        self, achieved_goal: List[float], desired_goal: List[float]
    ) -> bool:
        """Check if the desired goal was reached.

        Checks if all robot joints are at the desired position.
        The distance metric is a RMSE and the threshold is self.goal_dist.
        This function can only be called for one sample.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
        Returns:
            True if success
        """
        dist = np.sqrt(
            np.sum([(a - g) ** 2 for (a, g) in zip(achieved_goal, desired_goal)])
        )
        return dist <= self.goal_dist

    def _get_achieved_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """
        Extract the achieved goal from the observation.

        The achieved goal is the new joint angle position of all joints.

        Args:
            observation: The observation after the action is executed

        Returns:
            The achieved goal
        """
        prefix = self.robots[0].robot_model.naming_prefix
        return observation[prefix + "joint_pos"]

    def _get_desired_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """Extract the desired goal from the observation.

        The desired goal is a desired goal joint position.

        Args:
            observation: The observation after the action is executed

        Returns:
            The desired goal
        """
        return observation["desired_goal"]

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        # Set the desired new initial joint angles before resetting the robot.
        if self.randomize_initial_pos:
            if self.robots[0].controller is not None:
                self.robots[0].init_qpos = self._sample_valid_pos()
        super()._reset_internal()

        self._desired_goals = [self._sample_valid_pos() for _ in range(self._n_goals_to_sample_at_resets)]
        self._desired_goals_index = 0

        if isinstance(self.robots[0].robot_model, PinocchioManipulatorModel):
            (self.goal_marker_trans, self.goal_marker_rot) = self.robots[
                0
            ].robot_model.get_eef_transformation(self._desired_goals[self._desired_goals_index])

    def _sample_valid_pos(self):
        """Randomly sample a new valid joint configuration
            without self-collisions or collisions with the static environment.

        Returns:
            joint configuration (np.array)
        """
        robot = self.robots[0]
        pos_limits = np.array(robot.controller.position_limits)
        goal = np.zeros(pos_limits.shape[1])
        for i in range(20):
            rand = np.random.rand(pos_limits.shape[1])
            goal = pos_limits[0] + (pos_limits[1] - pos_limits[0]) * rand
            if isinstance(robot.robot_model, PinocchioManipulatorModel):
                if not self._check_action_safety(robot.robot_model, goal):
                    goal = np.zeros(pos_limits.shape[1])
                    if self.visualize_pinocchio:
                        self.visualize_pin(self.pin_viz)
                else:
                    break
            else:
                break

        return goal

    def _setup_arena(self):
        """Set up the mujoco arena.

        Must define self.mujoco_arena.
        Define self.objects and self.obstacles here.
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
        box_size = np.array([0.05, 0.05, 0.05])
        box = BoxObject(
            name="smallBox",
            size=box_size * 0.5,
            rgba=[0.1, 0.7, 0.3, 1],
        )
        self.objects = [box]
        # Placement sampler for objects
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=self.objects,
            x_range=[-bin_x_half, bin_x_half],
            y_range=[-bin_y_half, bin_y_half],
        )
        # << OBSTACLES >>
        self._setup_collision_objects(
            add_table=True,
            add_base=True,
            safety_margin=0.01
        )
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def _setup_references(self):
        """Set up references to important components."""
        super()._setup_references()

    def _setup_observables(self):
        """Set up observables to be used for this environment.

        Creates object-based observables if enabled.

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()
        # robot joint pos
        prefix = self.robots[0].robot_model.naming_prefix
        if prefix + "joint_pos" in observables:
            observables[prefix + "joint_pos"].set_active(True)
        if prefix + "joint_vel" in observables:
            observables[prefix + "joint_vel"].set_active(True)
        if prefix + "eef_pos" in observables:
            observables[prefix + "eef_pos"].set_active(True)
        if "human_joint_pos" in observables:
            observables["human_joint_pos"].set_active(True)
        if prefix + "joint_pos_cos" in observables:
            observables[prefix + "joint_pos_cos"].set_active(False)
        if prefix + "joint_pos_sin" in observables:
            observables[prefix + "joint_pos_sin"].set_active(False)
        if prefix + "gripper_qpos" in observables:
            observables[prefix + "gripper_qpos"].set_active(False)
        if prefix + "gripper_qvel" in observables:
            observables[prefix + "gripper_qvel"].set_active(False)
        if prefix + "eef_quat" in observables:
            observables[prefix + "eef_quat"].set_active(False)
        if "gripper_pos" in observables:
            observables["gripper_pos"].set_active(False)
        if "gripper_aperture" in observables:
            observables["gripper_aperture"].set_active(False)

        # low-level object information
        modality = "goal"

        @sensor(modality=modality)
        def desired_goal(obs_cache):
            return self.desired_goal

        @sensor(modality=modality)
        def goal_difference(obs_cache):
            return self.desired_goal - np.array([self.sim.data.qpos[x] for x in self.robots[0]._ref_joint_pos_indexes])

        sensors = [desired_goal]
        if len(self.robots[0]._ref_joint_pos_indexes) == self.desired_goal.shape[0]:
            sensors.append(goal_difference)

        names = [s.__name__ for s in sensors]

        # Create observables
        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            )
        return observables

    def _visualize_goal(self):
        """Visualize the goal state."""
        # arrow (type 100)
        return  # TODO goal_marker_trans is not set if robot does not inherit from pinocchio manipulator model
        self.viewer.viewer.add_marker(
            pos=self.goal_marker_trans,
            type=100,
            size=[0.01, 0.01, 0.2],
            mat=self.goal_marker_rot,
            rgba=[0.0, 1.0, 0.0, 0.7],
            label="",
            shininess=0.0,
        )

    def get_environment_state(self) -> ReachHumanEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            state (ReachHumanEnvState): The current state of the environment.
        """
        human_env_state = super().get_environment_state()
        return ReachHumanEnvState(
            desired_goals=self._desired_goals,
            desired_goals_index=self._desired_goals_index,
            **asdict(human_env_state),
        )

    def set_environment_state(self, state: ReachHumanEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            ReachHumanEnvState: The state to be set.
        """
        super().set_environment_state(state)
        self._desired_goals = state.desired_goals
        self._desired_goals_index = state.desired_goals_index

        if isinstance(self.robots[0].robot_model, PinocchioManipulatorModel):
            (self.goal_marker_trans, self.goal_marker_rot) = self.robots[
                0
            ].robot_model.get_eef_transformation(self._desired_goals[self._desired_goals_index])

        if self.has_renderer:
            self._visualize_goal()
