"""This file describes a reach task for a single robot with a human doing tasks nearby.

This class is based on the human environment.

Owner:
    Jakob Thumm (JT)

Contributors:
    Julian Balletshofer JB
Changelog:
    2.5.22 JT Formatted docstrings
    15.7.22 JB added optional stop at collision
"""
from human_robot_gym.environments.manipulation.human_env import HumanEnv

from typing import Dict, Union, List

import numpy as np

from robosuite.models.arenas import TableArena
from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler

from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)
import human_robot_gym.models.objects.obstacle


class ReachHuman(HumanEnv):
    """
    This class corresponds to the reaching task for a single robot arm in a human environment.

    Args:
        robots (str or list of str): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: "Sawyer" would generate one arm; ["Panda", "Panda", "Sawyer"] would generate three robot arms)
            Note: Must be a single single-arm robot!

        env_configuration (str): Specifies how to position the robots within the environment (default is "default").
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (str or list of dict): If set, contains relevant controller parameters for creating a
            custom controller. Else, uses the default controller for this specific task. Should either be single
            dict if same controller is to be used for all robots or else it should be a list of the same length as
            "robots" param

        gripper_types (str or list of str): type of gripper, used to instantiate
            gripper models from gripper factory. Default is "default", which is the default grippers(s) associated
            with the robot(s) the 'robots' specification. None removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single str if same gripper type is to be used for all
            robots or else it should be a list of the same length as "robots" param

        initialization_noise (dict or list of dict): Dict containing the initialization noise parameters.
            The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or 0.0 results in no noise being applied.
                If "gaussian" type of noise is applied then this magnitude scales the standard deviation applied,
                If "uniform" type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify "gaussian" or "uniform"

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as "robots" param

            :Note: Specifying "default" will automatically use the default noise settings.
                Specifying None will automatically create the required dict with "magnitude" set to 0.0.

        table_full_size (3-tuple): x, y, and z dimensions of the table.

        table_friction (3-tuple): the three mujoco friction parameters for
            the table.

        use_camera_obs (bool): if True, every observation includes rendered image(s)

        use_object_obs (bool): if True, include object information in the observation.

        reward_scale (None or float): Scales the normalized reward function by the amount specified.
            If None, environment reward remains unnormalized

        reward_shaping (bool): if True, use dense rewards, else use sparse rewards.

        goal_dist (double): Distance threshold for reaching the goal.

        collision_reward (double): Reward to be given in the case of a collision.

        object_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a UniformRandomSampler
            is used by default.
            Objects are elements that can and should be manipulated.

        obstacle_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place obstacles on every reset, else a UniformRandomSampler
            is used by default.
            Obstacles are elements that should be avoided.

        human_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place the human on every reset, else a UniformRandomSampler
            is used by default.

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

        camera_names (str or list of str): name of camera to be rendered. Should either be single str if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if @use_camera_obs is True.

            :Note: To render all robots' cameras of a certain type (e.g.: "robotview" or "eye_in_hand"), use the
                convention "all-{name}" (e.g.: "all-robotview") to automatically render all camera images from each
                robot's camera list).

        camera_heights (int or list of int): height of camera frame. Should either be single int if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_widths (int or list of int): width of camera frame. Should either be single int if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_depths (bool or list of bool): True if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            "camera names" param.

        camera_segmentations (None or str or list of str or list of list of str): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not None, multiple types of segmentations can be specified. A [list of str / str or None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A list of list of str specifies per-camera
            segmentation setting(s) to use.

        use_failsafe_controller (bool): Whether or not the safety shield / failsafe controller should be active

        visualize_failsafe_controller (bool): Whether or not the reachable sets of the failsafe controller should be
            visualized

        visualize_pinocchio (bool): Whether or pinocchios (collision prevention static env) should be visualized

        control_sample_time (double): Control frequency of the failsafe controller

        human_animation_names (list[str]): Human animations to play

        base_human_pos_offset (list[double]): Base human animation offset

        human_animation_freq (double): Speed of the human animation in fps.

        safe_vel (double): Safe cartesian velocity. The robot is allowed to move with this velocity in the vacinity of
            humans.

        randomize_initial_pos (bool): True - Use random initial joint position for robot.
                                      False - Do not override initial position.

        self_collision_safety (double): Safe distance for self collision detection

        seed (int): Random seed for np.random

    Raises:
        AssertionError: [Invalid number of robots specified]
    """

    def __init__(
        self,
        robots,
        robot_base_offset=None,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise="default",
        table_full_size=(2.5, 0.85, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=False,
        goal_dist=0.01,
        collision_reward=-10,
        object_placement_initializer=None,
        obstacle_placement_initializer=None,
        human_placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=10,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,  # {None, instance, class, element}
        renderer="mujoco",
        renderer_config=None,
        use_failsafe_controller=True,
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        control_sample_time=0.004,
        human_animation_names=[
            "62_01",
            "62_03",
            "62_04",
            "62_07",
            "62_09",
            "62_10",
            "62_12",
            "62_13",
            "62_14",
            "62_15",
            "62_16",
            "62_18",
            "62_19",
            "62_20",
            "62_21",
        ],
        base_human_pos_offset=[0.0, 0.0, 0.0],
        human_animation_freq=120,
        safe_vel=0.001,
        randomize_initial_pos=False,
        self_collision_safety=0.01,
        seed=0,
        done_at_collision=False
    ):  # noqa: D107
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        # settings for table top (hardcoded since it's not an essential part of the environment)
        self.table_offset = np.array((0.0, -0.48, 0.82))
        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping
        self.collision_reward = collision_reward
        self.goal_dist = goal_dist
        self.desired_goal = np.array([0.0])
        # object placement initializer
        self.object_placement_initializer = object_placement_initializer
        self.obstacle_placement_initializer = obstacle_placement_initializer
        # Robot Base offset
        if robot_base_offset is None:
            robot_base_offset = [[0.0, 0.0, 0.0] for robot in robots]
        self.randomize_initial_pos = randomize_initial_pos
        # if run should stop at collision
        self.done_at_collision = done_at_collision
        super().__init__(
            robots=robots,
            robot_base_offset=robot_base_offset,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            use_object_obs=use_object_obs,
            human_placement_initializer=human_placement_initializer,
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
            safe_vel=safe_vel,
            self_collision_safety=self_collision_safety,
            seed=seed,
        )

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

    def reward(
        self, achieved_goal: List[float], desired_goal: List[float], info: Dict
    ) -> float:
        """Compute the reward based on the achieved goal, the desired goal, and the info dict.

        If self.reward_shaping, we use a dense reward, otherwise a sparse reward.
        This function can only be called for one sample.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision
        Returns:
            reward
        """
        reward = -1.0
        if info["collision"]:
            return self.collision_reward
        # sparse completion reward
        if self._check_success(achieved_goal, desired_goal):
            reward = 0.0
        # use a shaping reward
        if self.reward_shaping:
            dist = np.sqrt(np.sum((achieved_goal - desired_goal)**2))
            reward -= dist * 0.1
        # Scale reward if requested
        if self.reward_scale is not None:
            reward *= self.reward_scale / 1.0
        return reward

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

    def _check_done(
        self, achieved_goal: List[float], desired_goal: List[float], info: Dict
    ) -> bool:
        """Compute the done flag based on the achieved goal, the desired goal, and the info dict.

        This function can only be called for one sample.
        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision
        Returns:
            done
        """
        done = super()._check_done(achieved_goal, desired_goal, info)
        if self.done_at_collision:
            return done or (self._check_success(achieved_goal, desired_goal))
        return self._check_success(achieved_goal, desired_goal)

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
        self.desired_goal = self._sample_valid_pos()
        if isinstance(self.robots[0].robot_model, PinocchioManipulatorModel):
            (self.goal_marker_trans, self.goal_marker_rot) = self.robots[
                0
            ].robot_model.get_eef_transformation(self.desired_goal)

    def _sample_valid_pos(self):
        """Randomly sample a new valid joint configuration without self-collisions or collisions with the static environment.

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
        )

        # Arena always gets set to zero origin
        self.mujoco_arena.set_origin([0, 0, 0])

        # Modify default agentview camera
        self.mujoco_arena.set_camera(
            camera_name="agentview",
            pos=[0.0, -1.5, 1.5],
            quat=[-0.0705929, 0.0705929, 0.7035742, 0.7035742],
        )

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
        if self.object_placement_initializer is not None:
            self.object_placement_initializer.reset()
            self.object_placement_initializer.add_objects(self.objects)
        else:
            self.object_placement_initializer = UniformRandomSampler(
                name="ObjectSampler",
                mujoco_objects=self.objects,
                x_range=[-bin_x_half, bin_x_half],
                y_range=[-bin_y_half, bin_y_half],
                rotation=(0, 0),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=False,
                reference_pos=self.table_offset,
                z_offset=0.0,
            )
        # << OBSTACLES >>
        # Obstacles are elements that the robot should avoid.
        safety_margin = 0.05
        # Box example
        # l = np.array([0.4, 0.4, 0.4])
        # box = BoxObject(
        #     name = "Table",
        #     size = l*0.5,
        #     rgba = [0.5, 0.5, 0.5, 1],
        # )
        self.obstacles = []
        # Obstacles should also have a collision object
        coll_table = human_robot_gym.models.objects.obstacle.Box(
            name="Table",
            x=self.table_full_size[0] + safety_margin,
            y=self.table_full_size[1] + safety_margin,
            z=self.table_offset[2] + safety_margin,
            translation=np.array(
                [
                    self.table_offset[0],
                    self.table_offset[1],
                    (self.table_offset[2] + safety_margin) / 2,
                ]
            ),
        )
        coll_base = human_robot_gym.models.objects.obstacle.Cylinder(
            name="Base",
            r=0.25 + safety_margin,
            z=0.91,
            translation=np.array([-0.46, 0, 0.455]),
        )
        coll_computer = human_robot_gym.models.objects.obstacle.Box(
            name="Computer", x=0.3, y=0.5, z=0.8, translation=np.array([-0.85, 0, 0.35])
        )
        self.collision_obstacles = [coll_table, coll_base, coll_computer]
        # Matches sim joint names to the collision obstacles
        # self.collision_obstacles_joints["Box"] = (box.joints[0], coll_box)
        # Placement sampler for obstacles
        if self.obstacle_placement_initializer is not None:
            self.obstacle_placement_initializer.reset()
            self.obstacle_placement_initializer.add_objects(self.obstacles)
        else:
            self.obstacle_placement_initializer = UniformRandomSampler(
                name="ObstacleSampler",
                mujoco_objects=self.obstacles,
                x_range=[0.0, 0.0],
                y_range=[-0.0, 0.0],
                rotation=(0, 0),
                rotation_axis="x",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.1,
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

        # low-level object information
        modality = "goal"

        @sensor(modality=modality)
        def desired_goal(obs_cache):
            return self.desired_goal

        sensors = [desired_goal]
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
        self.viewer.viewer.add_marker(
            pos=self.goal_marker_trans,
            type=100,
            size=[0.01, 0.01, 0.2],
            mat=self.goal_marker_rot,
            rgba=[0.0, 1.0, 0.0, 0.7],
            label="",
            shininess=0.0,
        )
