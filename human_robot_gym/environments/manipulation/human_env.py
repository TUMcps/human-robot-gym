"""This file describes an environment with a human inside.

Adds the possibility to use a failsafe controller on the robots.

Owner:
    Jakob Thumm (JT)

Contributors:
    Julian Balletshofer (JB)
    Felix Trost (FT)

Changelog:
    2.5.22 JT Formatted docstrings
    13.7.22 JB adjusted observation space (sensors) to relative distances eef and L_hand, R_hand, and Head
    05.09.23 FT new reward function interface
"""
from typing import Any, Dict, Union, List, Optional, Tuple
from dataclasses import dataclass
from enum import IntFlag
import math

import numpy as np
from scipy.spatial.transform import Rotation

import pinocchio as pin

from mujoco_py.builder import MujocoException

from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import TableArena
from robosuite.models.tasks import ManipulationTask
import robosuite.utils.macros as macros
from robosuite.robots import SingleArm, Bimanual

# from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler, ObjectPositionSampler
from robosuite.utils.transform_utils import quat2mat
from robosuite.utils.control_utils import set_goal_position
from robosuite.models.objects import PrimitiveObject

from human_robot_gym.models.objects.human.human import HumanObject
from human_robot_gym.utils.mjcf_utils import xml_path_completion, rot_to_quat, quat_to_rot
from human_robot_gym.utils.pairing import cantor_pairing
from human_robot_gym.utils.animation_utils import load_human_animation_data
from human_robot_gym.models.robots.manipulators.pinocchio_manipulator_model import (
    PinocchioManipulatorModel,
)
from human_robot_gym.controllers.failsafe_controller.failsafe_controller import (
    FailsafeController,
)
import human_robot_gym.models.objects.obstacle as obstacle


class COLLISION_TYPE(IntFlag):
    """Define the collision types.

    Their order corresponds to the severeness of the collision.
    `HUMAN` collisions may occur without explicit fault of the robot,
    e.g. if the robot remains still and the human moves into the robot.

    The value and fragility of other objects in the scene is not known.
    Accordingly, we mark `STATIC` collisions as more severe than `ROBOT` self-collisions.

    0 - No collision.
    1 - Collision with white-listed objects
    2 - Self-collision or robot-robot collision.
    3 - Collision with the static environment.
    4 - Collision with a human but robot has low speed.
    5 - Collision with a human and robot has high speed.
    """
    NULL = 0
    ALLOWED = 1
    HUMAN = 2
    ROBOT = 4
    STATIC = 8
    HUMAN_CRIT = 16


@dataclass
class HumanEnvState:
    """Dataclass for encapsulating the state of the HumanEnv environment.
    HumanEnv environment states should be fully recoverable from the information stored in this class.

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
    """
    sim_state: np.ndarray
    human_animation_ids: np.ndarray
    human_animation_ids_index: int
    animation_start_time: int
    animation_time: int
    low_level_time: int
    human_pos_offset: List[float]
    human_rot_offset: List[float]


class HumanEnv(SingleArmEnv):
    """This is the super class for any environment with a human.

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

        use_camera_obs (bool): if `True`, every observation includes rendered image(s)

        use_object_obs (bool): if `True`, include object information in the observation.

        reward_scale (float | None): Scales the normalized reward function by the amount specified.
            If `None`, environment reward remains unnormalized

        reward_shaping (bool): if `True`, augment environment reward with dense guidance reward.
            Otherwise: use sparse rewards.

        collision_reward (float): Reward to be given in the case of a collision.

        task_reward (float): Reward to be given in the case of completing a task.

        done_at_collision (bool): If `True`, the episode is terminated when a collision occurs

        done_at_success (bool): If `True`, the episode is terminated when the goal is reached

        human_placement_initializer (ObjectPositionSampler): if provided, will
            be used to place the human on every reset, else a `UniformRandomSampler`
            is used by default.

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
        use_camera_obs: bool = True,
        use_object_obs: bool = True,
        reward_scale: Optional[float] = 1.0,
        reward_shaping: bool = False,
        collision_reward: float = -10,
        task_reward: float = 1,
        done_at_collision: bool = False,
        done_at_success: bool = False,
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
            "CMU/62_20",
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
    ):  # noqa: D107
        macros.SIMULATION_TIMESTEP = control_sample_time
        self.mujoco_arena = None
        self.seed = seed
        self.verbose = verbose
        np.random.seed(self.seed)
        # Robot base offset
        if robot_base_offset is None:
            if isinstance(robots, str) or len(robots) == 1:
                robot_base_offset = [0, 0, 0]
            else:
                robot_base_offset = [[0, 0, 0] for robot in robots]
        self.robot_base_offset = np.array(robot_base_offset)
        # Failsafe controller settings
        self.failsafe_controller = None
        self.control_sample_time = control_sample_time
        # Currently, we always use the failsafe controller.
        # If you want to use a different kind of controller, you can set this to False.
        # If you want to deactivate the failsafe controller, set shield_type to "OFF"
        self.use_failsafe_controller = True
        self.shield_type = shield_type
        self.visualize_failsafe_controller = visualize_failsafe_controller
        self.safe_vel = safe_vel
        self.self_collision_safety = self_collision_safety

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs
        # Objects to create
        self.objects = []
        self.obstacles = []
        self.collision_obstacles_joints = dict()

        # Human animation definition
        self.human_animation_names = human_animation_names

        self.human_animation_data = load_human_animation_data(
            human_animation_names=human_animation_names,
            verbose=verbose,
        )

        self.base_human_pos_offset = base_human_pos_offset
        # Input to scipy: quat = [x, y, z, w]
        self.human_base_quat = Rotation.from_quat([0.5, 0.5, 0.5, 0.5])
        self.human_animation_freq = human_animation_freq
        self.low_level_time = int(0)
        self._n_animations_to_sample_at_resets = max(
            int(horizon * n_animations_sampled_per_100_steps / 100),
            1,
        )
        self._human_animation_ids = None
        self._human_animation_ids_index = 0
        self.animation_start_time = 0
        self.human_placement_initializer = None
        self.human_rand = human_rand

        # Pinocchio visualizer
        self.visualize_pinocchio = visualize_pinocchio
        if self.visualize_pinocchio:
            self.pin_viz = pin.visualize.MeshcatVisualizer()
            self.pin_viz.initViewer()

        # RL memory
        self.has_collision = False
        self.collision_type = COLLISION_TYPE.NULL
        # List of objects that may collide with the robot (e.g. manipulation objects)
        # Collision with these objects yields CollsiionType.ALLOWED
        # Set up in self._setup_collision_info()
        self.whitelisted_collision_geoms = None

        # Reward function parameters
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping
        self.collision_reward = collision_reward
        self.task_reward = task_reward
        self.simulation_crash_reward = -10

        self.done_at_collision = done_at_collision
        self.done_at_success = done_at_success

        self.collision_debounce_timer = 0
        self.collision_debounce_delay = collision_debounce_delay

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            mount_types="default",
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
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
        )

        # Override robot controller
        self._create_new_controller()
        self._override_controller()
        # Set the correct position of the robot model if pinocchio is used.
        self._reset_pin_models()
        # Setup collision variables
        self._setup_collision_info()

        self.n_collisions_robot = 0
        self.n_collisions_static = 0
        self.n_collisions_human = 0
        self.n_collisions_critical = 0

    @property
    def human_animation_id(self) -> int:
        """Get the current human animation id in the random list of human animation ids."""
        return self._human_animation_ids[self._human_animation_ids_index]

    @property
    def human_measurement(self) -> List[np.ndarray]:
        return [
            self.sim.data.get_site_xpos("Human_" + joint_element)
            for joint_element in self.human.joint_elements
        ]

    @property
    def human_animation_length(self) -> int:
        """Get the length of the current human animation."""
        return self.human_animation_data[self.human_animation_id][0]["Pelvis_pos_x"].shape[0]

    def step(self, action):
        """Override base step function.

        Takes a step in simulation with control command `action`.
        Controls the human animation.

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
        if self.done:
            raise ValueError("executing action in terminated episode")

        self.timestep += 1
        # Reset collision variable
        self.has_collision = False
        self.collision_type = COLLISION_TYPE.NULL
        # Since the env.step frequency is slower than the mjsim timestep frequency, the internal controller will output
        # multiple torque commands in between new high level action commands. Therefore, we need to denote via
        # 'policy_step' whether the current step we're taking is simply an internal update of the controller,
        # or an actual policy update
        policy_step = True
        failsafe_intervention = False
        try:
            # Loop through the simulation at the model timestep rate until we're ready to take the next policy step
            # (as defined by the control frequency specified at the environment level)
            for i in range(int(self.control_timestep / self.control_sample_time)):
                self.sim.forward()
                self._set_human_measurement(self.human_measurement, self.sim.data.time)
                # The first step i=0 is a policy step, the rest not.
                # Only in a policy step, set_goal of controller will be called.
                self._pre_action(action, policy_step)
                if self.use_failsafe_controller and not failsafe_intervention:
                    for robot in self.robots:
                        if robot.controller.get_safety() is False:
                            failsafe_intervention = True
                            self.failsafe_interventions += 1
                # Step the simulation n times
                for n in range(int(self.control_sample_time / self.model_timestep)):
                    self._control_human(force_update=True)
                    # If qpos or qvel have been modified directly, the user is required to call forward() before step()
                    # if their udd_callback requires access to MuJoCo state set during the forward dynamics.
                    self.sim.forward()
                    # Collision detection needs to be done before the simulation step.
                    # Otherwise, the velocity of the robot might be influenced by the collision.
                    self._collision_detection()
                    self.sim.step()
                    self._update_observables()
                policy_step = False
                self.low_level_time += 1
        except MujocoException as e:
            # There may occur numerical instabilities since we set the human qpos manually.
            # If this happens, we terminate the episode.
            print("[WARNING] Terminating the episode due to numerical instabilities in the simulation.")
            print(e)
            print("Human animation id: {}".format(self.human_animation_id))
            observations = self._get_observations()
            achieved_goal = self._get_achieved_goal_from_obs(observations)
            desired_goal = self._get_desired_goal_from_obs(observations)
            self.goal_reached = False
            info = self._get_info()
            reward = self._compute_reward(
                achieved_goal=achieved_goal,
                desired_goal=desired_goal,
                info=info
                )
            # Add a penalty for breaking the simulation
            reward += self.simulation_crash_reward
            done = True
            return observations, reward, done, info
        if (
            self.use_failsafe_controller
            and self.visualize_failsafe_controller
            and self.has_renderer
            and self.shield_type != "OFF"
        ):
            self._visualize_reachable_sets()
        # Note: this is done all at once to avoid floating point inaccuracies
        self.cur_time += self.control_timestep

        if self.viewer_get_obs:
            # observations = self.viewer._get_observations()
            raise NotImplementedError
        else:
            observations = self._get_observations()

        achieved_goal = self._get_achieved_goal_from_obs(observations)
        desired_goal = self._get_desired_goal_from_obs(observations)
        self.goal_reached = self._check_success(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal
            )
        if self.goal_reached:
            self.n_goal_reached += 1
        info = self._get_info()
        reward = self._compute_reward(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
            info=info
            )
        done = self._compute_done(
            achieved_goal=achieved_goal,
            desired_goal=desired_goal,
            info=info
            )

        if self.viewer is not None and self.renderer != "mujoco":
            self.viewer.update()

        return observations, reward, done, info

    def check_collision_action(self, action):
        """Checks if the given action collides.
        Checks collisions with static environment and self-collision.
        Requires a pinocchio robot model.

        Args:
            action (np.array): Action to execute

        Returns:
            bool: True: Action collides, False: Action is safe
        """
        # Check if the given action would lead to a direct collision with the environment.
        # Update obstacle positions
        for obs in self.obstacles:
            if obs.name in self.collision_obstacles_joints:
                joint_name = self.collision_obstacles_joints[obs.name][0]
                posrot = self.sim.data.get_joint_qpos(joint_name)
                pos = posrot[0:3]
                rot = quat2mat(posrot[3:7])
                self.collision_obstacles_joints[obs.name][1].set_transform(pos, rot)

        if self.visualize_pinocchio:
            self.visualize_pin(viz=self.pin_viz)

        for robot in self.robots:
            if robot.has_gripper:
                arm_action = action[: robot.controller.control_dim]
            else:
                arm_action = action
            scaled_delta = robot.controller.scale_action(arm_action)
            goal_qpos = set_goal_position(
                delta=scaled_delta,
                current_position=self.sim.data.qpos[robot.joint_indexes],
                position_limit=robot.controller.position_limits,
            )
            if isinstance(robot.robot_model, PinocchioManipulatorModel):
                if not self._check_action_safety(robot.robot_model, goal_qpos):
                    # There are several ways to handle unsafe actions
                    return True
        return False

    def reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute the reward based on the achieved goal, the desired goal, and the info dict.

        If `self.reward_shaping`, we use a dense reward, otherwise a sparse reward.
        The sparse reward yields
            - `self.task_reward` if the target is reached
            - `self.object_gripped_reward` if the object is gripped but the target is not reached
            - `-1` otherwise

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions
        Returns:
            float: reward
        """
        reward = self._sparse_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        if self.reward_shaping:
            reward += 1 + self._dense_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        # Add a penalty for self-collisions and collisions with the human
        collision_reward = self._collision_reward(achieved_goal=achieved_goal, desired_goal=desired_goal, info=info)

        reward = reward + collision_reward

        # Scale reward if requested
        if self.reward_scale is not None:
            reward *= self.reward_scale

        return reward

    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a sparse reward based on the achieved goal, the desired goal, and the info dict.

        The sparse reward function yields
            - `self.task_reward` if the target is reached,
            - `-1` otherwise.

        This method may be overridden by subclasses to add subgoal rewards.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: sparse environment reward
        """
        if self.goal_reached:
            return self.task_reward
        else:
            return -1

    def _dense_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a dense guidance reward based on the achieved goal, the desired goal, and the info dict.

        This method may be overridden to add environment-specific dense rewards.

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: dense environment reward
        """
        return 0.0

    def _collision_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute a penalty for self-collisions, collisions with the static environment,
        and critical collisions with the human.

        Collisions that are not critical, or that involve white-listed objects are not penalized

        Args:
            achieved_goal (List[float]): observation of robot state that is relevant for the goal
            desired_goal (List[float]): the desired goal
            info (Dict[str, Any]): dictionary containing additional information like collisions

        Returns:
            float: collision penalty
        """
        if self._check_illegal_collision(COLLISION_TYPE(info["collision_type"])):
            return self.collision_reward
        else:
            return 0.0

    def _get_info(self) -> Dict:
        """Return the info dictionary of this step.

        Returns
            info dict containing:
                * collision: whether there was a collision
                * collision_type: type of collision
                * timeout: whether timeout was reached
                * failsafe_intervention: whether the failsafe controller intervened
                    in this step
        """
        n_collisions = (
            self.n_collisions_static + self.n_collisions_robot + self.n_collisions_human + self.n_collisions_critical
        )

        info = {
            "collision": self.has_collision,
            "collision_type": self.collision_type.value,
            "n_collisions": n_collisions,
            "n_collisions_static": self.n_collisions_static,
            "n_collisions_robot": self.n_collisions_robot,
            "n_collisions_human": self.n_collisions_human,
            "n_collisions_critical": self.n_collisions_critical,
            "timeout": (self.timestep >= self.horizon),
            "failsafe_interventions": self.failsafe_interventions,
            "n_goal_reached": self.n_goal_reached
        }
        return info

    def _compute_reward(
        self,
        achieved_goal: Union[List[float], List[List[float]]],
        desired_goal: Union[List[float], List[List[float]]],
        info: Union[Dict, List[Dict]],
    ) -> Union[float, List[float]]:
        """Compute the reward based on the achieved goal, the desired goal, and the info dict.

        This function can either be called for one sample or a list of samples.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision

        Returns:
            reward (list of rewards)
        """
        if isinstance(info, Dict):
            # Only one sample
            return self.reward(achieved_goal, desired_goal, info)
        else:
            rewards = [
                self.reward(a_g, d_g, i)
                for (a_g, d_g, i) in zip(achieved_goal, desired_goal, info)
            ]
            return rewards

    def _compute_done(
        self,
        achieved_goal: Union[List[float], List[List[float]]],
        desired_goal: Union[List[float], List[List[float]]],
        info: Union[Dict, List[Dict]],
    ) -> Union[bool, List[bool]]:
        """Compute the done flag based on the achieved goal, the desired goal, and the info dict.

        This function can either be called for one sample or a list of samples.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision
        Returns:
            done (list of dones)
        """
        if isinstance(info, Dict):
            # Only one sample
            return self._check_done(achieved_goal, desired_goal, info)
        else:
            return [
                self._check_done(a_g, d_g, i)
                for (a_g, d_g, i) in zip(achieved_goal, desired_goal, info)
            ]

    def _check_success(
        self, achieved_goal: List[float], desired_goal: List[float]
    ) -> bool:
        """Check if the desired goal was reached.

        Should be overridden by subclasses to specify task success conditions.

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
        Returns:
            True if success
        """
        return False

    def _check_done(
        self, achieved_goal: List[float], desired_goal: List[float], info: Dict
    ) -> bool:
        """Compute the done flag based on the achieved goal, the desired goal, and the info dict.

        This function can only be called for one sample.

        Returns `done=True` if either
            - the desired goal was reached and `self.done_at_success=True`
            - a collision occurred and `self.done_at_collision=True`

        Args:
            achieved_goal: observation of robot state that is relevant for goal
            desired_goal: the desired goal
            info: dictionary containing additional information like collision
        Returns:
            done
        """
        if self.done_at_collision and self._check_illegal_collision(COLLISION_TYPE(info["collision_type"])):
            return True

        if self.done_at_success and self._check_success(achieved_goal, desired_goal):
            return True
        return False

    def _check_illegal_collision(self, collision_type: COLLISION_TYPE) -> bool:
        """Check whether an collision type contains information about an illegal collisions.

        Legal collisions are:
            - white-listed collisions
            - collisions with the human below a velocity of `safe_vel`

        Illegal collisions are:
            - collisions with static environment
            - self-collisions
            - collisions with the human above a velocity of `safe_vel`

        Args:
            collision_type (COLLISION_TYPE): The collision type to check

        Returns:
            bool: True if the collision is illegal, False otherwise
        """
        return collision_type in (COLLISION_TYPE.STATIC | COLLISION_TYPE.ROBOT | COLLISION_TYPE.HUMAN_CRIT)

    def _get_achieved_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """Extract the achieved goal from the observation.

        Args:
            observation: The observation after the action is executed

        Returns:
            The achieved goal
        """
        return [0]

    def _get_desired_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """
        Extract the desired goal from the observation.

        Args:
            - observation: The observation after the action is executed

        Returns:
            - The desired goal
        """
        return [0]

    def _setup_collision_info(self):
        """Set up variables for collision detection.

        Sets self.
            robot_collision_geoms
            human_collision_geoms
        """
        # Collision information of robot links.
        # key = collision id, value = robot id
        self.robot_collision_geoms = dict()
        for i in range(len(self.robots)):
            # Arm elements
            for item in self.robots[i].robot_model.contact_geoms:
                self.robot_collision_geoms[self.sim.model.geom_name2id(item)] = i
            # Gripper elements
            for el in self.robots[i].gripper.contact_geoms:
                self.robot_collision_geoms[self.sim.model.geom_name2id(el)] = i
        # Human elements
        self.human_collision_geoms = {
            self.sim.model.geom_name2id(item) for item in self.human.contact_geoms
        }

        self.whitelisted_collision_geoms = set()

    def _check_action_safety(self, robot_model, q):
        """Check if the robot would collide with the environment in the end position of the given action.

        Args:
            q (np.array): Joint configuration

        Returns:
            True: Robot would NOT collide with the static environment
            False: Robot would collide with the static environment
        """
        # Collision with env
        for collision_obstacle in self.collision_obstacles:
            if robot_model.check_collision(q, collision_obstacle):
                return False
        # Self-collision
        return not robot_model.has_self_collision(q)

    def _determine_geom_contact_type(self, geom_id: int) -> COLLISION_TYPE:
        """Determine to which category the given geom belongs, i.e. which COLLISION_TYPE it is associated with.

        Args:
            geom_id (int): The id of the geom

        Returns:
            COLLISION_TYPE: The associated contact type
        """
        if geom_id in self.robot_collision_geoms:
            return COLLISION_TYPE.ROBOT
        elif geom_id in self.human_collision_geoms:
            return COLLISION_TYPE.HUMAN
        elif geom_id in self.whitelisted_collision_geoms:
            return COLLISION_TYPE.ALLOWED
        else:
            return COLLISION_TYPE.STATIC

    def _on_self_collision_detected(self, contact_geom_1: int, contact_geom_2: int):
        """Perform bookkeeping when a self-collision is detected."""
        if self.verbose:
            print(
                "Self-collision detected between ",
                self.sim.model.geom_id2name(contact_geom_1),
                " and ",
                self.sim.model.geom_id2name(contact_geom_2),
            )

        self.collision_type |= COLLISION_TYPE.ROBOT
        self.n_collisions_robot += 1

    def _on_human_collision_detected(self, robot_contact_geom: int, human_contact_geom: int):
        """Perform bookkeeping when a human-robot collision is detected."""
        if self.collision_debounce_timer > 0:
            return
        self.collision_debounce_timer = self.collision_debounce_delay

        if self.verbose:
            print(
                "Human-robot collision detected between ",
                self.sim.model.geom_id2name(human_contact_geom),
                " and ",
                self.sim.model.geom_id2name(robot_contact_geom),
            )
        # <<< This value may not be correct since it rapidely changes BEFORE the collision >>>
        # Ways to handle this:
        # 1) forward dynamic of the robot
        #   --> We need to do this anyway at some point to allow low speed driving
        """
        if contact_type1 == COLLISION_TYPE.ROBOT:
            robot_id = self.robot_collision_geoms[contact.geom1]
        else:
            robot_id = self.robot_collision_geoms[contact.geom2]
        vel_safe = self._check_robot_vel_safe(
            robot_id=robot_id,
            threshold=self.safe_vel,
            q=self.sim.data.qpos[self.robots[robot_id].joint_indexes],
            dq=self.sim.data.qvel[self.robots[robot_id].joint_indexes])
        """
        # 2) Use the velocity of the simulation
        robot_geom_velocity = self.sim.data.geom_xvelp[robot_contact_geom]

        if self.verbose:
            print(f"Robot speed: {robot_geom_velocity}")

        vel_safe = self._check_vel_safe(
            v_arr=robot_geom_velocity, threshold=self.safe_vel,
        )

        if vel_safe:
            self.collision_type |= COLLISION_TYPE.HUMAN
            self.n_collisions_human += 1
            if self.verbose:
                print("Robot at safe speed.")
        else:
            self.collision_type |= COLLISION_TYPE.HUMAN_CRIT
            self.n_collisions_critical += 1
            if self.verbose:
                print("Robot too fast during collision!")

    def _on_allowed_collision_detected(self, robot_contact_geom: int, other_contact_geom: int):
        """Perform bookkeeping when a collision between the robot and a white-listed object is detected."""
        if self.verbose:
            print(
                "Collision with white-listed object detected between ",
                self.sim.model.geom_id2name(other_contact_geom),
                " and ",
                self.sim.model.geom_id2name(robot_contact_geom),
            )

        self.collision_type |= COLLISION_TYPE.ALLOWED

    def _on_static_collision_detected(self, robot_contact_geom: int, other_contact_geom: int):
        """Perform bookkeeping when a collision between the robot and the static environment is detected."""
        if self.verbose:
            print(
                "Collision with static environment detected between ",
                self.sim.model.geom_id2name(other_contact_geom),
                " and ",
                self.sim.model.geom_id2name(robot_contact_geom),
            )
        self.has_collision = True
        self.collision_type |= COLLISION_TYPE.STATIC
        self.n_collisions_static += 1

    def _on_collision_detected(self, contact_type: COLLISION_TYPE, robot_contact_geom: int, other_contact_geom: int):
        """Perform bookkeeping when a collision is detected."""
        self.has_collision = True

        # Self-collision
        if contact_type == COLLISION_TYPE.ROBOT:
            self._on_self_collision_detected(
                contact_geom_1=robot_contact_geom,
                contact_geom_2=other_contact_geom,
            )
        # Collision with the human
        elif contact_type == COLLISION_TYPE.HUMAN:
            self._on_human_collision_detected(
                robot_contact_geom=robot_contact_geom,
                human_contact_geom=other_contact_geom,
            )
        # Collision with a white-listed object
        elif contact_type == COLLISION_TYPE.ALLOWED:
            self._on_allowed_collision_detected(
                robot_contact_geom=robot_contact_geom,
                other_contact_geom=other_contact_geom,
            )
        # Collision with the static environment
        else:
            self._on_static_collision_detected(
                robot_contact_geom=robot_contact_geom,
                other_contact_geom=other_contact_geom,
            )

    def _collision_detection(self):
        """Detect true collisions in the simulation between the robot and the environment.

        Returns:
            CollisionType
        """
        current_robot_collisions = dict()

        self.collision_debounce_timer = max(0, self.collision_debounce_timer - self.model_timestep)

        # Note that the contact array has more than `ncon` entries,
        # so be careful to only read the valid entries.
        for contact in self.sim.data.contact[:self.sim.data.ncon]:
            contact_type1 = self._determine_geom_contact_type(contact.geom1)
            contact_type2 = self._determine_geom_contact_type(contact.geom2)

            # Only collisions with the robot are considered
            if contact_type1 != COLLISION_TYPE.ROBOT and contact_type2 != COLLISION_TYPE.ROBOT:
                continue

            # Add this collision to the current collision dictionary
            current_robot_collisions[cantor_pairing(contact.geom1, contact.geom2)] = 0
            current_robot_collisions[cantor_pairing(contact.geom2, contact.geom1)] = 0
            # If the current collision already existed previously, skip it to avoid double counting
            if cantor_pairing(contact.geom1, contact.geom2) in self.previous_robot_collisions:
                continue

            if contact_type1 == COLLISION_TYPE.ROBOT:
                self._on_collision_detected(
                    contact_type=contact_type2,
                    robot_contact_geom=contact.geom1,
                    other_contact_geom=contact.geom2,
                )
            else:
                self._on_collision_detected(
                    contact_type=contact_type1,
                    robot_contact_geom=contact.geom2,
                    other_contact_geom=contact.geom1,
                )

        self.previous_robot_collisions = current_robot_collisions
        return self.collision_type

    def _check_robot_vel_safe(self, robot_id, threshold, q, dq):
        """Check if all robot joints have a lower cartesian velocity than the given threshold.

        This assumes that the robot model is of type PinocchioManipulatorModel.

        Args:
            robot_id (int): Id of the robot to check
            threshold (double): Maximal cartesian velocity allowes
            q (np.array): Joint configuration
            dq (np.array): Joint velocity

        Returns:
            True: Cartesian velocity of all joints lower than threshold
            False: Cartesian velocity of any joint higher than threshold
        """
        v_joints = self.model.mujoco_robots[robot_id].get_joint_vel(q, dq)
        # +2: 1 for pinocchio base joint, 1 for fake end joint
        for v_joint in v_joints:
            if not self._check_vel_safe(v_joint, threshold):
                return False
        return True

    def _check_vel_safe_element_wise(self, v_arr, threshold):
        """Check if all absolute elements of the velocity vector are below the given threshold.

        Args:
            v_arr (array like): First three entries must be [v_x, v_y, v_z]
            threshold (double): Velocity limit

        Returns:
            True: velocity lower or equal than threshold
            False: velocity higher than threshold
        """
        return np.all(np.abs(v_arr[0:3]) <= threshold)

    def _check_vel_safe(self, v_arr, threshold):
        """Check if the velocity is below the given threshold.

        Args:
            v_arr (array like): First three entries must be [v_x, v_y, v_z]
            threshold (double): Velocity limit

        Returns:
            True: velocity lower or equal than threshold
            False: velocity higher than threshold
        """
        return np.linalg.norm(v_arr[0:3]) <= threshold

    def _setup_arena(self):
        """Set up the mujoco arena.

        Override this to create custom arenas.
        Must define self.mujoco_arena.
        Define self.objects and self.obstacles here.
        """
        # load model for table top workspace
        self.mujoco_arena = TableArena(
            table_full_size=[1, 1, 0.05],
            table_offset=[0.0, 0.0, 0.8],
            xml=xml_path_completion("arenas/table_arena.xml")
        )

        # Arena always gets set to zero origin
        self._set_origin()

        # Modify default agentview camera
        self._set_mujoco_camera()

        # << OBJECTS >>
        # Objects are elements that can be moved around and manipulated.
        # Create objects
        self.objects = []
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

    def _setup_placement_initializer(
        self,
        name: str,
        initializer: Optional[ObjectPositionSampler] = None,
        objects: List[PrimitiveObject] = [],
        x_range: Tuple[float, float] = (0.0, 0.0),
        y_range: Tuple[float, float] = (0.0, 0.0),
        rotation: Tuple[float, float] = (0, 0),
        rotation_axis: str = "z",
        ensure_object_boundary_in_range: bool = False,
        ensure_valid_placement: bool = False,
        reference_pos: List[float] = [0, 0, 0.8],
        z_offset: float = 0.0,
    ) -> ObjectPositionSampler:
        """Setup a placement initializer.

        Args:
            name (str): Name of the initializer
            initializer (ObjectPositionSampler, optional): Initializer to use. Defaults to None.
            objects (List[PrimitiveObject], optional): Objects to initialize. Defaults to [].
            x_range (Tuple[float, float], optional): Range for x position. Defaults to (0.0, 0.0).
            y_range (Tuple[float, float], optional): Range for y position. Defaults to (0.0, 0.0).
            rotation (Tuple[float, float], optional): Range for rotation. Defaults to (0, 0).
            rotation_axis (str, optional): Rotation axis. Defaults to "z".
            ensure_object_boundary_in_range (bool, optional): Ensure that the object boundary is in range.
                Defaults to False.
            ensure_valid_placement (bool, optional): Ensure that the object is placed on a valid surface.
                Defaults to False.
            reference_pos (List[float], optional): Reference position. Defaults to [0, 0, 0.8].
            z_offset (float, optional): Z offset. Defaults to 0.0.
        Returns:
            ObjectPositionSampler: The initialized initializer
        """
        if initializer is not None:
            initializer.reset()
            initializer.add_objects(objects)
        else:
            initializer = UniformRandomSampler(
                name=name,
                mujoco_objects=objects,
                x_range=x_range,
                y_range=y_range,
                rotation=rotation,
                rotation_axis=rotation_axis,
                ensure_object_boundary_in_range=ensure_object_boundary_in_range,
                ensure_valid_placement=ensure_valid_placement,
                reference_pos=reference_pos,
                z_offset=z_offset
            )
        return initializer

    def _set_origin(self,
                    origin: List[float] = [.0, .0, .0]):
        """Set the origin of the arena.

        Args:
            origin (list of 3 floats): Origin of the arena
        """
        self.mujoco_arena.set_origin(origin)

    def _set_mujoco_camera(
        self,
        camera_name: str = "agentview",
        pos: List[float] = [0.0, -1.5, 1.5],
        quat: List[float] = [-0.0705929, 0.0705929, 0.7035742, 0.7035742],
    ):
        """Set the mujoco camera.

        Args:
            camera_name (str): Name of the camera to be set
            pos (list of 3 floats): Position of the camera
            quat (list of 4 floats): Orientation of the camera as quaternion
        """
        # Modify default agentview camera
        self.mujoco_arena.set_camera(
            camera_name=camera_name,
            pos=pos,
            quat=quat,
        )

    def _setup_collision_objects(
        self,
        collision_objects: List[obstacle.ObstacleBase] = [],
        add_table: bool = True,
        add_base: bool = True,
        safety_margin: float = 0.0
    ):
        """Define the collision objects for pinocchio.

        Args:
            collision_objects (list of ObstacleBase): Additional collision objects
            add_table (bool): Add table collision object
            add_base (bool): Add base collision object
            safety_margin (float): Safety margin to add to the table and base collision objects.
        """
        self.collision_obstacles = collision_objects
        # Obstacles should also have a collision object
        if add_table:
            coll_table = obstacle.Box(
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
            self.collision_obstacles.append(coll_table)
        if add_base:
            coll_base = obstacle.Cylinder(
                name="Base",
                r=0.2 + safety_margin,
                z=0.91,
                translation=self.robot_base_offset + np.array([0.0, 0, 0.455]),
            )
            self.collision_obstacles.append(coll_base)
            coll_computer = obstacle.Box(
                name="Computer",
                x=0.3,
                y=0.5,
                z=0.8,
                translation=self.robot_base_offset + np.array([-0.4, 0, 0.35])
            )
            self.collision_obstacles.append(coll_computer)

    def _load_model(self):
        """Define the mujoco models and initialize the manipulation task.

        Define self.human and self.human_placement_initializer.
        The human is always added to the manipulation task.
        """
        super()._load_model()
        # Adjust base pose accordingly
        for i in range(len(self.robots)):
            if self.robot_base_offset.ndim == 2:
                xpos = self.robot_base_offset[i]
            else:
                xpos = self.robot_base_offset
            self.robots[i].robot_model.set_base_xpos(xpos)

        self._setup_arena()
        assert self.mujoco_arena is not None
        # << HUMAN >>
        # Initialize human
        self.human = HumanObject(name="Human")
        # Placement sampler for human
        if self.human_placement_initializer is not None:
            self.human_placement_initializer.reset()
            self.human_placement_initializer.add_objects(self.human)
        else:
            self.human_placement_initializer = UniformRandomSampler(
                name="HumanSampler",
                mujoco_objects=self.human,
                x_range=[-self.human_rand[0], self.human_rand[0]],
                y_range=[-self.human_rand[1], self.human_rand[1]],
                rotation=(-self.human_rand[2], self.human_rand[2]),
                rotation_axis="z",
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=[0.0, 0.0, 0.0],
                z_offset=0.0,
            )

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=[self.human] + self.objects + self.obstacles,
        )

    def _create_new_controller(self):
        """Create a new failsafe controller for all robots."""
        if self.use_failsafe_controller:
            self.failsafe_controller = []
            for i in range(len(self.robots)):
                self.robots[i].controller_config["init_qpos"] = self.robots[i].init_qpos
                self.robots[i].controller_config["base_pos"] = self.robots[i].base_pos
                self.robots[i].controller_config["base_orientation"] = self.robots[
                    i
                ].base_ori
                self.robots[i].controller_config[
                    "control_sample_time"
                ] = self.control_sample_time
                self.robots[i].controller_config["shield_type"] = self.shield_type
                self.failsafe_controller.append(
                    FailsafeController(**self.robots[i].controller_config)
                )
        else:
            self.failsafe_controller = None

    def _override_controller(self):
        """Manually override the controller with the failsafe controller."""
        if self.failsafe_controller is not None:
            for i in range(len(self.robots)):
                self.robots[i].controller = self.failsafe_controller[i]

    def _reset_controller(self):
        """Reset all failsafe controllers."""
        if self.use_failsafe_controller:
            if self.failsafe_controller is None or self.hard_reset or self.deterministic_reset:
                self._create_new_controller()
            else:
                for i in range(len(self.failsafe_controller)):
                    self.failsafe_controller[i].reset(
                        base_pos=self.robots[i].base_pos,
                        base_orientation=self.robots[i].base_ori,
                        shield_type=self.shield_type,
                    )
            self._override_controller()
        else:
            self.failsafe_controller = None

    def _set_human_measurement(self, human_measurement, time):
        """Set the human measurement in the failsafe controller.

        Args:
            human_measurement (list[list[double]]): List of human measurements [x, y, z]-joint positions.
                The order of joints is defined in `human.py`
            time (double): Current time
        """
        if self.failsafe_controller is not None:
            for i in range(len(self.robots)):
                self.robots[i].controller.set_human_measurement(human_measurement, time)

    def _setup_references(self):
        """Set up references to important components.

        A reference is typically an index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        super()._setup_references()
        if self.control_sample_time % self.model_timestep != 0:
            self.control_sample_time = (
                math.floor(self.control_sample_time / self.model_timestep)
                * self.model_timestep
            )

        simulation_step_freq = int(1 / self.model_timestep)
        self.human_animation_step_length = (
            simulation_step_freq / self.human_animation_freq
        )
        assert (
            self.human_animation_step_length >= 1
        ), "No human animation frequency faster than {} Hz is allowed".format(
            self.model_freq
        )
        self.human_joint_addr = []
        self.human_joint_names = []
        for joint_element in self.human.joint_elements:
            for dim in ["_x", "_y", "_z"]:
                joint_name = joint_element + dim
                self.human_joint_names.append(joint_name)
                self.human_joint_addr.append(
                    self.sim.model.get_joint_qpos_addr(
                        self.human.naming_prefix + joint_name
                    )
                )

    def _setup_observables(self):
        """Set up observables to be used for this environment.

        Creates object-based observables if enabled

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()

        # low-level object information
        if self.use_object_obs:
            # Get robot prefix and define observables modality
            # TODO: Allow multi-robot here.
            pf = self.robots[0].robot_model.naming_prefix
            modality = "object"

            @sensor(modality=modality)
            def gripper_pos(obs_cache):
                return (
                    obs_cache[f"{pf}eef_pos"]
                    if f"{pf}eef_pos" in obs_cache
                    else np.zeros(3)
                )

            @sensor(modality=modality)
            def gripper_aperture(obs_cache):
                if f"{pf}gripper_qpos" in obs_cache:
                    gripper_qpos = obs_cache[f"{pf}gripper_qpos"]
                    if not hasattr(self.robots[0].gripper, "qpos_range"):
                        if self.verbose:
                            print("Gripper has no qpos_range attribute. Gripper aperture observable is not normalized!")
                        return np.mean(gripper_qpos)

                    gripper_qpos_range = self.robots[0].gripper.qpos_range
                    normed_qpos = (
                        (gripper_qpos - gripper_qpos_range[0]) / (gripper_qpos_range[1] - gripper_qpos_range[0])
                    )
                    gripper_aperture = np.mean(normed_qpos)
                    return gripper_aperture
                else:
                    return 0

            @sensor(modality=modality)
            def human_joint_pos(obs_cache):
                return np.concatenate(
                    [
                        self.sim.data.get_site_xpos("Human_" + joint_element)
                        for joint_element in self.human.obs_joint_elements
                    ],
                    axis=-1,
                )

            @sensor(modality=modality)
            def vec_eef_to_human_lh(obs_cache):
                """Return the distance from the human left hand to the end effector in each world coordinate."""
                if f"{pf}eef_pos" in obs_cache:
                    return self.sim.data.get_site_xpos(self.human.left_hand)[:3] - obs_cache[f"{pf}eef_pos"][:3]
                else:
                    return np.zeros(3)

            @sensor(modality=modality)
            def dist_eef_to_human_lh(obs_cache):
                if "vec_eef_to_human_lh" in obs_cache:
                    return np.linalg.norm(obs_cache["vec_eef_to_human_lh"])
                else:
                    return 0

            @sensor(modality=modality)
            def vec_eef_to_human_rh(obs_cache):
                """Return the distance from the human right hand to the end effector in each world coordinate."""
                if f"{pf}eef_pos" in obs_cache:
                    return self.sim.data.get_site_xpos(self.human.right_hand)[:3] - obs_cache[f"{pf}eef_pos"][:3]
                else:
                    return np.zeros(3)

            @sensor(modality=modality)
            def dist_eef_to_human_rh(obs_cache):
                if "vec_eef_to_human_rh" in obs_cache:
                    return np.linalg.norm(obs_cache["vec_eef_to_human_rh"])
                else:
                    return 0

            @sensor(modality=modality)
            def vec_eef_to_human_head(obs_cache):
                """Return the distance from the human head to the end effector in each world coordinate."""
                if f"{pf}eef_pos" in obs_cache:
                    return self.sim.data.get_site_xpos(self.human.head)[:3] - obs_cache[f"{pf}eef_pos"][:3]
                else:
                    return np.zeros(3)

            @sensor(modality=modality)
            def dist_eef_to_human_head(obs_cache):
                if "vec_eef_to_human_head" in obs_cache:
                    return np.linalg.norm(obs_cache["vec_eef_to_human_head"])
                else:
                    return 0

            sensors = [
                gripper_pos,
                gripper_aperture,
                vec_eef_to_human_lh,
                dist_eef_to_human_lh,
                vec_eef_to_human_rh,
                dist_eef_to_human_rh,
                vec_eef_to_human_head,
                dist_eef_to_human_head,
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

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        super()._reset_internal()

        # Quick fix for an open issue in robosuite:
        # reset the current_action values of all grippers to 0 so that actions prior to the reset have
        # no effect on the next episode
        for robot in self.robots:
            if isinstance(robot, SingleArm):
                if robot.has_gripper:
                    robot.gripper.current_action = np.zeros(robot.gripper.dof)
            elif isinstance(robot, Bimanual):
                for arm in robot.arms:
                    if robot.has_gripper[arm]:
                        robot.gripper[arm].current_action = np.zeros(robot.gripper[arm].dof)

        self._reset_controller()
        self._reset_pin_models()

        # Reset collision information
        self.previous_robot_collisions = dict()
        self.has_collision = False
        self.goal_reached = False
        self.collision_type = COLLISION_TYPE.NULL
        self.failsafe_interventions = 0
        self.n_collisions_static = 0
        self.n_collisions_robot = 0
        self.n_collisions_human = 0
        self.n_collisions_critical = 0
        self.n_goal_reached = 0

        self.collision_debounce_timer = 0

        self._human_animation_ids = np.random.randint(
            0, len(self.human_animation_data), size=self._n_animations_to_sample_at_resets
        )
        self._human_animation_ids_index = 0

        self.low_level_time = 0
        self.animation_start_time = 0
        self.animation_time = -1

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        if not self.deterministic_reset:
            # Sample from the placement initializer for all objects
            human_placements = self.human_placement_initializer.sample()
            object_placements = self.object_placement_initializer.sample()
            obstacle_placements = self.obstacle_placement_initializer.sample()
            # We know we're only setting a single object (the door), so specifically set its pose
            human_pos, human_quat, _ = human_placements[self.human.name]
            self.human_pos_offset = [
                self.base_human_pos_offset[i] + human_pos[i] for i in range(3)
            ]
            self.human_rot_offset = human_quat
            # Loop through all objects and reset their positions
            for obj_pos, obj_quat, obj in object_placements.values():
                self.sim.data.set_joint_qpos(
                    obj.joints[0],
                    np.concatenate([np.array(obj_pos), np.array(obj_quat)]),
                )
            # Loop through all obstacles and reset their positions
            for obs_pos, obs_quat, obs in obstacle_placements.values():
                self.sim.data.set_joint_qpos(
                    obs.joints[0],
                    np.concatenate([np.array(obs_pos), np.array(obs_quat)]),
                )
                if obs.name in self.collision_obstacles_joints:
                    self.collision_obstacles_joints[obs.name][1].set_transform(
                        translation=np.array(obs_pos), rotation=quat2mat(obs_quat)
                    )

    def _reset_pin_models(self):
        """Set the base pose of the pinocchio robots."""
        for robot in self.robots:
            if isinstance(robot.robot_model, PinocchioManipulatorModel):
                rot = quat2mat(robot.base_ori)
                trans = np.eye(4)
                trans[0:3, 0:3] = rot
                trans[0:3, 3] = robot.base_pos
                robot.robot_model.set_base_placement(trans)

    def _compute_animation_time(self, control_time: float) -> float:
        """Compute the animation time from the control time and the animation start time.

        Can be overridden to implement custom animation time computation, e.g. for looping, pausing, etc.

        Args:
            control_time (float): Current control time

        Returns:
            float: Animation time
        """
        return control_time - self.animation_start_time

    def _progress_to_next_animation(self, animation_start_time: int):
        """Changes the human animation id during an episode.

        Args:
            animation_start_time (int): Current control time. Used to set the animation start time.
        """
        self._human_animation_ids_index = (
            (self._human_animation_ids_index + 1) % self._n_animations_to_sample_at_resets
        )
        self.animation_time = 0
        self.animation_start_time = animation_start_time

    def _control_human(self, force_update: bool = True):
        """Set the human joint positions according to the human animation files.

        Args:
            force_update (bool): Force the human to be controlled
                even if the animation time has not changed since the last call.
        """
        # <<< Time management and animation selection >>>
        # Convert low level time to human animation time
        control_time = math.floor(
            self.low_level_time / self.human_animation_step_length
        )

        updated_animation_time = self._compute_animation_time(control_time)
        # If the animation time would stay the same, there is no need to update the human.
        if updated_animation_time == self.animation_time and not force_update:
            return

        self.animation_time = updated_animation_time
        # Check if current animation is finished
        if self.animation_time > self.human_animation_length - 1:
            self._progress_to_next_animation(animation_start_time=control_time)

        human_animation, human_animation_info = self.human_animation_data[self.human_animation_id]

        # Root bone transformation
        animation_pos = [
            human_animation["Pelvis_pos_x"][self.animation_time],
            human_animation["Pelvis_pos_y"][self.animation_time],
            human_animation["Pelvis_pos_z"][self.animation_time],
        ]
        animation_offset = human_animation_info["position_offset"]
        # These settings are adjusted to fit the CMU motion capture BVH files!
        human_pos = [
            (animation_pos[0] + animation_offset[0]),
            (animation_pos[1] + animation_offset[1]),
            (animation_pos[2] + animation_offset[2]),
        ]
        # Base rotation (without animation)
        animation_offset_rot = Rotation.from_quat(human_animation_info["orientation_quat"])
        human_rot = self.human_base_quat.__mul__(animation_offset_rot)
        # Apply rotation to position
        human_pos = human_rot.apply(human_pos)
        # Add human base translation
        human_pos = [human_pos[i] + self.human_pos_offset[i] for i in range(3)]
        human_base_rotation = quat_to_rot(self.human_rot_offset)
        human_rot = human_base_rotation.__mul__(human_rot)
        # Animation rotation
        rot = Rotation.from_quat(human_animation["Pelvis_quat"][self.animation_time])
        human_rot = human_rot.__mul__(rot)
        human_quat = rot_to_quat(human_rot)
        # Set base position and rotation
        self.sim.data.set_mocap_pos(self.human.root_body, human_pos)
        self.sim.data.set_mocap_quat(self.human.root_body, human_quat)

        # Set rotation of all other joints
        all_joint_pos = [human_animation[key][self.animation_time] for key in self.human_joint_names]
        self.sim.data.qpos[self.human_joint_addr] = all_joint_pos

    def _visualize_reachable_sets(self):
        """Visualize the robot and human reachable set."""
        if self.use_failsafe_controller:
            for i in range(len(self.robots)):
                robot_capsules = self.robots[i].controller.get_robot_capsules()
                for cap in robot_capsules:
                    self.viewer.viewer.add_marker(
                        pos=cap.pos,
                        type=3,
                        size=cap.size,
                        mat=cap.mat.flatten(),
                        rgba=[0.0, 0.0, 1.0, 0.2],
                        label="",
                        shininess=0.0,
                    )
                # These should 100% match for all robots.
                human_capsules = self.robots[i].controller.get_human_capsules()
                for cap in human_capsules:
                    self.viewer.viewer.add_marker(
                        pos=cap.pos,
                        type=3,
                        size=cap.size,
                        mat=cap.mat.flatten(),
                        rgba=[0.0, 1.0, 0.0, 0.2],
                        label="",
                        shininess=0.0,
                    )
                # Visualize human joints
                # for joint_element in self.human.joint_elements:
                #    pos = self.sim.data.get_site_xpos("Human_" + joint_element)
                #    self.viewer.viewer.add_marker(pos=pos, type=2, size=[0.05, 0.05, 0.05],
                #       mat=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                #       rgba=[1.0, 0.0, 0.0, 1.0], label="", shininess=0.0)

    @property
    def _visualizations(self):
        """Set the visualization keywords for this environment.

        Returns:
            set: All components that can be individually visualized for this environment
        """
        vis_set = super()._visualizations
        return vis_set

    def visualize_pin(
        self, viz: pin.visualize.MeshcatVisualizer = None
    ) -> pin.visualize.MeshcatVisualizer:
        """Plot the scenario in a Meshcat visualizer.

        Calls the "visualize" function for each Object in the scenario. As Objects are the high-level representation
        of "things" in the Environment, this visualization function uses their high-level plotting functionalities, so
        visual meshes etc. will be chosen if available.
        """
        if viz is None:
            viz = pin.visualize.MeshcatVisualizer()
            viz.initViewer()

        for obs in self.collision_obstacles:
            obs.visualize(viz)

        for robot in self.robots:
            robot.robot_model.visualize(viz)

        return viz

    def render(self, **kwargs):
        """Render the environment.

        Args:
            **kwargs: Keyword arguments for the render function of the viewer.

        Returns:
            np.ndarray: The rendered image.
        """
        self.viewer.render()

    def get_environment_state(self) -> HumanEnvState:
        """Get the current state of the environment. Can be used for storing/loading.

        Returns:
            HumanEnvState: The current state of the environment.
        """
        return HumanEnvState(
            sim_state=self.sim.get_state().flatten(),
            human_animation_ids=self._human_animation_ids,
            human_animation_ids_index=self._human_animation_ids_index,
            animation_start_time=self.animation_start_time,
            animation_time=self.animation_time,
            low_level_time=self.low_level_time,
            human_pos_offset=self.human_pos_offset,
            human_rot_offset=self.human_rot_offset,
        )

    def set_environment_state(self, state: HumanEnvState):
        """Set the current state of the environment. Can be used for storing/loading.

        Args:
            state (HumanEnvState): The state to be set.
        """
        self.sim.reset()
        self.sim.set_state_from_flattened(state.sim_state)
        self.sim.data.time = 0

        self._human_animation_ids = state.human_animation_ids
        self._human_animation_ids_index = state.human_animation_ids_index
        self.animation_start_time = state.animation_start_time
        self.animation_time = state.animation_time
        self.low_level_time = state.low_level_time
        self.human_pos_offset = state.human_pos_offset
        self.human_rot_offset = state.human_rot_offset
        self._control_human(force_update=True)
        self.sim.forward()

        for robot in self.robots:
            robot_qpos = np.array(self.sim.data.qpos[robot.controller.qpos_index])
            clamp_diff = np.clip(
                robot_qpos,
                robot.controller.position_limits[0],
                robot.controller.position_limits[1]
            ) - robot_qpos
            if np.sum(np.abs(clamp_diff)) > 1e-6:
                if self.verbose:
                    print("Warning: Robot joint limits violated in loaded state!")
                    print("Clamping to joint limits")

                self.init_qpos = robot_qpos + clamp_diff + np.sign(clamp_diff) * 1e-6
                self.sim.data.qpos[robot.controller.qpos_index] = self.init_qpos
                self.sim.forward()
            else:
                self.init_qpos = robot_qpos

        self._reset_controller()
