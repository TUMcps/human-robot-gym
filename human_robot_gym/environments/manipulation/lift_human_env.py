"""RoboSuite environments with human simulation and safety features.

This module provides a cleaner approach by directly inheriting from robosuite
environments and adding human simulation capabilities on top.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created RoboSuiteHumanEnv architecture
"""

from typing import Any, Dict, Union, List, Optional, Tuple  # noqa: F401

import math

import numpy as np
from scipy.spatial.transform import Rotation

import pinocchio as pin

from robosuite.models.tasks import ManipulationTask
import robosuite.utils.macros as macros
from robosuite.robots import SingleArm, Bimanual

# from robosuite.models.objects.primitive.box import BoxObject
from robosuite.utils.placement_samplers import (
    UniformRandomSampler
)
from robosuite.utils.transform_utils import quat2mat

from human_robot_gym.models.objects.human.human import HumanObject

from human_robot_gym.utils.animation_utils import load_human_animation_data

from human_robot_gym.controllers.failsafe_controller.failsafe_controller import (  # noqa: F401
    FailsafeController,
)

from robosuite.environments.manipulation.lift import Lift

from human_robot_gym.environments.manipulation.human_env import (  # noqa: F401
    HumanEnv,
    COLLISION_TYPE,
    HumanEnvState,
)


class BaseLiftHumanEnv(Lift):
    """Base class that inherits from robosuite environments and adds human simulation.

    This approach directly inherits from robosuite environments (like Lift) and adds:
    - Human animation and collision detection
    - Sara-shield safety controller
    - Failsafe collision prevention
    - Multi-level collision categorization

    This is much cleaner than the previous approach of copying objects from a separate env.
    """

    def __init__(
        self,
        robots="Panda",
        robot_base_offset=None,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise="default",
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,
        renderer="mujoco",
        renderer_config=None,
        # Human-specific parameters
        base_human_pos_offset=[0.0, 0.0, 0.0],
        human_animation_names=["CMU/62_01"],
        human_animation_freq: float = 120,
        human_rand=[0.0, 0.0, 0.0],
        base_human_joint_pos="default",
        human_observable=True,
        n_animations_sampled_per_100_steps=8,
        safe_vel=0.001,
        self_collision_safety=0.01,
        collision_debounce_delay=0.01,
        seed=0,
        verbose=False,
        # Safety-specific parameters
        shield_type="SSM",
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        control_sample_time: float = 0.004,
        # Goal-related parameters (inherited from HumanEnv)
        goal_dist=0.1,
        n_goals_sampled_per_100_steps=8,
        **kwargs,
    ):
        """Initialize RoboSuite environment with human simulation.

        Args:
            robots: Robot configuration (inherited from robosuite)
            All other robosuite parameters are passed through...
            base_human_pos_offset: Offset for human base position
            human_animation_names: List of human animation files
            human_rand: Random offset for human position
            shield_type: Type of safety shield to use
            goal_dist: Goal distance parameter (from HumanEnv)
            n_goals_sampled_per_100_steps: Goal sampling rate
            **kwargs: Additional arguments
        """
        macros.SIMULATION_TIMESTEP = control_sample_time
        # Store human-specific parameters
        self.base_human_pos_offset = base_human_pos_offset
        self.human_animation_names = human_animation_names
        self.human_rand = human_rand
        self.base_human_joint_pos = base_human_joint_pos
        self.human_observable = human_observable
        self.n_animations_sampled_per_100_steps = n_animations_sampled_per_100_steps
        self.safe_vel = safe_vel
        self.self_collision_safety = self_collision_safety
        self.collision_debounce_delay = collision_debounce_delay
        self.seed = seed
        self.verbose = verbose

        if robot_base_offset is None:
            if isinstance(robots, str) or len(robots) == 1:
                robot_base_offset = [0, 0, 0]
            else:
                robot_base_offset = [[0, 0, 0] for robot in robots]
        self.robot_base_offset = np.array(robot_base_offset)

        # Objects and obstacles
        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs
        # Objects to create
        self.objects = []
        self.obstacles = []
        self.collision_obstacles_joints = dict()
        self.object_placement_initializer = None
        self.obstacle_placement_initializer = None

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

        # Safety parameters
        self.shield_type = shield_type
        self.visualize_failsafe_controller = visualize_failsafe_controller
        self.visualize_pinocchio = visualize_pinocchio

        # Goal parameters (from HumanEnv)
        self.goal_dist = goal_dist
        self.n_goals_sampled_per_100_steps = n_goals_sampled_per_100_steps

        # Human animation definition
        self.human = None
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

        # Define all the stolen functions
        self._setup_human_simulation()

        # Initialize the robosuite Lift environment
        super().__init__(
            robots=robots,
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
            placement_initializer=placement_initializer,
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
            **kwargs,
        )

    @property
    def mujoco_arena(self):
        return self.model.mujoco_arena

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
        return self.human_animation_data[self.human_animation_id][0][
            "Pelvis_pos_x"
        ].shape[0]

    @property
    def _visualizations(self):
        """Set the visualization keywords for this environment.

        Returns:
            set: All components that can be individually visualized for this environment
        """
        vis_set = super()._visualizations
        return vis_set

    def _setup_human_simulation(self):
        """Add human simulation capabilities to the robosuite environment.

        This method adds all the human-robot-gym specific functionality:
        - Human animation system
        - Safety collision objects
        - Sara-shield integration
        - Human observation spaces
        """
        # Import HumanEnv methods for human simulation
        # This is a bit of a hack, but allows us to reuse human simulation code
        from human_robot_gym.environments.manipulation.human_env import HumanEnv

        # Copy essential human simulation methods
        self._setup_collision_objects = HumanEnv._setup_collision_objects.__get__(
            self, type(self)
        )
        self.check_collision_action = HumanEnv.check_collision_action.__get__(
            self, type(self)
        )
        self.step = HumanEnv.step.__get__(self, type(self))
        self._get_info = HumanEnv._get_info.__get__(self, type(self))
        self.check_collision_action = HumanEnv.check_collision_action.__get__(
            self, type(self)
        )
        self._setup_collision_info = HumanEnv._setup_collision_info.__get__(
            self, type(self)
        )
        self._check_action_safety = HumanEnv._check_action_safety.__get__(
            self, type(self)
        )
        self._determine_geom_contact_type = (
            HumanEnv._determine_geom_contact_type.__get__(self, type(self))
        )
        self._setup_placement_initializer = (
            HumanEnv._setup_placement_initializer.__get__(self, type(self))
        )
        self._set_origin = HumanEnv._set_origin.__get__(self, type(self))
        self._set_mujoco_camera = HumanEnv._set_mujoco_camera.__get__(self, type(self))
        self._setup_collision_objects = HumanEnv._setup_collision_objects.__get__(
            self, type(self)
        )
        self._create_new_controller = HumanEnv._create_new_controller.__get__(
            self, type(self)
        )
        self._override_controller = HumanEnv._override_controller.__get__(
            self, type(self)
        )
        self._reset_controller = HumanEnv._reset_controller.__get__(self, type(self))
        self._set_human_measurement = HumanEnv._set_human_measurement.__get__(
            self, type(self)
        )
        self._reset_pin_models = HumanEnv._reset_pin_models.__get__(self, type(self))
        self._compute_animation_time = HumanEnv._compute_animation_time.__get__(
            self, type(self)
        )
        self._progress_to_next_animation = HumanEnv._progress_to_next_animation.__get__(
            self, type(self)
        )
        self._control_human = HumanEnv._control_human.__get__(self, type(self))
        self._visualize_reachable_sets = HumanEnv._visualize_reachable_sets.__get__(
            self, type(self)
        )
        self.visualize_pin = HumanEnv.visualize_pin.__get__(self, type(self))
        self.render = HumanEnv.render.__get__(self, type(self))
        self.get_environment_state = HumanEnv.get_environment_state.__get__(
            self, type(self)
        )
        self.set_environment_state = HumanEnv.set_environment_state.__get__(
            self, type(self)
        )

    def _collision_detection(self):
        pass

    def _setup_arena(self):
        """Set up the mujoco arena.

        Override this to create custom arenas.
        Must define self.mujoco_arena.
        Define self.objects and self.obstacles here.
        """
        # super()._setup_arena()
        # Arena always gets set to zero origin
        self._set_origin()

        # Modify default agentview camera
        self._set_mujoco_camera()

        # << OBJECTS >>
        # Empty placement sampler for objects
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=[],
            x_range=[-bin_x_half, bin_x_half],
            y_range=[-bin_y_half, bin_y_half],
        )

        # << OBSTACLES >>
        self._setup_collision_objects(add_table=True, add_base=True, safety_margin=0.01)
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def _setup_references(self):
        """Set up references to important components.

        A reference is typically an index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        super()._setup_references()
        if self.control_sample_time % self.model_timestep != 0:
            self.control_sample_time = (
                math.floor(self.control_sample_time / float(self.model_timestep))
                * self.model_timestep
            )

        simulation_step_freq = int(1.0 / float(self.model_timestep))
        self.human_animation_step_length = (
            simulation_step_freq / self.human_animation_freq
        )
        assert self.human_animation_step_length >= 1, (
            "No human animation frequency faster than {} Hz is allowed".format(
                self.model_freq
            )
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
                        robot.gripper[arm].current_action = np.zeros(
                            robot.gripper[arm].dof
                        )

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
            0,
            len(self.human_animation_data),
            size=self._n_animations_to_sample_at_resets,
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

    def _load_model(self):
        """Define the mujoco models and initialize the manipulation task.

        Define self.human and self.human_placement_initializer.
        The human is always added to the manipulation task.
        """
        super()._load_model()
        # Adjust base pose accordingly
        # for i in range(len(self.robots)):
        #     if self.robot_base_offset.ndim == 2:
        #         xpos = self.robot_base_offset[i]
        #     else:
        #         xpos = self.robot_base_offset
        #     self.robots[i].robot_model.set_base_xpos(xpos)

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
            mujoco_objects=self.model.mujoco_objects
            + [self.human]
            + self.obstacles,
        )

    def _get_achieved_goal_from_obs(self, obs):
        return obs

    def _get_desired_goal_from_obs(self, obs):
        return obs

    def _check_success(self, achieved_goal=None, desired_goal=None):
        return super()._check_success()

    def _compute_reward(self, achieved_goal=None, desired_goal=None, info=None):
        return super().reward(None)

    def _compute_done(self, achieved_goal=None, desired_goal=None, info=None):
        """Compute if the episode is done."""
        if self.ignore_done:
            return False

        # Check if the cube was lifted to the target height
        # if self._check_success(achieved_goal, desired_goal):
        #     self.goal_reached = True
        #     return True

        if self.timestep >= self.horizon:
            return True
        # Check for collisions
        # if self.has_collision:
        #     return True

        # If we reach here, the episode is not done
        return False


class LiftHumanEnv(BaseLiftHumanEnv):
    """Lift task with human simulation."""

    def __init__(self, **kwargs):
        """Initialize Lift environment with human simulation."""
        super().__init__(**kwargs)
