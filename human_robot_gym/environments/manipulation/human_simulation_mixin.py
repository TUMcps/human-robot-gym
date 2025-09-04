"""Human Simulation Mixin for RoboSuite environments.

This module provides a mixin class that adds human simulation capabilities
to any robosuite environment, eliminating code duplication across different
human-robot interaction environments.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created HumanSimulationMixin to eliminate code duplication
"""

from typing import List

import math

import numpy as np
from scipy.spatial.transform import Rotation

import pinocchio as pin

import robosuite.macros as macros

from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.utils.transform_utils import quat2mat

from human_robot_gym.models.objects.human.human import HumanObject
from human_robot_gym.models.objects.human.single_point_human import SinglePointHumanObject
from human_robot_gym.utils.animation_utils import load_human_animation_data
from human_robot_gym.controllers.failsafe_controller.failsafe_controller import FailsafeController  # noqa: F401

from human_robot_gym.environments.manipulation.human_env import HumanEnv, COLLISION_TYPE
from human_robot_gym.controllers.parts.gripper.composite_controller_patch import (
    apply_composite_controller_patch
)


class HumanSimulationMixin:
    """Mixin that adds human simulation capabilities to any robosuite environment.

    This mixin provides:
    - Human animation and collision detection
    - Sara-shield safety controller
    - Failsafe collision prevention
    - Multi-level collision categorization

    The mixin approach allows adding human simulation to any robosuite environment
    without code duplication or multiple inheritance issues.
    """

    def setup_human_simulation(
        self,
        use_simple_human: bool = True,
        base_human_pos_offset=[0.0, 0.0, 0.0],
        human_animation_names=["CMU/62_01"],
        human_animation_freq: float = 100,
        human_rand=[0.0, 0.0, 0.0],
        base_human_joint_pos="default",
        human_observable=True,
        n_animations_sampled_per_100_steps=8,
        safe_vel=0.001,
        self_collision_safety=0.01,
        collision_debounce_delay=0.01,
        seed=0,
        verbose=False,
        shield_type="SSM",
        visualize_failsafe_controller=False,
        visualize_pinocchio=False,
        control_freq: float = 10,
        control_sample_time: float = 0.004,
        goal_dist=0.1,
        n_goals_sampled_per_100_steps=8,
        robot_base_offset=None,
        horizon=1000,
        use_waypoints_action: bool = False,
        n_waypoints: int = 1,
        **kwargs,
    ):
        """Setup human simulation capabilities.

        Args:
            use_simple_human: Whether to use a simplified human model as dynamic obstacle
            base_human_pos_offset: Offset for human base position
            human_animation_names: List of human animation files
            human_animation_freq: Frequency of human animation playback
            human_rand: Random offset for human position
            base_human_joint_pos: Base human joint position
            human_observable: Whether human is observable
            n_animations_sampled_per_100_steps: Animation sampling rate
            safe_vel: Safe velocity threshold
            self_collision_safety: Self collision safety margin
            collision_debounce_delay: Collision debounce delay
            seed: Random seed
            verbose: Verbose output
            shield_type: Type of safety shield to use
            visualize_failsafe_controller: Whether to visualize failsafe controller
            visualize_pinocchio: Whether to visualize pinocchio
            control_sample_time: Control sample time
            goal_dist: Goal distance parameter
            n_goals_sampled_per_100_steps: Goal sampling rate
            robot_base_offset: Robot base offset
            horizon: Episode horizon
            **kwargs: Additional arguments
        """
        macros.SIMULATION_TIMESTEP = control_sample_time

        # Store human-specific parameters
        self.use_simple_human = use_simple_human
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
            if isinstance(getattr(self, "robots", "Panda"), str) or len(getattr(self, "robots", ["Panda"])) == 1:
                robot_base_offset = [0, 0, 0]
            else:
                robot_base_offset = [[0, 0, 0] for robot in getattr(self, "robots", ["Panda"])]
        self.robot_base_offset = np.array(robot_base_offset)

        # Objects and obstacles
        self.use_object_obs = getattr(self, "use_object_obs", True)
        self.objects = []
        self.obstacles = []
        self.collision_obstacles_joints = dict()
        self.object_placement_initializer = None
        self.obstacle_placement_initializer = None

        # Failsafe controller settings
        self.failsafe_controller = None
        self.gripper_controllers = None
        self.control_sample_time = control_sample_time
        self.use_failsafe_controller = True
        self.shield_type = shield_type
        self.visualize_failsafe_controller = visualize_failsafe_controller
        self.safe_vel = safe_vel
        self.self_collision_safety = self_collision_safety

        # Handle waypoints action settings
        self.use_waypoints_action = use_waypoints_action
        self.n_waypoints = n_waypoints
        total_control_steps = 1.0/(control_freq*control_sample_time)
        self.n_control_steps_per_waypoint = np.floor(total_control_steps/n_waypoints)
        if self.use_waypoints_action and not self.use_failsafe_controller:
            raise NotImplementedError(
                "Waypoints action is only implemented together with the failsafe controller. \
                  You can use ShieldType.OFF to deactivate the failsafe controller."
            )
        # Apply composite controller monkey patch if using waypoint actions
        if self.use_waypoints_action:
            apply_composite_controller_patch()

        # Safety parameters
        self.shield_type = shield_type
        self.visualize_failsafe_controller = visualize_failsafe_controller
        self.visualize_pinocchio = visualize_pinocchio

        # Goal parameters
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

        # Setup human simulation methods
        self._setup_human_simulation_methods()

    def _post_setup_human_simulation(self):
        """Post-setup steps for human simulation."""
        self._create_new_controller()
        self._override_controller(
            override_failsafe=self.use_failsafe_controller,
            override_gripper=self.use_waypoints_action,
            override_action_split=True,
        )
        # Set the correct position of the robot model if pinocchio is used.
        self._reset_pin_models()
        # Setup collision variables
        self._setup_collision_info()

        self.n_collisions_robot = 0
        self.n_collisions_static = 0
        self.n_collisions_human = 0
        self.n_collisions_critical = 0

    def _get_arena_config(self):
        """Get environment-specific arena configuration.

        This method should be overridden by subclasses to provide
        environment-specific configuration.

        Returns:
            dict: Arena configuration parameters
        """
        return {"add_table": True, "add_base": True, "safety_margin": 0.01}

    def _setup_human_simulation_methods(self):
        """Setup human simulation methods by copying them from HumanEnv."""
        # Copy essential human simulation methods from HumanEnv
        self._setup_collision_objects = HumanEnv._setup_collision_objects.__get__(self, type(self))
        self.check_collision_action = HumanEnv.check_collision_action.__get__(self, type(self))
        self.step = HumanEnv.step.__get__(self, type(self))
        self._render_scene = HumanEnv._render_scene.__get__(self, type(self))
        self._get_info = HumanEnv._get_info.__get__(self, type(self))
        self._setup_collision_info = HumanEnv._setup_collision_info.__get__(self, type(self))
        self._check_action_safety = HumanEnv._check_action_safety.__get__(self, type(self))
        self._determine_geom_contact_type = HumanEnv._determine_geom_contact_type.__get__(self, type(self))
        self._setup_placement_initializer = HumanEnv._setup_placement_initializer.__get__(self, type(self))
        self._set_origin = HumanEnv._set_origin.__get__(self, type(self))
        self._set_mujoco_camera = HumanEnv._set_mujoco_camera.__get__(self, type(self))
        self._create_new_controller = HumanEnv._create_new_controller.__get__(self, type(self))
        self._create_new_failsafe_controller = HumanEnv._create_new_failsafe_controller.__get__(self, type(self))
        self._create_new_waypoint_gripper_controllers = HumanEnv._create_new_waypoint_gripper_controllers.__get__(self, type(self))  # noqa: E501
        self._override_controller = HumanEnv._override_controller.__get__(self, type(self))
        self._override_failsafe_controller = HumanEnv._override_failsafe_controller.__get__(self, type(self))
        self._override_gripper_controller = HumanEnv._override_gripper_controller.__get__(self, type(self))
        self._override_composite_controller_action_split = HumanEnv._override_composite_controller_action_split.__get__(self, type(self))  # noqa: E501
        self._reset_controller = HumanEnv._reset_controller.__get__(self, type(self))
        self._set_human_measurement = HumanEnv._set_human_measurement.__get__(self, type(self))
        self._reset_pin_models = HumanEnv._reset_pin_models.__get__(self, type(self))
        self._compute_animation_time = HumanEnv._compute_animation_time.__get__(self, type(self))
        self._progress_to_next_animation = HumanEnv._progress_to_next_animation.__get__(self, type(self))
        self._control_human = HumanEnv._control_human.__get__(self, type(self))
        self._visualize_reachable_sets = HumanEnv._visualize_reachable_sets.__get__(self, type(self))
        self.visualize_pin = HumanEnv.visualize_pin.__get__(self, type(self))
        self.render = HumanEnv.render.__get__(self, type(self))
        self.get_environment_state = HumanEnv.get_environment_state.__get__(self, type(self))
        self.set_environment_state = HumanEnv.set_environment_state.__get__(self, type(self))

    @property
    def human_animation_id(self) -> int:
        """Get the current human animation id in the random list of human animation ids."""
        return self._human_animation_ids[self._human_animation_ids_index]

    @property
    def human_measurement(self) -> List[np.ndarray]:
        return [
          self.sim.data.get_site_xpos(
            f"{self.human.name}_" + joint_element
          ) for joint_element in self.human.joint_elements
        ]

    @property
    def human_animation_length(self) -> int:
        """Get the length of the current human animation."""
        return self.human_animation_data[self.human_animation_id][0]["Pelvis_pos_x"].shape[0]

    def _collision_detection(self):
        pass

    def setup_human_arena(self):
        """Set up the arena with human simulation objects."""
        # Arena always gets set to zero origin
        self._set_origin()

        # Modify default agentview camera
        # self._set_mujoco_camera()

        # Get environment-specific configuration
        arena_config = self._get_arena_config()

        # << OBJECTS >>
        # Setup object placement initializer
        bin_x_half = getattr(self, "table_full_size", (0.8, 0.8, 0.05))[0] / 2 - 0.05
        bin_y_half = getattr(self, "table_full_size", (0.8, 0.8, 0.05))[1] / 2 - 0.05
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler",
            initializer=self.object_placement_initializer,
            objects=[],
            x_range=[-bin_x_half, bin_x_half],
            y_range=[-bin_y_half, bin_y_half],
        )

        # << OBSTACLES >>
        self._setup_collision_objects(**arena_config)
        # Obstacles are elements that the robot should avoid.
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=self.obstacle_placement_initializer,
            objects=self.obstacles,
        )

    def setup_human_references(self):
        """Set up references to human simulation components."""
        if self.control_sample_time % self.model_timestep != 0:
            self.control_sample_time = (
                math.floor(self.control_sample_time / float(self.model_timestep)) * self.model_timestep
            )

        simulation_step_freq = int(1.0 / float(self.model_timestep))
        self.human_animation_step_length = simulation_step_freq / self.human_animation_freq
        assert self.human_animation_step_length >= 1, (
            "No human animation frequency faster than {} Hz is allowed".format(
                getattr(self, "model_freq", simulation_step_freq)
            )
        )
        self.human_joint_addr = []
        self.human_joint_names = []
        for joint_element in self.human.joint_elements:
            for dim in ["_x", "_y", "_z"]:
                joint_name = joint_element + dim
                self.human_joint_names.append(joint_name)
                self.human_joint_addr.append(self.sim.model.get_joint_qpos_addr(self.human.naming_prefix + joint_name))

    def reset_human_simulation(self):
        """Reset human simulation state."""
        # Quick fix for an open issue in robosuite:
        # reset the current_action values of all grippers to 0 so that actions prior to the reset have
        # no effect on the next episode
        for robot in getattr(self, "robots", []):
            # In robosuite 1.5, all robots use FixedBaseRobot with arms dict structure
            if hasattr(robot, "arms") and hasattr(robot, "has_gripper"):
                for arm in robot.arms:
                    if robot.has_gripper.get(arm, False):
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
            0,
            len(self.human_animation_data),
            size=self._n_animations_to_sample_at_resets,
        )
        self._human_animation_ids_index = 0

        self.low_level_time = 0
        self.animation_start_time = 0
        self.animation_time = -1

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        if not getattr(self, "deterministic_reset", False):
            # Sample from the placement initializer for all objects
            human_placements = self.human_placement_initializer.sample()
            object_placements = self.object_placement_initializer.sample()
            obstacle_placements = self.obstacle_placement_initializer.sample()
            # We know we're only setting a single object (the human), so specifically set its pose
            human_pos, human_quat, _ = human_placements[self.human.name]
            self.human_pos_offset = [self.base_human_pos_offset[i] + human_pos[i] for i in range(3)]
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

    def setup_human_model(self):
        """Setup human model and placement."""
        # << HUMAN >>
        # Initialize human
        if self.use_simple_human:
            self.human = SinglePointHumanObject(name="SinglePointHuman")
        else:
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

    def _get_achieved_goal_from_obs(self, obs):
        return obs

    def _get_desired_goal_from_obs(self, obs):
        return obs

    def _check_success(self, achieved_goal=None, desired_goal=None):
        return super()._check_success() if hasattr(super(), "_check_success") else False

    def _compute_reward(self, achieved_goal=None, desired_goal=None, info=None):
        return super().reward(None) if hasattr(super(), "reward") else 0.0

    def _compute_done(self, achieved_goal=None, desired_goal=None, info=None):
        """Compute if the episode is done."""
        if getattr(self, "ignore_done", False):
            return False

        if getattr(self, "timestep", 0) >= getattr(self, "horizon", 1000):
            return True

        # If we reach here, the episode is not done
        return False
