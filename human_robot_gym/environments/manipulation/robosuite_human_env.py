"""Generic RoboSuite Human Environment.

This module provides a generic base class that can wrap any robosuite environment
and add human simulation capabilities using composition instead of inheritance.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created generic RoboSuiteHumanEnv to eliminate code duplication
"""

import numpy as np

from robosuite.models.tasks import ManipulationTask

from human_robot_gym.environments.manipulation.human_env import COLLISION_TYPE
from human_robot_gym.environments.manipulation.human_simulation_mixin import HumanSimulationMixin


class RoboSuiteHumanEnv(HumanSimulationMixin):
    """Generic base class for robosuite environments with human simulation.

    This class can wrap any robosuite environment and add human simulation
    capabilities using composition. It eliminates the need for separate
    implementations for each robosuite environment type.
    """

    def __init__(
        self,
        robosuite_env_class,
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
        use_waypoints_action: bool = False,
        n_waypoints: int = 1,
        # Goal-related parameters
        goal_dist=0.1,
        n_goals_sampled_per_100_steps=8,
        # Environment-specific configuration
        arena_config=None,
        robot_base_offset=None,
        **kwargs,
    ):
        """Initialize RoboSuite environment with human simulation.

        Args:
            robosuite_env_class: The robosuite environment class to wrap
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
            arena_config: Environment-specific arena configuration
            **kwargs: Additional arguments passed to robosuite environment
        """
        # Store the robosuite environment class
        self.robosuite_env_class = robosuite_env_class

        # Store arena configuration
        self._arena_config = arena_config or self._get_default_arena_config()

        # Setup human simulation first
        self.setup_human_simulation(
            base_human_pos_offset=base_human_pos_offset,
            human_animation_names=human_animation_names,
            human_animation_freq=human_animation_freq,
            human_rand=human_rand,
            base_human_joint_pos=base_human_joint_pos,
            human_observable=human_observable,
            n_animations_sampled_per_100_steps=n_animations_sampled_per_100_steps,
            safe_vel=safe_vel,
            self_collision_safety=self_collision_safety,
            collision_debounce_delay=collision_debounce_delay,
            seed=seed,
            verbose=verbose,
            shield_type=shield_type,
            visualize_failsafe_controller=visualize_failsafe_controller,
            visualize_pinocchio=visualize_pinocchio,
            control_sample_time=control_sample_time,
            use_waypoints_action=use_waypoints_action,
            n_waypoints=n_waypoints,
            goal_dist=goal_dist,
            n_goals_sampled_per_100_steps=n_goals_sampled_per_100_steps,
            robot_base_offset=robot_base_offset,
            **kwargs,
        )

        # Filter out human-specific parameters before passing to robosuite
        robosuite_kwargs = {
            k: v
            for k, v in kwargs.items()
            if k
            not in {
                "base_human_pos_offset",
                "human_animation_names",
                "human_animation_freq",
                "human_rand",
                "base_human_joint_pos",
                "human_observable",
                "n_animations_sampled_per_100_steps",
                "safe_vel",
                "self_collision_safety",
                "collision_debounce_delay",
                "shield_type",
                "visualize_failsafe_controller",
                "visualize_pinocchio",
                "control_sample_time",
                "goal_dist",
                "n_goals_sampled_per_100_steps",
                "robot_base_offset",
                "arena_config",
                "use_waypoints_action",
                "n_waypoints"
            }
        }

        # Initialize the robosuite environment using multiple inheritance approach
        super(HumanSimulationMixin, self).__init__(**robosuite_kwargs)
        self._post_setup_human_simulation()

    def _get_default_arena_config(self):
        """Get default arena configuration.

        Can be overridden by subclasses for environment-specific settings.

        Returns:
            dict: Default arena configuration
        """
        return {"add_table": True, "add_base": True, "safety_margin": 0.01}

    def _get_arena_config(self):
        """Get arena configuration for this environment."""
        return self._arena_config

    @property
    def mujoco_arena(self):
        return self.model.mujoco_arena

    @property
    def _visualizations(self):
        """Set the visualization keywords for this environment.

        Returns:
            set: All components that can be individually visualized for this environment
        """
        vis_set = super()._visualizations
        return vis_set

    def _setup_arena(self):
        """Set up the mujoco arena with human simulation."""
        self.setup_human_arena()

    def _setup_references(self):
        """Set up references to important components."""
        super()._setup_references()
        self.setup_human_references()

    def _reset_internal(self):
        """Reset the simulation internal configurations."""
        super()._reset_internal()
        # self.reset_human_simulation()

        # Quick fix for an open issue in robosuite:
        # reset the current_action values of all grippers to 0 so that actions prior to the reset have
        # no effect on the next episode
        # TODO implement this for the new robosuite API
        for robot in self.robots:
            # In robosuite 1.5, all robots use FixedBaseRobot with arms dict structure
            if hasattr(robot, "arms") and hasattr(robot, "has_gripper"):
                for arm in robot.arms:
                    if robot.has_gripper.get(arm, False):
                        robot.gripper[arm].current_action = np.zeros(robot.gripper[arm].dof)

        self._reset_controller()
        self._reset_pin_models()

        if self.use_waypoints_action:
            self._action_dim = 0
            # Reset robot and update action space dimension along the way
            for robot in self.robots:
                self._action_dim += robot.action_dim

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
            # We know we're only setting a single object (the door), so specifically set its pose
            human_pos, human_quat, _ = human_placements[self.human.name]
            self.human_pos_offset = [self.base_human_pos_offset[i] + human_pos[i] for i in range(3)]
            self.human_rot_offset = human_quat

    def _load_model(self):
        """Define the mujoco models and initialize the manipulation task."""
        super()._load_model()

        # Setup arena (which includes human simulation setup)
        self._setup_arena()
        assert self.mujoco_arena is not None

        # Setup human model
        self.setup_human_model()

        # Update the manipulation task to include human and obstacles
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.model.mujoco_objects + [self.human] + self.obstacles,
        )
