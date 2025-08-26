"""Generic RoboSuite Human Environment.

This module provides a generic base class that can wrap any robosuite environment
and add human simulation capabilities using composition instead of inheritance.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created generic RoboSuiteHumanEnv to eliminate code duplication
"""

from typing import Any, Dict, Union, List, Optional, Tuple

from robosuite.models.tasks import ManipulationTask

from .human_simulation_mixin import HumanSimulationMixin


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
            goal_dist=goal_dist,
            n_goals_sampled_per_100_steps=n_goals_sampled_per_100_steps,
            robot_base_offset=robot_base_offset,
            **kwargs
        )
        
        # Filter out human-specific parameters before passing to robosuite
        robosuite_kwargs = {k: v for k, v in kwargs.items() if k not in {
            'base_human_pos_offset', 'human_animation_names', 'human_animation_freq',
            'human_rand', 'base_human_joint_pos', 'human_observable',
            'n_animations_sampled_per_100_steps', 'safe_vel', 'self_collision_safety',
            'collision_debounce_delay', 'shield_type', 'visualize_failsafe_controller',
            'visualize_pinocchio', 'control_sample_time', 'goal_dist',
            'n_goals_sampled_per_100_steps', 'robot_base_offset', 'arena_config'
        }}
        
        # Initialize the robosuite environment using multiple inheritance approach
        super(HumanSimulationMixin, self).__init__(**robosuite_kwargs)

    def _get_default_arena_config(self):
        """Get default arena configuration.
        
        Can be overridden by subclasses for environment-specific settings.
        
        Returns:
            dict: Default arena configuration
        """
        return {
            "add_table": True,
            "add_base": True,
            "safety_margin": 0.01
        }

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
        self.reset_human_simulation()

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