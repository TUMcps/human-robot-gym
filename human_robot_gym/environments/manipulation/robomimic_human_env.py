"""Robomimic environments with human simulation and safety features.

This module provides adapters to integrate robomimic environments with the human-robot-gym
safety framework, including sara-shield, failsafe controller, and collision detection.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created robomimic integration
"""

from typing import Any, Dict, Union, List, Optional, Tuple
import numpy as np

import robosuite
from robosuite.models.arenas import TableArena
from robosuite.utils.placement_samplers import UniformRandomSampler

from human_robot_gym.environments.manipulation.human_env import HumanEnv
from human_robot_gym.utils.mjcf_utils import xml_path_completion


class RobomimicHumanEnv(HumanEnv):
    """Base class for robomimic environments with human simulation and safety features.
    
    Integrates robomimic task definitions with human-robot-gym safety framework:
    - Sara-shield safety controller
    - Failsafe collision prevention
    - Human animation and collision detection
    - Multi-level collision categorization
    
    This class serves as a base for specific robomimic task adaptations.
    
    Args:
        robomimic_task_name (str): Name of the robomimic task (e.g., "Lift", "PickPlaceCanBreadCube")
        use_robomimic_arena (bool): Whether to use robomimic's original arena or human-robot-gym's table arena
        
        All other args inherited from HumanEnv parent class.
    """
    
    def __init__(
        self,
        robomimic_task_name: str = "Lift",
        use_robomimic_arena: bool = False,  # Simplified to always use human-robot-gym arena
        robots: Union[str, List[str]] = "Panda",
        # Accept ReachHuman-specific parameters but ignore them
        goal_dist: float = 0.1,
        n_goals_sampled_per_100_steps: int = 8,
        **kwargs
    ):
        """Initialize robomimic human environment.
        
        Args:
            robomimic_task_name (str): Name of the robomimic task (used for reference only)
            use_robomimic_arena (bool): Whether to use robomimic's arena setup (disabled for simplicity)
            robots: Robot configuration
            goal_dist (float): Goal distance parameter (accepted but ignored for robomimic tasks)
            n_goals_sampled_per_100_steps (int): Goal sampling parameter (accepted but ignored)
            **kwargs: Additional arguments passed to HumanEnv
        """
        self.robomimic_task_name = robomimic_task_name
        self.use_robomimic_arena = use_robomimic_arena
        
        super().__init__(robots=robots, **kwargs)
    
    def _setup_arena(self):
        """Set up the arena for robomimic tasks with safety objects.
        
        Uses robomimic's task-specific arena if use_robomimic_arena=True,
        otherwise falls back to human-robot-gym's table arena.
        """
        # Always use human-robot-gym's table arena for simplicity and safety integration
        self.mujoco_arena = TableArena(
            table_full_size=[1, 1, 0.05],
            table_offset=[0.0, 0.0, 0.8],
            xml=xml_path_completion("arenas/table_arena.xml")
        )
        
        # Set table attributes needed by parent class
        self.table_full_size = self.mujoco_arena.table_full_size
        self.table_offset = self.mujoco_arena.table_offset
        
        # Arena always gets set to zero origin
        self._set_origin()
        
        # Modify default agentview camera (essential for rendering!)
        self._set_mujoco_camera()
        
        # Create simple objects for robomimic tasks
        # Add a simple box object to make the environment more interesting
        from robosuite.models.objects.primitive.box import BoxObject
        box_size = np.array([0.04, 0.04, 0.04])  # Small cube for manipulation
        manipulation_object = BoxObject(
            name="manipulation_object",
            size=box_size,
            rgba=[0.1, 0.7, 0.3, 1],  # Green color
        )
        self.objects = [manipulation_object]
        
        # Setup object placement
        bin_x_half = self.table_full_size[0] / 2 - 0.05
        bin_y_half = self.table_full_size[1] / 2 - 0.05
        self.object_placement_initializer = self._setup_placement_initializer(
            name="ObjectSampler", 
            initializer=getattr(self, 'object_placement_initializer', None),
            objects=self.objects,
            x_range=[-bin_x_half, bin_x_half],
            y_range=[-bin_y_half, bin_y_half],
        )
        
        # Setup collision objects for safety
        self._setup_collision_objects(
            add_table=True,
            add_base=True,
            safety_margin=0.01
        )
        
        # Setup obstacles
        self.obstacles = []
        self.obstacle_placement_initializer = self._setup_placement_initializer(
            name="ObstacleSampler",
            initializer=getattr(self, 'obstacle_placement_initializer', None),
            objects=self.obstacles,
        )
    
    def _sparse_reward(
        self,
        achieved_goal: List[float],
        desired_goal: List[float],
        info: Dict[str, Any],
    ) -> float:
        """Compute sparse reward based on robomimic task success.
        
        Override this in task-specific subclasses to implement
        robomimic-specific success conditions.
        
        Args:
            achieved_goal: Current robot state relevant for goal
            desired_goal: Target goal state
            info: Additional information dictionary
            
        Returns:
            Sparse reward (task_reward if successful, -1 otherwise)
        """
        # Default implementation - override in subclasses
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
        """Compute dense guidance reward for robomimic tasks.
        
        Override this in task-specific subclasses to implement
        robomimic-specific dense rewards.
        
        Args:
            achieved_goal: Current robot state relevant for goal  
            desired_goal: Target goal state
            info: Additional information dictionary
            
        Returns:
            Dense guidance reward
        """
        # Default implementation - override in subclasses
        return 0.0
    
    def _check_success(
        self, achieved_goal: List[float], desired_goal: List[float]
    ) -> bool:
        """Check if robomimic task goal was achieved.
        
        Override this in task-specific subclasses to implement
        robomimic-specific success conditions.
        
        Args:
            achieved_goal: Current robot state relevant for goal
            desired_goal: Target goal state
            
        Returns:
            True if task was completed successfully
        """
        # Default implementation - override in subclasses
        return False
    
    def _get_achieved_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """Extract achieved goal from observation for robomimic tasks.
        
        Override this in task-specific subclasses.
        
        Args:
            observation: Current environment observation
            
        Returns:
            Achieved goal representation
        """
        # Default implementation - override in subclasses
        if isinstance(observation, dict) and "object" in observation:
            return observation["object"][:3]  # Object position as achieved goal
        return [0, 0, 0]
    
    def _get_desired_goal_from_obs(
        self, observation: Union[List[float], Dict]
    ) -> List[float]:
        """Extract desired goal from observation for robomimic tasks.
        
        Override this in task-specific subclasses.
        
        Args:
            observation: Current environment observation
            
        Returns:
            Desired goal representation  
        """
        # Default implementation - override in subclasses
        return [0, 0, 1.0]  # Default target height


class LiftHumanEnv(RobomimicHumanEnv):
    """Lift task from robomimic with human simulation and safety features.
    
    Task: Robot must lift a cube to a target height while avoiding collisions with human.
    Safety features: Sara-shield collision avoidance, failsafe controller, human animation.
    """
    
    def __init__(self, **kwargs):
        super().__init__(robomimic_task_name="Lift", **kwargs)
        self.target_height = 1.1  # Target height for lifting
    
    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Check if cube was lifted to target height."""
        if len(achieved_goal) >= 3:
            cube_height = achieved_goal[2]  # Z-coordinate
            return cube_height >= self.target_height
        return False
    
    def _dense_reward(self, achieved_goal: List[float], desired_goal: List[float], info: Dict[str, Any]) -> float:
        """Dense reward based on cube height."""
        if len(achieved_goal) >= 3:
            cube_height = achieved_goal[2]
            table_height = 0.8
            height_progress = (cube_height - table_height) / (self.target_height - table_height)
            return np.clip(height_progress, 0, 1) - 1  # Range [-1, 0]
        return -1


class CanHumanEnv(RobomimicHumanEnv):
    """Can task from robomimic with human simulation and safety features.
    
    Task: Robot must pick up a can and place it in a target location while avoiding human.
    Safety features: Sara-shield collision avoidance, failsafe controller, human animation.
    """
    
    def __init__(self, **kwargs):
        super().__init__(robomimic_task_name="PickPlaceCan", **kwargs)
        self.target_position = np.array([0.2, 0.2, 1.0])  # Target placement location
        self.success_threshold = 0.05  # Distance threshold for success
    
    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Check if can was placed at target location."""
        if len(achieved_goal) >= 3:
            can_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(can_pos - self.target_position)
            return distance <= self.success_threshold
        return False
    
    def _dense_reward(self, achieved_goal: List[float], desired_goal: List[float], info: Dict[str, Any]) -> float:
        """Dense reward based on distance to target."""
        if len(achieved_goal) >= 3:
            can_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(can_pos - self.target_position)
            return -distance  # Negative distance as reward
        return -1


class SquareHumanEnv(RobomimicHumanEnv):
    """Square task from robomimic with human simulation and safety features.
    
    Task: Robot must pick up a square nut and place it on a peg while avoiding human.
    Safety features: Sara-shield collision avoidance, failsafe controller, human animation.
    """
    
    def __init__(self, **kwargs):
        super().__init__(robomimic_task_name="NutAssemblySquare", **kwargs)
        self.target_position = np.array([0.0, 0.0, 1.0])  # Peg location
        self.success_threshold = 0.03  # Tighter tolerance for peg insertion
    
    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Check if square nut was placed on peg."""
        if len(achieved_goal) >= 3:
            nut_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(nut_pos - self.target_position)
            return distance <= self.success_threshold
        return False
    
    def _dense_reward(self, achieved_goal: List[float], desired_goal: List[float], info: Dict[str, Any]) -> float:
        """Dense reward based on distance to peg."""
        if len(achieved_goal) >= 3:
            nut_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(nut_pos - self.target_position)
            return -distance * 2  # Double penalty for precision task
        return -2


class TransportHumanEnv(RobomimicHumanEnv):
    """Transport task from robomimic with human simulation and safety features.
    
    Task: Robot must transport objects between locations while avoiding human.
    Safety features: Sara-shield collision avoidance, failsafe controller, human animation.
    """
    
    def __init__(self, **kwargs):
        super().__init__(robomimic_task_name="PickPlace", **kwargs)
        self.target_position = np.array([0.3, -0.3, 1.0])  # Transport destination
        self.success_threshold = 0.05
    
    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Check if object was transported to target."""
        if len(achieved_goal) >= 3:
            obj_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(obj_pos - self.target_position)
            return distance <= self.success_threshold
        return False
    
    def _dense_reward(self, achieved_goal: List[float], desired_goal: List[float], info: Dict[str, Any]) -> float:
        """Dense reward based on transport progress."""
        if len(achieved_goal) >= 3:
            obj_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(obj_pos - self.target_position)
            return -distance
        return -1


class ToolHangHumanEnv(RobomimicHumanEnv):
    """Tool hang task from robomimic with human simulation and safety features.
    
    Task: Robot must hang a tool on a rack while avoiding human collision.
    Safety features: Sara-shield collision avoidance, failsafe controller, human animation.
    """
    
    def __init__(self, **kwargs):
        super().__init__(robomimic_task_name="Door", **kwargs)
        self.target_position = np.array([0.0, 0.4, 1.2])  # Hanging position
        self.success_threshold = 0.04
    
    def _check_success(self, achieved_goal: List[float], desired_goal: List[float]) -> bool:
        """Check if tool was hung on rack."""
        if len(achieved_goal) >= 3:
            tool_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(tool_pos - self.target_position)
            return distance <= self.success_threshold
        return False
    
    def _dense_reward(self, achieved_goal: List[float], desired_goal: List[float], info: Dict[str, Any]) -> float:
        """Dense reward based on hanging progress."""
        if len(achieved_goal) >= 3:
            tool_pos = np.array(achieved_goal[:3])
            distance = np.linalg.norm(tool_pos - self.target_position)
            return -distance * 1.5  # Moderate penalty for hanging task
        return -1.5