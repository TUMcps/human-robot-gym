"""RoboSuite Lift environments with human simulation and safety features.

This module provides Lift environments using the new mixin-based architecture
to eliminate code duplication while maintaining full functionality.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Refactored to use HumanSimulationMixin architecture
"""

from robosuite.environments.manipulation.lift import Lift

from .robosuite_human_env import RoboSuiteHumanEnv


class BaseLiftHumanEnv(RoboSuiteHumanEnv, Lift):
    """Base class for Lift environments with human simulation.

    Uses the new mixin-based architecture to eliminate code duplication
    while maintaining all functionality.
    """

    def __init__(self, **kwargs):
        """Initialize Lift environment with human simulation.

        Args:
            **kwargs: All parameters for human simulation and robosuite Lift
        """
        # Initialize using the new architecture
        super().__init__(
            robosuite_env_class=Lift,
            arena_config={
                "add_table": True,
                "add_base": True,
                "safety_margin": 0.01
            },
            **kwargs
        )


class LiftHumanEnv(BaseLiftHumanEnv):
    """Lift task with human simulation."""

    def __init__(self, **kwargs):
        """Initialize Lift environment with human simulation."""
        super().__init__(**kwargs)
