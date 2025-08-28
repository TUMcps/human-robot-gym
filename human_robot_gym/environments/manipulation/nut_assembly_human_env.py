"""RoboSuite NutAssembly environments with human simulation and safety features.

This module provides NutAssembly environments using the new mixin-based architecture
to eliminate code duplication while maintaining full functionality.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Refactored to use HumanSimulationMixin architecture
"""

from robosuite.environments.manipulation.nut_assembly import NutAssembly

from .robosuite_human_env import RoboSuiteHumanEnv


class BaseNutAssemblyHumanEnv(RoboSuiteHumanEnv, NutAssembly):
    """Base class for NutAssembly environments with human simulation.

    Uses the new mixin-based architecture to eliminate code duplication
    while maintaining all functionality.
    """

    def __init__(self, **kwargs):
        """Initialize NutAssembly environment with human simulation.

        Args:
            **kwargs: All parameters for human simulation and robosuite NutAssembly
        """
        # Initialize using the new architecture
        super().__init__(
            robosuite_env_class=NutAssembly,
            arena_config={
                "add_table": True,
                "add_base": True,
                "safety_margin": 0.01
            },
            **kwargs
        )


class NutAssemblySquareHumanEnv(BaseNutAssemblyHumanEnv):
    """NutAssemblySquare task with human simulation."""

    def __init__(self, **kwargs):
        """Initialize NutAssemblySquare environment with human simulation."""
        assert "single_object_mode" not in kwargs and "nut_type" not in kwargs, "invalid set of arguments"
        super().__init__(single_object_mode=2, nut_type="square", **kwargs)
