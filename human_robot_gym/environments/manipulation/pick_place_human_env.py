"""RoboSuite PickPlace environments with human simulation and safety features.

This module provides PickPlace environments using the new mixin-based architecture
to eliminate code duplication while maintaining full functionality.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Refactored to use HumanSimulationMixin architecture
"""

from robosuite.environments.manipulation.pick_place import PickPlace

from .robosuite_human_env import RoboSuiteHumanEnv


class BasePickPlaceHumanEnv(RoboSuiteHumanEnv, PickPlace):
    """Base class for PickPlace environments with human simulation.

    Uses the new mixin-based architecture to eliminate code duplication
    while maintaining all functionality.
    """

    def __init__(self, **kwargs):
        """Initialize PickPlace environment with human simulation.

        Args:
            **kwargs: All parameters for human simulation and robosuite PickPlace
        """
        # Initialize using the new architecture
        super().__init__(
            robosuite_env_class=PickPlace,
            arena_config={
                "add_table": False,  # PickPlace has its own table handling
                "add_base": True,
                "safety_margin": 0.01
            },
            **kwargs
        )


class PickPlaceCanHumanEnv(BasePickPlaceHumanEnv):
    """PickPlaceCan task with human simulation."""

    def __init__(self, **kwargs):
        assert "single_object_mode" not in kwargs and "object_type" not in kwargs, "invalid set of arguments"
        super().__init__(single_object_mode=2, object_type="can", **kwargs)
