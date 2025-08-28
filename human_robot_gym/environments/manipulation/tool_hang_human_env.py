"""RoboSuite ToolHang environments with human simulation and safety features.

This module provides ToolHang environments using the new mixin-based architecture
to eliminate code duplication while maintaining full functionality.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Refactored to use HumanSimulationMixin architecture
"""

from robosuite.environments.manipulation.tool_hang import ToolHang

from .robosuite_human_env import RoboSuiteHumanEnv


class ToolHangHumanEnv(RoboSuiteHumanEnv, ToolHang):
    """Base class for ToolHang environments with human simulation.

    Uses the new mixin-based architecture to eliminate code duplication
    while maintaining all functionality.
    """

    def __init__(self, **kwargs):
        """Initialize ToolHang environment with human simulation.

        Args:
            **kwargs: All parameters for human simulation and robosuite ToolHang
        """
        # Initialize using the new architecture
        super().__init__(
            robosuite_env_class=ToolHang,
            arena_config={
                "add_table": True,
                "add_base": True,
                "safety_margin": 0.01
            },
            **kwargs
        )
