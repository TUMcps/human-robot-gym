"""This is the human-robot-gym."""
# flake8: noqa
import os
from robosuite.environments.base import make

# Import safety controllers and factory
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.safety_controller_base import (
    SafetyController,
)
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.sara_shield_controller import (
    SaraShieldController,
)
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.cbf_safety_controller import (
    CBFSafetyController,
)
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.safety_controller_factory import (
    create_safety_controller,
)

# Keep old import for backwards compatibility
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.sara_shield_controller import (
    SaraShieldController as FailsafeController,  # Alias for backwards compatibility
)

from human_robot_gym.environments.manipulation.human_env import HumanEnv
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman

from robosuite.environments import ALL_ENVIRONMENTS
from robosuite.controllers import ALL_COMPOSITE_CONTROLLERS, ALL_PART_CONTROLLERS, load_composite_controller_config
from robosuite.robots import ALL_ROBOTS
from robosuite.models.grippers import ALL_GRIPPERS

__version__ = "0.1.0"
__logo__ = """
                                        
  .,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,.   
  .                                 .   
  .............            ,(/,,(##..   
  .*,,,,(,,,,,(    (#(/*,,,,,/*  /,*.   
  .*,,,,(,,,,,(  */###(#*(.      /,(.   
  .*,,,,*......   (/*/(        //##*.   
  .*,,,,,         *,,,/      *   .* .   
  .*,,,,,         *,,,/       .*.   .   
  .*,,,,,         *,,,/             .   
  .*,,,,(,,,,,(   *,,,/             .   
  .*,,,,(,,,,,(,,,/,,,/             .   
  .*,,,,(,,,,,(,,,/,,,/             .   
  ..*****,,,,,*,,,*,,,,             .   
  .                                 .   
                                        
"""

human_robot_gym_root = os.path.dirname(__file__)
