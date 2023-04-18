"""This is the human-robot-gym."""
# flake8: noqa
import os
from robosuite.environments.base import make

from human_robot_gym.controllers.failsafe_controller.failsafe_controller import (
    FailsafeController,
)

from human_robot_gym.environments.manipulation.human_env import HumanEnv
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman

from robosuite.environments import ALL_ENVIRONMENTS
from robosuite.controllers import ALL_CONTROLLERS, load_controller_config
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
