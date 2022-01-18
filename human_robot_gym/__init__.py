from robosuite.environments.base import make

from .environments.manipulation.reach import Reach
from .environments.manipulation.reach_human import ReachHuman

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
