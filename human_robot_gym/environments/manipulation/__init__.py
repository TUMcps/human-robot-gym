"""This package defines the manipulation environments in the human-robot-gym."""

# Original human-robot-gym environments - using actual class names from grep results
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman  # noqa: F401
from human_robot_gym.environments.manipulation.reach_human_cartesian_env import ReachHumanCart  # noqa: F401
from human_robot_gym.environments.manipulation.pick_place_human_cartesian_env import PickPlaceHumanCart  # noqa: F401
from human_robot_gym.environments.manipulation.pick_place_close_human_cartesian_env import PickPlaceCloseHumanCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.pick_place_pointing_human_cartesian_env import PickPlacePointingHumanCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.human_robot_handover_cartesian_env import HumanRobotHandoverCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.robot_human_handover_cartesian_env import RobotHumanHandoverCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.human_object_inspection_cartesian_env import HumanObjectInspectionCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.collaborative_lifting_cartesian_env import CollaborativeLiftingCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.collaborative_hammering_cartesian_env import CollaborativeHammeringCart  # noqa: F401, E501
from human_robot_gym.environments.manipulation.collaborative_stacking_cartesian_env import CollaborativeStackingCart  # noqa: F401, E501

# Robomimic environments with human simulation and safety
from human_robot_gym.environments.manipulation.lift_human_env import (  # noqa: F401
    LiftHumanEnv,
)

from human_robot_gym.environments.manipulation.pick_place_human_env import (  # noqa: F401
    PickPlaceCanHumanEnv,
)

from human_robot_gym.environments.manipulation.nut_assembly_human_env import (  # noqa: F401
    NutAssemblySquareHumanEnv,
)