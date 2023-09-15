"""This package defines all environments and necessary gym wrappers."""
from robosuite.environments.base import REGISTERED_ENVS, MujocoEnv  # noqa: F401

from human_robot_gym.environments.manipulation.human_env import HumanEnv  # noqa: F401
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman  # noqa: F401
from human_robot_gym.environments.manipulation.reach_human_cartesian_env import ReachHumanCart  # noqa: F401
from human_robot_gym.environments.manipulation.pick_place_human_cartesian_env import PickPlaceHumanCart  # noqa: F401
from human_robot_gym.environments.manipulation.pick_place_close_human_cartesian_env import (  # noqa: F401
    PickPlaceCloseHumanCart
)
from human_robot_gym.environments.manipulation.robot_human_handover_cartesian_env import (  # noqa: F401
    RobotHumanHandoverCart
)
from human_robot_gym.environments.manipulation.pick_place_pointing_human_cartesian_env import (  # noqa: F401
    PickPlacePointingHumanCart
)
from human_robot_gym.environments.manipulation.human_object_inspection_cartesian_env import (  # noqa: F401
    HumanObjectInspectionCart
)
from human_robot_gym.environments.manipulation.collaborative_hammering_cartesian_env import (  # noqa: F401
    CollaborativeHammeringCart
)

ALL_ENVIRONMENTS = REGISTERED_ENVS.keys()
