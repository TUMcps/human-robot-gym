"""This package defines the models loaded by the simulation."""

import os
from robosuite.models.world import MujocoWorldBase  # noqa: F401

assets_root = os.path.join(os.path.dirname(__file__), "assets")
