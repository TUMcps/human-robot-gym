"""This file defines dataclasses outlining the structure of the used hydra config files.

To reduce redundancy and verbosity, these classes are held as generic as possible.
They serve as a overview of the general structure of the config files and all fields that are directly used in the code.

Author:
    Felix Trost (FT)

Changelog:
    04.04.23 FT File creation
"""
from typing import Any, List, Optional
from dataclasses import dataclass

from hydra.core.config_store import ConfigStore

from omegaconf import MISSING


@dataclass
class TrainingConfig:
    """Outline of training run properties."""
    n_envs: int
    n_steps: int
    save_freq: int
    test_only: bool
    load_episode: Any  # Optional[Union[int, str]]
    run_id: Optional[str]
    run_type: str
    log_interval: int
    seed: Optional[int]
    eval_seed: Optional[int]
    start_index: int
    n_test_episodes: int
    env_type: str
    obs_keys: Optional[List[str]]
    expert_obs_keys: Optional[List[str]]
    log_info_keys: Optional[List[str]]
    monitor_dir: str
    vec_env_kwargs: dict
    monitor_kwargs: dict
    verbose: bool


@dataclass
class EnvironmentKwargsConfig:
    """Keyword arguments for environment creation."""
    seed: int
    robots: Any = MISSING
    controller_configs: Any = MISSING


@dataclass
class EnvironmentConfig:
    """Environment sub-configuration."""
    env_id: str
    kwargs: EnvironmentKwargsConfig


@dataclass
class AlgorithmKwargsConfig:
    """Keyword arguments for agent creation."""
    seed: int
    tensorboard_log: Optional[str]
    env: Any = MISSING


@dataclass
class AlgorithmConfig:
    """Agent algorithm sub-configuration."""
    name: str
    kwargs: AlgorithmKwargsConfig


@dataclass
class RobotConfig:
    """Robot sub-configuration."""
    name: str
    controller_config_path: str
    robot_config_path: str


@dataclass
class CollisionPreventionWrapperKwargsConfig:
    """Keyword arguments for the collision prevention wrapper."""
    pass


@dataclass
class CollisionPreventionWrapperConfig:
    """Collision prevention wrapper configuration."""
    enabled: bool
    kwargs: CollisionPreventionWrapperKwargsConfig


@dataclass
class VisualizationWrapperConfig:
    """Visualization wrapper configuration."""
    enabled: bool


@dataclass
class IKPositionDeltaWrapperKwargsConfig:
    """Keyword arguments for the IK position delta wrapper."""
    pass


@dataclass
class IKPositionDeltaWrapperConfig:
    """IK position delta wrapper configuration."""
    enabled: bool
    kwargs: IKPositionDeltaWrapperKwargsConfig


@dataclass
class WrappersConfig:
    """Sub-configuration for selected wrappers."""
    collision_prevention: Optional[CollisionPreventionWrapperConfig]
    visualization: Optional[VisualizationWrapperConfig]
    ik_position_delta: Optional[IKPositionDeltaWrapperConfig]


@dataclass
class WandbConfig:
    """WandB sub-configuration."""
    project: str
    entity: Optional[str]
    group: Optional[str]
    name: Optional[str]
    tags: Optional[List[str]]


@dataclass
class Config:
    """Main configuration class. Holds all sub-configurations."""
    robot: RobotConfig
    environment: EnvironmentConfig
    wrappers: WrappersConfig
    training: TrainingConfig
    algorithm: AlgorithmConfig
    wandb: Optional[WandbConfig] = None


# Register config class
cs = ConfigStore.instance()
cs.store(name="base", node=Config)
