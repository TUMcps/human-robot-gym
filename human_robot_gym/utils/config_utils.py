from typing import Any, List, Optional
from dataclasses import dataclass

from hydra.core.config_store import ConfigStore
from omegaconf import MISSING


@dataclass
class TrainingConfig:
    n_envs: int
    n_steps: int
    save_freq: int
    test_only: bool
    load_episode: Any = None  # Optional[Union[int, str]]
    run_id: Optional[str] = MISSING
    run_type: str
    log_interval: int
    seed: int
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
    pass


@dataclass
class EnvironmentConfig:
    env_id: str
    kwargs: EnvironmentKwargsConfig


@dataclass
class AlgorithmKwargsConfig:
    pass


@dataclass
class AlgorithmConfig:
    name: str
    kwargs: AlgorithmKwargsConfig


@dataclass
class RobotConfig:
    name: str
    controller_config_path: str
    robot_config_path: str


@dataclass
class CollisionPreventionWrapperKwargsConfig:
    pass


@dataclass
class CollisionPreventionWrapperConfig:
    enabled: bool
    kwargs: CollisionPreventionWrapperKwargsConfig


@dataclass
class VisualizationWrapperConfig:
    enabled: bool


@dataclass
class IKPositionDeltaWrapperKwargsConfig:
    pass


@dataclass
class IKPositionDeltaWrapperConfig:
    enabled: bool
    kwargs: IKPositionDeltaWrapperKwargsConfig


@dataclass
class WrappersConfig:
    collision_prevention: Optional[CollisionPreventionWrapperConfig] = None
    visualization: Optional[VisualizationWrapperConfig] = None
    ik_position_delta: Optional[IKPositionDeltaWrapperConfig] = None


@dataclass
class WandbConfig:
    project: str
    entity: Optional[str]
    group: Optional[str]
    name: Optional[str]
    tags: Optional[List[str]]


@dataclass
class Config:
    robot: RobotConfig
    environment: EnvironmentConfig
    wrappers: WrappersConfig
    training: TrainingConfig
    algorithm: AlgorithmConfig
    wandb: Optional[WandbConfig] = None


cs = ConfigStore.instance()
cs.store(name="base", node=Config)
