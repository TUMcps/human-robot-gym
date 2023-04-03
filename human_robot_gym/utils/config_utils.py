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
    obs_keys: List[str]
    expert_obs_keys: List[str]
    log_info_keys: List[str]
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
    entity: str
    group: str
    name: Optional[str]
    tags: Optional[List[str]]


@dataclass
class Config:
    robot: RobotConfig
    environment: EnvironmentConfig
    wrappers: WrappersConfig
    training: TrainingConfig
    algorithm: AlgorithmConfig
    wandb: WandbConfig


cs = ConfigStore.instance()
cs.store(name="base_training_config", node=TrainingConfig)
cs.store(name="base_environment_kwargs_config", node=EnvironmentKwargsConfig)
cs.store(name="base_environment_config", node=EnvironmentConfig)
cs.store(name="base_algorithm_kwargs_config", node=AlgorithmKwargsConfig)
cs.store(name="base_algorithm_config", node=AlgorithmConfig)
cs.store(name="base_robot_config", node=RobotConfig)
cs.store(name="base_collision_prevention_wrapper_kwargs_config", node=CollisionPreventionWrapperKwargsConfig)
cs.store(name="base_collision_prevention_wrapper_config", node=CollisionPreventionWrapperConfig)
cs.store(name="base_visualization_wrapper_config", node=VisualizationWrapperConfig)
cs.store(name="base_ik_position_delta_wrapper_kwargs_config", node=IKPositionDeltaWrapperKwargsConfig)
cs.store(name="base_ik_position_delta_config", node=IKPositionDeltaWrapperConfig)
cs.store(name="base_wrappers_config", node=WrappersConfig)
cs.store(name="base_wandb_config", node=WandbConfig)
cs.store(name="base_config", node=Config)
