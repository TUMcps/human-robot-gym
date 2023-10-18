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

from omegaconf import OmegaConf

from hydra.core.config_store import ConfigStore


@dataclass
class RunConfig:
    """Outline of training run properties."""
    n_envs: int
    n_steps: int
    save_freq: int
    test_only: bool
    load_step: Any  # Optional[Union[int, str]]
    id: Optional[str]
    type: str
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
    resetting_interval: Optional[int]


@dataclass
class EnvironmentConfig:
    """Environment sub-configuration."""
    env_id: str
    gripper_types: str
    use_object_obs: bool
    use_camera_obs: bool
    has_renderer: bool
    has_offscreen_renderer: bool
    render_camera: str
    control_freq: int
    horizon: int
    ignore_done: bool
    hard_reset: bool
    use_failsafe_controller: bool
    control_sample_time: float
    human_animation_names: List[str]
    safe_vel: float
    self_collision_safety: float
    seed: int
    verbose: bool


@dataclass
class AlgorithmConfig:
    """Agent algorithm sub-configuration."""
    name: str
    policy: str
    learning_rate: float
    batch_size: int
    verbose: int
    seed: int
    device: str


@dataclass
class RobotConfig:
    """Robot sub-configuration."""
    name: str
    controller_config_path: str
    robot_config_path: str


@dataclass
class CollisionPreventionWrapperConfig:
    """Collision prevention wrapper configuration."""
    replace_type: int
    n_resamples: int


@dataclass
class VisualizationWrapperConfig:
    """Visualization wrapper configuration."""
    pass


@dataclass
class IKPositionDeltaWrapperConfig:
    """IK position delta wrapper configuration."""
    urdf_file: str
    action_limit: float
    x_output_max: float
    x_position_limits: Optional[List[List[float]]]
    residual_threshold: float
    max_iter: int


@dataclass
class ActionBasedExpertImitationRewardWrapperConfig:
    """Action based expert imitation reward wrapper configuration."""
    alpha: float
    rsi_prob: Optional[float]
    dataset_name: str


@dataclass
class StateBasedExpertImitationRewardWrapperConfig:
    """State based expert imitation reward wrapper configuration."""
    alpha: float
    rsi_prob: Optional[float]
    dataset_name: str


@dataclass
class DatasetObsNormWrapperConfig:
    """Dataset observation normalization wrapper configuration."""
    dataset_name: Optional[str]
    mean: Optional[List[float]]
    std: Optional[List[float]]
    squash_factor: Optional[float]


@dataclass
class WrappersConfig:
    """Sub-configuration for selected wrappers."""
    collision_prevention: Optional[CollisionPreventionWrapperConfig]
    visualization: Optional[VisualizationWrapperConfig]
    ik_position_delta: Optional[IKPositionDeltaWrapperConfig]
    action_based_expert_imitation_reward: Optional[ActionBasedExpertImitationRewardWrapperConfig]
    state_based_expert_imitation_reward: Optional[StateBasedExpertImitationRewardWrapperConfig]
    dataset_obs_norm: Optional[DatasetObsNormWrapperConfig]


@dataclass
class ExpertConfig:
    """Expert sub-configuration."""
    id: str
    obs_keys: List[str]


@dataclass
class WandbConfig:
    """WandB sub-configuration."""
    project: str
    entity: Optional[str]
    group: Optional[str]
    name: Optional[str]
    tags: Optional[List[str]]


@dataclass
class TrainingConfig:
    """Main configuration class. Holds all sub-configurations."""
    robot: RobotConfig
    environment: EnvironmentConfig
    wrappers: WrappersConfig
    run: RunConfig
    algorithm: AlgorithmConfig
    expert: Optional[ExpertConfig] = None
    wandb_run: Optional[WandbConfig] = None


@dataclass
class DataCollectionConfig(TrainingConfig):
    """Data collection configuration class. Holds all sub-configurations."""
    dataset_name: str = "dataset"
    n_episodes: int = 100
    start_episode_index: int = 0
    n_threads: Optional[int] = None
    load_episode_index: Optional[int] = None


# Register config class
cs = ConfigStore.instance()
cs.store(name="base", node=TrainingConfig)

# Allows performing arithmetic operations in value interpolation
OmegaConf.register_new_resolver("eval", eval)
