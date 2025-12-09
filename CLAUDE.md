# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

human-robot-gym is a reinforcement learning framework for human-robot interaction environments. It extends the robosuite package to train RL algorithms on robots operating in human environments. The project focuses on safety-critical human-robot collaboration scenarios with integrated safety mechanisms.

## Architecture & Key Components

### Core Structure
- **Environments** (`human_robot_gym/environments/`): Gym-compatible environments for various human-robot interaction scenarios
  - `manipulation/`: Task-specific environments (reach, pick-place, handover, etc.)
  - `gym_envs/`: Gym registration and environment factory
- **Models** (`human_robot_gym/models/`): Robot models, human models, and scene assets
  - `robots/`: Schunk manipulator with Pinocchio kinematics
  - `objects/human/`: Human mesh models and animation data
  - `grippers/`: End-effector implementations
- **Safety Systems** (`human_robot_gym/controllers/failsafe_controller/`): Safety-critical collision prevention
  - `sara-shield/`: Reachability-based safety shield using SaRA library
  - `failsafe_controller.py`: Main safety controller implementation
- **Training** (`human_robot_gym/training/`): RL training infrastructure with Stable-Baselines3
  - Hydra-based configuration system for experiments
  - Support for PPO, SAC, and HER algorithms
  - Expert demonstration and imitation learning capabilities
- **Wrappers** (`human_robot_gym/wrappers/`): Environment wrappers for safety, visualization, and data collection

### Configuration System
The project uses Hydra for configuration management with hierarchical YAML configs in `human_robot_gym/training/config/`. Key config categories:
- `environment/`: Environment-specific settings
- `algorithm/`: RL algorithm parameters
- `robot/`: Robot configuration
- `wrappers/`: Wrapper configurations for safety, visualization, etc.

## Development Commands

### Environment Setup
```bash
# Install conda environment
conda env create -f environment.yml
conda activate hrgym

# Install safety shield (requires Eigen3)
export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
cd human_robot_gym/controllers/failsafe_controller/sara-shield
pip install -r requirements.txt
python setup.py install

# Install main package
pip install -e .
```

### Testing & Quality
To run any python code, we should always use the conda environment `hrgym` like so:
```bash
source ~/anaconda3/etc/profile.d/conda.sh && conda activate hrgym && python ...
```

```bash
# Linting
flake8
pydocstyle

# Run tests
python -m pytest human_robot_gym/controllers/failsafe_controller/sara-shield/safety_shield/tests

# Test demo
python human_robot_gym/demos/demo_reach_human_environment.py
```

### Training & Evaluation
```bash
# Basic training
python human_robot_gym/training/train_SB3.py -cn human_reach_ppo_parallel

# With Weights & Biases logging
python human_robot_gym/training/train_SB3.py -cn human_reach_ppo_parallel run.type=wandb

# Create expert datasets
python human_robot_gym/training/create_expert_dataset.py

# Evaluation
python human_robot_gym/training/evaluate_models_to_csv_SB3.py
```

### Documentation
```bash
cd docs
make html
# Output: docs/build/html/index.html
```

## Safety Integration

The clamping prevention system integrates with the safety shield to prevent mechanical clamping between robot and environment. Key safety components:
- **Sara-shield**: Reachability-based collision avoidance using forward reachable sets
- **Failsafe controller**: Real-time trajectory modification for safety
- **Collision detection**: Multi-level collision monitoring (static, robot, human, critical)

## Environment Names & Usage

Key environments available through `make_gym.py`:
- `ReachHuman-v0`: Basic reaching tasks with human presence
- `PickPlaceHuman-v0`: Pick and place with human collaboration
- `CollaborativeLifting-v0`: Joint lifting tasks
- `HumanRobotHandover-v0`: Object handover scenarios

## Important Notes

- MuJoCo 2.1 is required with specific path setup in `~/.bashrc`
- Safety shield requires C++17 and Eigen3 3.4
- Python 3.8 is the supported version
- All demos and training scripts are designed to work with the conda environment
- The project includes Docker support for both development and CI/CD