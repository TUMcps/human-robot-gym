# human-robot-gym

This project is designed for reinforcement learning in human-robot environments.
We provide environments, safety functionality, and training scripts.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules
```
### Install MuJoCo
1. Download the MuJoCo version 2.1 binaries for
   [Linux](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz) or
   [OSX](https://mujoco.org/download/mujoco210-macos-x86_64.tar.gz).
1. Extract the downloaded `mujoco210` directory into `~/.mujoco/mujoco210`.

If you want to specify a nonstandard location for the package,
use the env variable `MUJOCO_PY_MUJOCO_PATH`.

Under linux, make sure to install: 
```
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
```
### Setup anaconda environment
```
conda create -n hrgym python=3.8
```
### Install the failsafe controller / safety shield
This requires `cmake`.
```
cd human-robot-gym/human_robot_gym/controllers/failsafe_controller
python setup.py install
```
### Install the human-robot-gym
```
cd human-robot-gym
pip install -e .
```

# Test a demo
```
python human_robot_gym/human_robot_gym/demos/demo_gym_functionality_Schunk.py
```

# Run a RL training
```
python human_robot_gym/human_robot_gym/training/train_human_reach_SB3_sac_her.py schunk_sac_her_safe.json --wandb
```
