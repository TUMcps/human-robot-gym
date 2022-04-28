# human-robot-gym

This project is designed for reinforcement learning in human-robot environments.
We provide environments, safety functionality, and training scripts.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules git@gitlab.lrz.de:cps-rl/human-robot-gym.git
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
If you haven't done already, [install anaconda](https://docs.anaconda.com/anaconda/install/linux/), and create a new conda environment:
```
conda create -n hrgym python=3.8
```
### Install the failsafe controller / safety shield
This requires `cmake`.
```
cd human-robot-gym/human_robot_gym/controllers/failsafe_controller
pip install -r requirements.txt
python setup.py install
```
### Install the human-robot-gym
```
cd human-robot-gym
pip install -e .
```
### Installing pinocchio

Installing `pinocchio` from the `conda-forge` channel can be achieved by adding `conda-forge` to your channels with:

```
conda config --add channels conda-forge
conda config --set channel_priority strict
```

Once the `conda-forge` channel has been enabled, `pinocchio` can be installed with:

```
conda install pinocchio
```

It is possible to list all of the versions of `pinocchio` available on your platform with:

```
conda search pinocchio --channel conda-forge
```

### Add to your `~/.bashrc` 
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/thummj/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```

# Test a demo
```
python human_robot_gym/human_robot_gym/demos/demo_gym_functionality_Schunk.py
```

# Run a RL training
```
python human_robot_gym/human_robot_gym/training/train_human_reach_SB3_sac_her.py schunk_sac_her_safe.json --wandb
```

# Developer's guide
To check your code for linting style:
```
flake8
```
Recommended tool for auto-formatting: 
```
pip install black
```
