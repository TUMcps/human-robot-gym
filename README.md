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
If you haven't done already, [install anaconda](https://docs.anaconda.com/anaconda/install/linux/).
Add `conda-forge` to your channels with
```
conda config --add channels conda-forge
conda config --set channel_priority strict
```
and create the `hrgym` conda environment:
```
conda env create -f environment.yml
```
All requirements will automatically get installed by conda.
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
pydocstyle
```
Recommended tool for auto-formatting: 
```
pip install black
```
# Dockerization
There are two Dockerfiles 
  - `Dockerfile.build` is the minimum environment neccessary to build and run `human-robot-gym`. This image is used by our CI pipeline.
  - `Dockerfile.dev` is based on Nvidia cuda and is therefore capable of running trainings on the GPU and also provide a visual output as if you would run the code locally. 
### Building the image
We automated the creation of the `Dockerfile.dev` docker image. Simply run
```
./build_docker.sh root
```
### Running an iterative container
After creating the image with `./build_docker.sh`, run 
```
./run_docker.sh root gui
```
to create an iterative container with GUI support, or run
```
./run_docker.sh root 
```
if you don't need the GUI (i.e., training only).
Our `entrypoint.sh` builds `safety_shield_py` and `human_robot_gym` when starting up the container.
We defined the CMD as `/bin/bash`, so that you can use the interactive console after creation.
Now, enjoy your docker container.