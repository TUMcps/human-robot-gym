# human-robot-gym

This project is designed for reinforcement learning in human-robot environments.
We provide environments, safety functionality, and training scripts.

# Documentation
[**Read the documentation**](https://cps-rl.pages.gitlab.lrz.de/human-robot-gym/docs/)

To generate the documentation locally, install
```
apt install sphinx graphviz
pip install sphinx_rtd_theme
pip install sphinx-autoapi
pip install graphviz
```
Then make the documentation with
```
cd docs
make html
```

Inspect the documentation at `docs/build/html/index.html`.

# Installation
### Clone the repo with submodules
```
git clone --recurse-submodules git@github.com:TUMcps/human-robot-gym.git
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
sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3 libgtest-dev
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
conda activate hrgym
```
All requirements will automatically get installed by conda.
### Install the failsafe controller / safety shield
The installation requires `gcc`, `c++>=17`, and `Eigen3` version 3.4 (download it here: https://eigen.tuxfamily.org/index.php?title=Main_Page).
Set the path to your eigen3 installation to this env variable, e.g.,
```
export EIGEN3_INCLUDE_DIR="/usr/include/eigen3/eigen-3.4.0"
```
Now run
```
cd human-robot-gym/human_robot_gym/controllers/failsafe_controller/sara-shield
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
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USER/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
```

# Test a demo
```
python human_robot_gym/demos/demo_reach_human_environment.py
```

# Run a RL training
```
python human_robot_gym/training/train_SB3.py -cn human_reach_ppo_parallel
```
You can activate weights and biases with
```
python human_robot_gym/training/train_SB3.py -cn human_reach_ppo_parallel run.type=wandb
```

# Known issues
### `GLIBCXX_3.4.29' not found
```
ImportError: /lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /opt/conda/envs/hrgym/lib/python3.8/site-packages/google/protobuf/pyext/_message.cpython-38-x86_64-linux-gnu.so
```
Solution
```
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get upgrade libstdc++6
sudo apt-get dist-upgrade
```

### libGLEW.so from LD_PRELOAD cannot be preloaded
[The error](https://github.com/openai/mujoco-py/issues/44)
```
ERROR: ld.so: object '/usr/lib/x86_64-linux-gnu/libGLEW.so' from LD_PRELOAD cannot be preloaded (cannot open shared object file): ignored.
```
can be fixed by adding
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```
to your bashrc.

### E: Package 'libgl1-mesa-glx' has no installation candidate
Try the installation without `libgl1-mesa-glx` and use this fix if the error remains: https://askubuntu.com/questions/1517352/issues-installing-libgl1-mesa-glx

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


# ICRA 2024 Experiment Reproduction

To reproduce the results described in the [human-robot-gym paper](https://arxiv.org/pdf/2310.06208.pdf), execute

```
./icra_2024_run_experiments_in_docker.sh
```

This will
- Create a docker image,
- Run a container, and
- Run all experiments within the container.

Please note that due to the amount of training runs (6 environments x 4 methods x 5 random seeds), the execution of this command will take a very significant amount of time (order of magnitude: 1 month) and will run on up to 50 threads.

To recreate the exact plots, please see the documentation in `results_ICRA2024`.