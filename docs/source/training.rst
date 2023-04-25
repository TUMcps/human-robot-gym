Reinforcement Learning Training
===============================

The human-robot-gym is designed for training reinforcement learning agents.
The training is done in two steps:

    - Define the the training parameters in the config files.
    - Run the training.

Defining training parameters
----------------------------

We use the `hydra <https://hydra.cc/>`_ library for defining configuration files for the training.
This allows us to define the parameters in a hierarchical way and to override them with other config files and from the command line.
The configuration files can be found at `human_robot_gym/training/config`.
It contains several subfolders defining different aspects of the training:

    - `algorithm`: defines the learning algorithm used. Currently, we support algorithms from the `stable-baselines3` framework.
    - `environment`: configuration files for the different environments provided by the human-robot-gym.
    - `expert`: specifies expert policies that could be used for imitation learning or RL from demonstrations (optional).
    - `robot`: which robot the agent should control inside the environment.
    - `training`: defines the training hyperparameters, such as the number of training steps, the number of episodes, etc.
    - `wandb_run`: configures the logging to `Weights & Biases <https://wandb.ai>`_ (optional).
    - `wrappers`: used to add different wrappers to the environment to enable different features:

        - `action_based_expert_imitation_reward`: augments the reward function by adding an expert behavioral cloning term. Requires an expert to be configured.
        - `collision_prevention`: Enables action replacement using the SaRA safety shield.
        - `ik_position_delta`: Changes the action space from joint space to Cartesian space. As such, actions are interpreted as intended cartesian movement of the robot end effector and joint torques are computed using inverse kinematics.
        - `visualization`: render the environment at every step

To examine how a complete configuration file looks like, you can take a look at `human_robot_gym/training/config/human_reach_ppo_parallel.yaml`.
A basic structure of the config files can be examined in `human-robot-gym/utils/config_utils.py`.

Running the training
--------------------

We provide training scripts for different training scenarios. The scripts can be found in the `human_robot_gym/training` folder.
As all parameters that are relevant for training should be defined in the config, these files mainly differ in the config file that is used per default.

To choose a different config file, you can use the ``--config-name`` flag:

.. code-block:: bash

    python human_robot_gym/training/train_human_reach_PPO_parallel.py --config-name custom_config_name 

If you want to display the assembled config file instead of running the training, you can use the ``--cfg job`` argument:

.. code-block:: bash

    python human_robot_gym/training/train_human_reach_PPO_parallel.py --cfg job

For more information, please refer to the `hydra documentation <https://hydra.cc/docs/advanced/hydra-command-line-flags/>`_.

Config parameters can be overridden from the command line:

.. code-block:: bash

    python human_robot_gym/training/train_human_reach_PPO_parallel.py training.run_type=debug training.n_envs=8 environment.horizon=1000

The configuration files are also stored in the `outputs/` directory at the corresponding timestamp.

You can also choose to upload details about your training run to `Weights & Biases <https://wandb.ai>`_.
This may make it easier to gather training runs from multiple devices and to compare and organize them.

To enable this, you need to create an account and install the `wandb` package by running

.. code-block:: bash

    pip install wandb

Then you can configure wandb for your training:

.. code-block:: bash

    python human_robot_gym/training/train_human_reach_PPO_parallel.py training.run_type=wandb wandb_run.project_name=my_project wandb_run.name=name_of_my_run wandb_run.group=group_of_my_run

To evaluate a training by deploying the trained policy on a rendered test environment, add ``eval`` to the end of the defaults list. For example:

.. code-block:: yaml

    defaults:
      - robot: schunk
      - environment: reach_human
      - wrappers: safe
      - training: parallel_training
      - algorithm: ppo
      - wandb_run: default_wandb
      - _self_
      - consistent_seeding
      - eval
