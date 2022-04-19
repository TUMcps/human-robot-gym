#!/usr/bin/env python
import os
import struct
import gym
import numpy as np
#
import json
from datetime import datetime

from stable_baselines3 import SAC, HerReplayBuffer
import robosuite
from robosuite.wrappers import GymWrapper
from robosuite.controllers import controller_factory, load_controller_config

import human_robot_gym.robots
from human_robot_gym.environments.manipulation.reach_human_env import ReachHuman
from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.wrappers.goal_env_wrapper import GoalEnvironmentGymWrapper
from human_robot_gym.wrappers.time_limit import TimeLimit
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.HER_buffer_add_monkey_patch import custom_add, _custom_sample_transitions

# Command line arguments: 
# Training config file
# Use wandb

if __name__ == '__main__':
    # TODO: Input args
    training_config_file = "schunk_sac_her_safe.json"
    use_wandb = False

    callback = None
    if use_wandb:
        import wandb
        from human_robot_gym.wrappers.tensorboard_callback import TensorboardCallback

    # Training config 
    training_config_path = file_path_completion("training/config/"+training_config_file)
    try:
        with open(training_config_path) as f:
            training_config = json.load(f)
    except FileNotFoundError:
        print("Error opening controller filepath at: {}. " "Please check filepath and try again.".format(training_config_path))
    
    # Load robot and controller config files
    controller_config = dict()
    controller_conig_path = file_path_completion(training_config["robot"]["controller_config"])
    robot_conig_path = file_path_completion(training_config["robot"]["robot_config"])
    controller_config = load_controller_config(custom_fpath=controller_conig_path)
    robot_config = load_controller_config(custom_fpath=robot_conig_path)
    controller_config = merge_configs(controller_config, robot_config)
    training_config["environment"]["controller_configs"] = [controller_config]

    env = GoalEnvironmentGymWrapper(
        robosuite.make(
            "ReachHuman",
            robots=training_config["robot"]["name"],
            robot_base_offset = training_config["environment"]["robot_base_offset"],
            env_configuration = training_config["environment"]["env_configuration"],
            controller_configs = training_config["environment"]["controller_configs"], 
            gripper_types = training_config["environment"]["gripper_types"],
            initialization_noise = training_config["environment"]["initialization_noise"],
            table_full_size = training_config["environment"]["table_full_size"],
            table_friction = training_config["environment"]["table_friction"],
            use_camera_obs = training_config["environment"]["use_camera_obs"],
            use_object_obs = training_config["environment"]["use_object_obs"],
            reward_scale = training_config["environment"]["reward_scale"],
            reward_shaping = training_config["environment"]["reward_shaping"],
            goal_dist = training_config["environment"]["goal_dist"],
            collision_reward = training_config["environment"]["collision_reward"],
            has_renderer = training_config["environment"]["has_renderer"],
            has_offscreen_renderer = training_config["environment"]["has_offscreen_renderer"],
            render_camera = training_config["environment"]["render_camera"],
            render_collision_mesh = training_config["environment"]["render_collision_mesh"],
            render_visual_mesh = training_config["environment"]["render_visual_mesh"],
            render_gpu_device_id = training_config["environment"]["render_gpu_device_id"],
            control_freq = training_config["environment"]["control_freq"],
            horizon = training_config["environment"]["horizon"],
            ignore_done = training_config["environment"]["ignore_done"],
            hard_reset = training_config["environment"]["hard_reset"],
            camera_names = training_config["environment"]["camera_names"],
            camera_heights = training_config["environment"]["camera_heights"],
            camera_widths = training_config["environment"]["camera_widths"],
            camera_depths = training_config["environment"]["camera_depths"],
            camera_segmentations = training_config["environment"]["camera_segmentations"],
            renderer = training_config["environment"]["renderer"],
            renderer_config = training_config["environment"]["renderer_config"],
            use_failsafe_controller = training_config["environment"]["use_failsafe_controller"],
            visualize_failsafe_controller = training_config["environment"]["visualize_failsafe_controller"],
            visualize_pinocchio = training_config["environment"]["visualize_pinocchio"],
            control_sample_time = training_config["environment"]["control_sample_time"],
            human_animation_names = training_config["environment"]["human_animation_names"],
            base_human_pos_offset = training_config["environment"]["base_human_pos_offset"],
            human_animation_freq = training_config["environment"]["human_animation_freq"],
            safe_vel = training_config["environment"]["safe_vel"],
            self_collision_safety = training_config["environment"]["self_collision_safety"],
        )
    )
    ### Environment Wrappers
    if env.spec is None:
        env.spec = struct
    env = TimeLimit(env, max_episode_steps=training_config["algorithm"]["max_ep_len"])
    env = VisualizationWrapper(env)

    now = datetime.now()
    load_episode = -1
    if load_episode in training_config["algorithm"]:
        load_episode = training_config["algorithm"]
        if load_episode >= 0:
            if "run_id" in training_config["algorithm"]:  
                run_id = training_config["algorithm"]["run_id"]
            else:
                print("Please provide a run_id in the config in the algorithm section.")

    last_time_steps = np.ndarray(0)
    
    # Monitor and log the training
    #env = Monitor(env, filename=outdir, info_keywords=("collision", "criticalCollision", "goalReached"))

    
    HerReplayBuffer.add = custom_add
    HerReplayBuffer._sample_transitions = _custom_sample_transitions
    start_episode = 0
    if load_episode==-1:
      ## Defining the "run" for weights and biases
        if use_wandb:
            run = wandb.init(
                project = "safe_human_robot_rl",
                config = training_config,
                sync_tensorboard = True,
                monitor_gym = False,
                save_code = False,
            )
        else:
            run = struct
            run.id = int(np.random.rand(1)*100000)
        ## Initialize the model
        model = SAC(
            "MultiInputPolicy",
            env,
            replay_buffer_class=HerReplayBuffer,
            replay_buffer_kwargs=dict(
              n_sampled_goal=training_config["algorithm"]["k_her_samples"],
              goal_selection_strategy=training_config["algorithm"]["goal_selection_strategy"],
              online_sampling=True,
            ),
            buffer_size=training_config["algorithm"]["replay_size"],
            verbose=1,
            learning_rate=training_config["algorithm"]["lr"], 
            learning_starts=training_config["algorithm"]["start_steps"], 
            batch_size=training_config["algorithm"]["batch_size"], 
            tau=training_config["algorithm"]["tau"], 
            gamma=training_config["algorithm"]["gamma"], 
            train_freq=(training_config["algorithm"]["update_every"], "episode"), 
            gradient_steps=training_config["algorithm"]["gradient_steps"], 
            action_noise=training_config["algorithm"]["action_noise"], 
            optimize_memory_usage=False, 
            ent_coef=training_config["algorithm"]["ent_coef"], 
            target_update_interval=training_config["algorithm"]["target_update_interval"], 
            target_entropy=training_config["algorithm"]["target_entropy"], 
            use_sde=training_config["algorithm"]["use_sde"], 
            sde_sample_freq=training_config["algorithm"]["sde_sample_freq"], 
            use_sde_at_warmup=training_config["algorithm"]["use_sde_at_warmup"], 
            create_eval_env=False, 
            seed=training_config["algorithm"]["seed"], 
            device='auto', 
            _init_setup_model=True,
            policy_kwargs=dict(net_arch=training_config["algorithm"]["hid"]),
            tensorboard_log=f"runs/{run.id}",
            #max_episode_length=training_config["algorithm"]["max_ep_len"]
        )
    else:
        ## Defining the "run" for weights and biases
        if use_wandb:
            run = wandb.init(
              project = "safe_human_robot_rl",
              config = training_config["algorithm"],
              sync_tensorboard = True,
              monitor_gym = False,
              save_code = False,
              resume="must",
              id = run_id
            )
        ## Load the model
        model = SAC.load("{}/model_{}".format(f"models/{run_id}", str(load_episode)), env=env)
        # load it into the loaded_model
        model.load_replay_buffer("{}/replay_buffer.pkl".format(f"models/{run_id}"))
        # now the loaded replay is not empty anymore
        #rospy.loginfo("The loaded_model now has {} transitions in its buffer".format(model.replay_buffer.replay_buffer.size()))
        start_episode = model._episode_num
        model.set_env(env)
        model.env.reset()

    if use_wandb:
        callback = TensorboardCallback(
            eval_env=env,
            gradient_save_freq = 100,
            model_save_path = f"models/{run.id}",
            verbose = 2,
            save_freq = training_config["algorithm"]["save_freq"],
            model_file=f"models/{run.id}",
            start_episode = start_episode,
            additional_log_info_keys = ["goalReached", "collision", "criticalCollision"],
            n_eval_episodes = 0,
            deterministic = True,
            log_interval=training_config["algorithm"]["log_interval"],
          )
    ## Train the agent
    if not training_config["algorithm"]["test_only"]:
      model.learn(
        total_timesteps=training_config["algorithm"]["n_episodes"]*training_config["algorithm"]["max_ep_len"],
        log_interval=training_config["algorithm"]["log_interval"],
        reset_num_timesteps=(load_episode==-1),
        callback=callback)
      model.save(f"models/{run.id}/model_final")
      
    ## Evaluate the agent
    if use_wandb:
        callback = TensorboardCallback(
            eval_env=env,
            verbose = 2,
            additional_log_info_keys = ["goalReached", "collision", "criticalCollision"],
            n_eval_episodes = training_config["algorithm"]["num_test_episodes"],
            deterministic = True,
          )
    ######## Evaluate model #####################
    model.learn(
        total_timesteps=0,
        log_interval=training_config["algorithm"]["log_interval"],
        callback=callback)
    # Close everything
    if use_wandb:
        run.finish()
    env.close()
