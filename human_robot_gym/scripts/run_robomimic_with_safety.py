#!/usr/bin/env python3
"""
Script to run trained robomimic diffusion policies with human-robot-gym safety features.

This script modifies robomimic's run_trained_agent.py to use our custom environments
with Sara-shield safety controller and failsafe collision prevention.

Usage:
    python run_robomimic_with_safety.py --agent /path/to/model.pth \
        --n_rollouts 5 --horizon 100 --seed 0 --render \
        --env_name LiftHumanEnv

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created robomimic safety integration script
"""

import argparse
import json
import sys
import os
import numpy as np
from copy import deepcopy

import torch

# Add robomimic to path
sys.path.insert(0, '/home/jakob/Promotion/code/robomimic')

import robomimic
import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.env_utils as EnvUtils
from robomimic.envs.env_base import EnvBase
from robomimic.envs.wrappers import EnvWrapper
from robomimic.algo import RolloutPolicy

# Human-robot-gym imports
import robosuite as suite
from robosuite.wrappers import GymWrapper
from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper


class SafetyEnvFactory:
    """Factory to create human-robot-gym environments with safety features."""
    
    def __init__(self):
        self.env_mapping = {
            "LiftHumanEnv": "LiftHumanEnv",
            "CanHumanEnv": "CanHumanEnv", 
            "SquareHumanEnv": "SquareHumanEnv",
            "ToolHangHumanEnv": "ToolHangHumanEnv",
        }
    
    def create_safe_env(self, env_name, max_steps=400, render=False):
        """
        Create a human-robot-gym environment with safety features.
        
        Args:
            env_name (str): Name of the environment (e.g., "LiftHumanEnv")
            max_steps (int): Maximum steps per episode
            render (bool): Whether to enable rendering
            
        Returns:
            env: Wrapped environment with safety features
        """
        print(f"Creating safe environment: {env_name}")
        
        # Setup controller configuration (same as working demo)
        failsafe_config_path = file_path_completion(
            "controllers/failsafe_controller/config/failsafe.json"
        )
        robot_config_path = file_path_completion("models/robots/config/panda.json")
        
        # Load the failsafe controller config from file
        with open(failsafe_config_path, 'r') as f:
            failsafe_config = json.load(f)
        
        # Load robot-specific limits
        with open(robot_config_path, 'r') as f:
            robot_config = json.load(f)
        
        # Merge robot limits into failsafe config
        controller_config = {'body_parts': {'right': {}}}
        controller_config['body_parts']['right'] = merge_configs(
            failsafe_config['body_parts']['right'], robot_config
        )
        controller_configs = [controller_config]
        
        # Create environment using the same pattern as working demo
        env = GymWrapper(
            suite.make(
                env_name,
                robots="Panda",
                robot_base_offset=[0, 0, 0],
                use_camera_obs=False,  # do not use pixel observations
                has_offscreen_renderer=False,  # not needed since not using pixel obs
                has_renderer=render,  # make sure we can render to the screen
                render_camera=None,
                renderer="mjviewer" if render else None,
                render_collision_mesh=False,
                reward_shaping=True,  # use dense rewards
                control_freq=5,  # control should happen fast enough so that simulation looks smooth
                horizon=max_steps,
                hard_reset=False,
                controller_configs=controller_configs,
                shield_type="SSM",
                visualize_failsafe_controller=render,  # Enable failsafe visualization only when rendering
                visualize_pinocchio=False,
                base_human_pos_offset=[0.0, 0.0, 0.0],
                verbose=False,  # Reduce verbosity during policy execution
                goal_dist=0.0001,
                human_rand=[0.0, 0.0, 0.0],
                human_animation_names=["SinglePoint/left_right"],
                human_animation_freq=20
            ),
            keys=["object-state", "robot0_proprio-state"],
        )
        
        # Add collision prevention wrapper
        env = CollisionPreventionWrapper(
            env=env, 
            collision_check_fn=env.check_collision_action, 
            replace_type=0
        )
        
        # Add visualization wrapper (same as working demo)
        env = VisualizationWrapper(env)
        
        return env


# Monkey patch the env_from_checkpoint function to use our safety factory
def create_safe_env_from_checkpoint(
    ckpt_path=None, 
    ckpt_dict=None, 
    env_name=None, 
    render=False, 
    render_offscreen=False, 
    verbose=False
):
    """
    Creates an environment using safety features, overriding robomimic's default behavior.
    """
    ckpt_dict = FileUtils.maybe_dict_from_checkpoint(ckpt_path=ckpt_path, ckpt_dict=ckpt_dict)
    
    # Get configuration info from checkpoint
    config, _ = FileUtils.config_from_checkpoint(ckpt_dict=ckpt_dict)
    rollout_horizon = config.experiment.rollout.horizon
    
    # If no env_name provided, try to get from checkpoint metadata
    if env_name is None:
        env_meta = ckpt_dict.get("env_metadata", {})
        env_name = env_meta.get("env_name", "LiftHumanEnv")  # Default fallback
    
    # Map robomimic environment names to our human-robot-gym names
    env_name_mapping = {
        "Lift": "LiftHumanEnv",
        "PickPlaceCan": "CanHumanEnv", 
        "NutAssemblySquare": "SquareHumanEnv",
        "ToolHang": "ToolHangHumanEnv",
    }
    
    # Use mapping if available, otherwise use env_name as-is
    mapped_env_name = env_name_mapping.get(env_name, env_name)
    
    if verbose:
        print(f"Creating safe environment: {mapped_env_name}")
        print(f"Original env_name: {env_name}")
        print(f"Rollout horizon: {rollout_horizon}")
    
    # Create safe environment
    factory = SafetyEnvFactory()
    env = factory.create_safe_env(
        env_name=mapped_env_name,
        max_steps=rollout_horizon,
        render=render
    )
    
    # Apply any additional wrappers from robomimic config
    env = EnvUtils.wrap_env_from_config(env, config=config)
    
    if verbose:
        print("============= Created Safe Environment =============")
        print(env)
    
    return env, ckpt_dict


def rollout(policy, env, horizon, render=False, video_writer=None, video_skip=5, return_obs=False, camera_names=None):
    """
    Rollout function adapted from robomimic's run_trained_agent.py
    """
    assert isinstance(env, (EnvBase, EnvWrapper, GymWrapper, VisualizationWrapper, CollisionPreventionWrapper))
    assert isinstance(policy, RolloutPolicy)
    assert not (render and (video_writer is not None))

    policy.start_episode()
    obs = env.reset()
    
    # Try to get state dict - some environments may not support this
    try:
        state_dict = env.get_state()
        # hack that is necessary for robosuite tasks for deterministic action playback
        obs = env.reset_to(state_dict)
    except (AttributeError, NotImplementedError):
        state_dict = {"states": None}
        print("Warning: Environment doesn't support get_state() - using None")

    results = {}
    video_count = 0  # video frame counter
    total_reward = 0.
    safety_interventions = 0
    collisions = 0
    
    traj = dict(actions=[], rewards=[], dones=[], states=[], initial_state_dict=state_dict)
    if return_obs:
        # store observations too
        traj.update(dict(obs=[], next_obs=[]))
        
    try:
        for step_i in range(horizon):

            # get action from policy
            act = policy(ob=obs)

            # play action
            next_obs, r, done, info = env.step(act)

            # compute reward
            total_reward += r
            
            # Check for success - handle different info structures
            if hasattr(env, 'is_success'):
                success = env.is_success()["task"]
            elif isinstance(info, dict) and "n_goal_reached" in info:
                success = info.get("n_goal_reached", 0) > 0
            else:
                success = False

            # Track safety metrics
            if isinstance(info, dict):
                if info.get("collision", False):
                    collisions += 1
                if "failsafe_interventions" in info:
                    safety_interventions = info["failsafe_interventions"]

            # visualization
            if render:
                if hasattr(env, 'render'):
                    try:
                        if camera_names and len(camera_names) > 0:
                            env.render(mode="human", camera_name=camera_names[0])
                        else:
                            env.render(mode="human")
                    except:
                        # Fallback rendering
                        env.render()
                        
            if video_writer is not None:
                if video_count % video_skip == 0:
                    video_img = []
                    for cam_name in camera_names:
                        try:
                            video_img.append(env.render(mode="rgb_array", height=512, width=512, camera_name=cam_name))
                        except:
                            # Fallback if camera rendering fails
                            video_img.append(np.zeros((512, 512, 3), dtype=np.uint8))
                    if video_img:
                        video_img = np.concatenate(video_img, axis=1) # concatenate horizontally
                        video_writer.append_data(video_img)
                video_count += 1

            # collect transition
            traj["actions"].append(act)
            traj["rewards"].append(r)
            traj["dones"].append(done)
            if state_dict["states"] is not None:
                try:
                    current_state = env.get_state()
                    traj["states"].append(current_state["states"])
                except:
                    traj["states"].append(state_dict["states"])
            else:
                traj["states"].append(None)
                
            if return_obs:
                traj["obs"].append(obs)
                traj["next_obs"].append(next_obs)

            # break if done or if success
            if done or success:
                break

            # update for next iter
            obs = deepcopy(next_obs)
            try:
                state_dict = env.get_state()
            except:
                pass

    except Exception as e:
        print("WARNING: got rollout exception {}".format(e))

    # Include safety metrics in stats
    stats = dict(
        Return=total_reward, 
        Horizon=(step_i + 1), 
        Success_Rate=float(success),
        Safety_Interventions=safety_interventions,
        Collisions=collisions
    )

    if return_obs:
        # convert list of dict to dict of list for obs dictionaries (for convenient writes to hdf5 dataset)
        from robomimic.utils.tensor_utils import list_of_flat_dict_to_dict_of_list
        traj["obs"] = list_of_flat_dict_to_dict_of_list(traj["obs"])
        traj["next_obs"] = list_of_flat_dict_to_dict_of_list(traj["next_obs"])

    # list to numpy array
    for k in traj:
        if k == "initial_state_dict":
            continue
        if isinstance(traj[k], dict):
            for kp in traj[k]:
                if traj[k][kp] is not None:
                    traj[k][kp] = np.array(traj[k][kp])
        else:
            if traj[k] and traj[k][0] is not None:
                traj[k] = np.array(traj[k])

    return stats, traj


def run_trained_agent_with_safety(args):
    """
    Main function adapted from robomimic's run_trained_agent.py
    """
    # some arg checking
    write_video = (args.video_path is not None)
    assert not (args.render and write_video) # either on-screen or video but not both
    if args.render:
        # on-screen rendering can only support one camera
        assert len(args.camera_names) == 1

    # relative path to agent
    ckpt_path = args.agent

    # device
    device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    # restore policy
    policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=ckpt_path, device=device, verbose=True)

    # read rollout settings
    rollout_num_episodes = args.n_rollouts
    rollout_horizon = args.horizon
    if rollout_horizon is None:
        # read horizon from config
        config, _ = FileUtils.config_from_checkpoint(ckpt_dict=ckpt_dict)
        rollout_horizon = config.experiment.rollout.horizon

    # create environment using our safety factory
    env, _ = create_safe_env_from_checkpoint(
        ckpt_dict=ckpt_dict, 
        env_name=args.env_name, 
        render=args.render, 
        render_offscreen=(args.video_path is not None), 
        verbose=True,
    )

    # maybe set seed
    if args.seed is not None:
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)

    # maybe create video writer
    video_writer = None
    if write_video:
        import imageio
        video_writer = imageio.get_writer(args.video_path, fps=20)

    rollout_stats = []
    for i in range(rollout_num_episodes):
        print(f"\n=== Rollout {i+1}/{rollout_num_episodes} ===")
        stats, traj = rollout(
            policy=policy, 
            env=env, 
            horizon=rollout_horizon, 
            render=args.render, 
            video_writer=video_writer, 
            video_skip=args.video_skip, 
            return_obs=False,  # Don't store obs to save memory
            camera_names=args.camera_names,
        )
        rollout_stats.append(stats)
        
        print(f"Episode {i+1} Stats:")
        print(f"  Return: {stats['Return']:.3f}")
        print(f"  Horizon: {stats['Horizon']}")
        print(f"  Success: {stats['Success_Rate']}")
        print(f"  Safety Interventions: {stats['Safety_Interventions']}")
        print(f"  Collisions: {stats['Collisions']}")

    # Compute averages
    from robomimic.utils.tensor_utils import list_of_flat_dict_to_dict_of_list
    rollout_stats = list_of_flat_dict_to_dict_of_list(rollout_stats)
    avg_rollout_stats = { k : np.mean(rollout_stats[k]) for k in rollout_stats }
    avg_rollout_stats["Num_Success"] = np.sum(rollout_stats["Success_Rate"])
    
    print("\n" + "="*50)
    print("FINAL RESULTS")
    print("="*50)
    print("Average Rollout Stats:")
    print(json.dumps(avg_rollout_stats, indent=4))
    
    print(f"\nSafety Summary:")
    print(f"  Total Safety Interventions: {np.sum(rollout_stats['Safety_Interventions'])}")
    print(f"  Total Collisions: {np.sum(rollout_stats['Collisions'])}")
    print(f"  Average Safety Interventions per Episode: {avg_rollout_stats['Safety_Interventions']:.2f}")
    print(f"  Average Collisions per Episode: {avg_rollout_stats['Collisions']:.2f}")

    if write_video:
        video_writer.close()
        print(f"\nVideo saved to: {args.video_path}")

    # Close environment
    env.close()
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run trained robomimic agents with human-robot-gym safety")

    # Path to trained model
    parser.add_argument(
        "--agent",
        type=str,
        required=True,
        help="path to saved checkpoint pth file",
    )

    # number of rollouts
    parser.add_argument(
        "--n_rollouts",
        type=int,
        default=5,
        help="number of rollouts",
    )

    # maximum horizon of rollout, to override the one stored in the model checkpoint
    parser.add_argument(
        "--horizon",
        type=int,
        default=None,
        help="(optional) override maximum horizon of rollout from the one in the checkpoint",
    )

    # Env Name (to override the one stored in model checkpoint)
    parser.add_argument(
        "--env_name",
        type=str,
        default=None,
        help="(optional) environment name to use (e.g., LiftHumanEnv, CanHumanEnv, SquareHumanEnv, ToolHangHumanEnv)",
    )

    # Whether to render rollouts to screen
    parser.add_argument(
        "--render",
        action='store_true',
        help="on-screen rendering",
    )

    # Dump a video of the rollouts to the specified path
    parser.add_argument(
        "--video_path",
        type=str,
        default=None,
        help="(optional) render rollouts to this video file path",
    )

    # How often to write video frames during the rollout
    parser.add_argument(
        "--video_skip",
        type=int,
        default=5,
        help="render frames to video every n steps",
    )

    # camera names to render
    parser.add_argument(
        "--camera_names",
        type=str,
        nargs='+',
        default=["agentview"],
        help="(optional) camera name(s) to use for rendering on-screen or to video",
    )

    # for seeding before starting rollouts
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="(optional) set seed for rollouts",
    )

    args = parser.parse_args()
    run_trained_agent_with_safety(args)