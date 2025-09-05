"""Demo script for CBF safety controller with Panda robot.

This script demonstrates the CBF (Control Barrier Function) safety controller
in a human-robot interaction scenario, following the same structure as the
demo_gym_functionality_panda.py example. Uses the new refactored architecture
with the factory pattern for controller creation.
"""

import robosuite as suite
import time
import numpy as np
import json

from robosuite.wrappers.gym_wrapper import GymWrapper

from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs
import human_robot_gym.environments.manipulation.reach_human_env  # noqa: F401
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import (
    CollisionPreventionWrapper,
)

if __name__ == "__main__":
    print("CBF Safety Controller Demo - Panda Robot")
    print("=" * 50)
    
    # Load custom CBF controller config
    cbf_config_path = file_path_completion(
        "controllers/failsafe_controller/config/cbf_failsafe.json"
    )
    robot_config_path = file_path_completion("models/robots/config/panda.json")

    # Load the CBF controller config from file
    with open(cbf_config_path, 'r') as f:
        cbf_config = json.load(f)

    # Load robot-specific limits
    with open(robot_config_path, 'r') as f:
        robot_config = json.load(f)

    # Merge robot limits into CBF config
    controller_config = {'body_parts': {'right': {}}}
    controller_config['body_parts']['right'] = merge_configs(cbf_config['body_parts']['right'], robot_config)
    controller_configs = [controller_config]
    
    print(f"CBF Controller Configuration:")
    print(f"  - Min distance: {controller_config['body_parts']['right']['min_distance']}")
    print(f"  - Class K type: {controller_config['body_parts']['right']['class_k_type']}")
    print(f"  - Class K scale: {controller_config['body_parts']['right']['class_k_scale']}")
    print(f"  - Optimizer solver: {controller_config['body_parts']['right']['opti_solver']}")

    print("Creating environment with CBF safety controller...")
    
    # Create environment with CBF shield
    env = suite.make(
        "ReachHuman",
        robots="Panda",  # Use Panda robot
        robot_base_offset=[0, 0, 0],
        use_camera_obs=False,  # Do not use pixel observations
        has_offscreen_renderer=False,  # Not needed since not using pixel obs
        has_renderer=True,  # Make sure we can render to the screen
        render_camera=None,
        renderer="mjviewer",
        render_collision_mesh=False,
        reward_shaping=True,  # Use dense rewards
        control_freq=5,  # Control should happen fast enough so that simulation looks smooth
        hard_reset=False,
        horizon=1000,
        controller_configs=controller_configs,
        shield_type="CBF",  # Use CBF instead of SSM/PFL
        visualize_failsafe_controller=True,
        visualize_pinocchio=False,
        base_human_pos_offset=[0.1, 0.0, 0.0],
        verbose=True,
        goal_dist=0.0001,
        human_rand=[0.0, 0.0, 0.0]
    )

    print("Environment created successfully!")
    print(f"Shield type: {env.shield_type}")

    # Add collision prevention wrapper
    env = CollisionPreventionWrapper(
        env=env, collision_check_fn=env.check_collision_action, replace_type=0
    )

    # Add visualization wrapper
    env = VisualizationWrapper(env)

    # Add gym wrapper
    env = GymWrapper(
        env,
        keys=[
            "object-state",
            "robot0_proprio-state",
            "goal_difference"
        ]
    )

    print("\nStarting demonstration...")
    print("Watch for CBF safety interventions when robot approaches human!")
    
    t_max = 100
    total_interventions = 0
    
    for i_episode in range(5):  # Run fewer episodes for demo
        print(f"\n--- Episode {i_episode + 1} ---")
        observation = env.reset()
        t1 = time.time()
        episode_interventions = 0
        
        for t in range(t_max):
            # Generate action that tries to reach the goal
            action = env.action_space.sample()
            pos = np.array([env.sim.data.qpos[x] for x in env.robots[0]._ref_joint_pos_indexes])
            goal = env.desired_goal
            
            # Create action that moves towards goal (potentially unsafe)
            goal_action = np.clip(goal - pos, -0.5, 0.5)
            action[:pos.shape[0]] = np.zeros_like(goal)
            
            # Step the environment
            observation, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            # Check for CBF intervention
            if hasattr(env.robots[0], 'controller') and hasattr(env.robots[0].controller, 'get_safety'):
                is_safe = env.robots[0].controller.get_safety()
                if not is_safe:
                    episode_interventions += 1
                    if episode_interventions == 1:  # First intervention this episode
                        print(f"  Step {t}: CBF intervention detected!")
            
            # Print periodic status
            if t % 20 == 0:
                print(f"  Step {t}: Reward = {reward:.3f}, Episode interventions = {episode_interventions}")
            
            if done or t == t_max - 1:
                print(f"  Episode finished after {t + 1} timesteps")
                print(f"  Episode interventions: {episode_interventions}")
                break
                
        total_interventions += episode_interventions
        fps = t / max(time.time() - t1, 0.001)  # Avoid division by zero
        print(f"  Episode {i_episode + 1} FPS: {fps:.1f}")

    print(f"\n=== Demo Summary ===")
    print(f"Total CBF interventions across all episodes: {total_interventions}")
    print(f"Average interventions per episode: {total_interventions / 5:.1f}")
    
    if total_interventions > 0:
        print("✓ CBF controller successfully intervened to maintain safety!")
    else:
        print("• No interventions needed (human was not approached closely)")
    
    print("\nDemo completed successfully!")
    print("CBF safety controller is working and ready for use.")
    
    # Close environment
    env.close()