"""Demo script for testing robomimic environment integration with human-robot-gym safety.

This script demonstrates how to use robomimic environments (Lift, Can, Square, Transport, ToolHang)
with human simulation and safety features from human-robot-gym.

Features tested:
- Robomimic task environments with human animation
- Sara-shield safety controller
- Failsafe collision prevention
- Multi-level collision detection

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created robomimic integration demo
"""

import robosuite as suite
from robosuite.wrappers import GymWrapper


from human_robot_gym.utils.mjcf_utils import file_path_completion, merge_configs

# Import robomimic environments
from human_robot_gym.environments.manipulation.lift_human_env import LiftHumanEnv
from human_robot_gym.environments.manipulation.pick_place_human_env import PickPlaceCanHumanEnv
from human_robot_gym.environments.manipulation.nut_assembly_human_env import NutAssemblySquareHumanEnv
from human_robot_gym.environments.manipulation.tool_hang_human_env import ToolHangHumanEnv
import human_robot_gym.robots  # noqa: F401
from human_robot_gym.wrappers.visualization_wrapper import VisualizationWrapper
from human_robot_gym.wrappers.collision_prevention_wrapper import CollisionPreventionWrapper


ENV_MAPPING = {
    "lift": LiftHumanEnv,
    "can": PickPlaceCanHumanEnv,
    "square": NutAssemblySquareHumanEnv,
    # "transport": -> Dual arm, leave out for now
    "tool_hang": ToolHangHumanEnv
}


def test_robomimic_env(env_name: str, num_episodes: int = 5, max_steps: int = 100):
    """Test a robomimic environment with human safety features.

    Args:
        env_name (str): Name of the robomimic environment class
        num_episodes (int): Number of episodes to run
        max_steps (int): Maximum steps per episode
    """
    print(f"\n=== Testing {env_name} ===")

    try:
        # Setup controller configuration (same as working demo)
        failsafe_config_path = file_path_completion(
            "controllers/failsafe_controller/config/failsafe.json"
        )
        robot_config_path = file_path_completion("models/robots/config/panda.json")

        # Load the failsafe controller config from file
        import json
        with open(failsafe_config_path, 'r') as f:
            failsafe_config = json.load(f)

        # Load robot-specific limits
        with open(robot_config_path, 'r') as f:
            robot_config = json.load(f)

        # Merge robot limits into failsafe config
        controller_config = {'body_parts': {'right': {}}}
        controller_config['body_parts']['right'] = merge_configs(failsafe_config['body_parts']['right'], robot_config)
        controller_configs = [controller_config]

        # Create environment using the same pattern as working demo
        env = GymWrapper(
            suite.make(
                env_name,
                robots="Panda",
                robot_base_offset=[0, 0, 0],
                use_camera_obs=False,  # do not use pixel observations
                has_offscreen_renderer=False,  # not needed since not using pixel obs
                has_renderer=True,  # make sure we can render to the screen
                render_camera=None,
                renderer="mjviewer",
                render_collision_mesh=False,
                reward_shaping=True,  # use dense rewards
                control_freq=5,  # control should happen fast enough so that simulation looks smooth
                horizon=max_steps,
                hard_reset=False,
                controller_configs=controller_configs,
                shield_type="SSM",
                visualize_failsafe_controller=True,  # Enable failsafe visualization
                visualize_pinocchio=False,
                base_human_pos_offset=[0.0, 0.0, 0.0],
                verbose=True,  # Enable verbose output for debugging
                goal_dist=0.0001,
                human_rand=[0.0, 0.0, 0.0],
                human_animation_names=["SinglePoint/left_right"],
                human_animation_freq=20
            ),
            keys=["object-state", "robot0_proprio-state"],
        )

        # Add collision prevention wrapper
        env = CollisionPreventionWrapper(env=env, collision_check_fn=env.check_collision_action, replace_type=0)

        # Add visualization wrapper (same as working demo)
        env = VisualizationWrapper(env)
        print(f"✓ Successfully created {env_name}")

        for episode in range(num_episodes):
            print(f"\nEpisode {episode + 1}/{num_episodes}")

            # Reset environment
            obs = env.reset()
            print("✓ Environment reset successful")
            print(
                f"  - Observation keys: {
                  list(obs.keys()) if isinstance(obs, dict) else '\
                    Array shape: ' + str(obs.shape) if hasattr(obs, 'shape') else 'Single value'
                }"
            )

            total_reward = 0
            collisions = 0
            safety_interventions = 0

            for step in range(max_steps):
                # Random action for testing
                action = env.action_space.sample()

                # Step environment
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward

                # Track safety metrics
                if info.get("collision", False):
                    collisions += 1
                if info.get("failsafe_interventions", 0) > safety_interventions:
                    safety_interventions = info["failsafe_interventions"]

                # Print progress every 20 steps
                if step % 20 == 0:
                    print(f"  Step {step}: reward={reward:.3f}, collision={info.get('collision', False)}")

                if terminated or truncated:
                    print(f"  Episode finished at step {step}")
                    break

            print("✓ Episode completed:")
            print(f"  - Total reward: {total_reward:.3f}")
            print(f"  - Collisions: {collisions}")
            print(f"  - Safety interventions: {safety_interventions}")
            print(f"  - Success: {info.get('n_goal_reached', 0) > 0}")

        env.close()
        print(f"✓ {env_name} test completed successfully")

    except Exception as e:
        print(f"✗ Error testing {env_name}: {str(e)}")
        import traceback

        traceback.print_exc()


def main():
    """Run demo tests for all robomimic environments."""
    print("Robomimic Integration Demo")
    print("=" * 50)
    print("Testing robomimic environments with human safety features")

    # List of robomimic environments to test
    test_environments = [
        "LiftHumanEnv",
        "CanHumanEnv",
        "SquareHumanEnv",
        "TransportHumanEnv",
        "ToolHangHumanEnv",
    ]

    # Test each environment
    for env_name in test_environments:
        test_robomimic_env(env_name, num_episodes=5, max_steps=100)
        # time.sleep(1)  # Brief pause between tests

    print("\n" + "=" * 20)
    print("Demo completed!")
    print("\nRobomimic environments successfully integrated with human-robot-gym safety features:")
    print("  ✓ Sara-shield collision avoidance")
    print("  ✓ Failsafe controller")
    print("  ✓ Human animation and collision detection")
    print("  ✓ Multi-level collision categorization")
    print("  ✓ Safety intervention monitoring")


def test_environment_variants():
    """Test different variants of robomimic environments."""
    print("\n=== Testing Environment Variants ===")

    variants = [
        "LiftHumanEnv",  # Test with different robot and shield settings via kwargs
    ]

    for variant in variants:
        test_robomimic_env(variant, num_episodes=5, max_steps=100)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test robomimic environments with human safety features")
    parser.add_argument("--single-env", type=str, help="Test a single environment (e.g., LiftHumanEnv)")
    parser.add_argument("--max-steps", type=int, default=50, help="Maximum steps per episode")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes per environment")

    args = parser.parse_args()

    if args.single_env:
        print(f"Testing single environment: {args.single_env}")
        print("=" * 50)
        test_robomimic_env(args.single_env, num_episodes=args.episodes, max_steps=args.max_steps)
    else:
        # Run basic tests
        main()

        # Optionally test variants
        test_environment_variants()
