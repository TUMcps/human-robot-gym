"""Gym environment registrations for robomimic tasks with human simulation and safety.

This module registers robomimic environments adapted to work with human-robot-gym
safety features including sara-shield, failsafe controller, and collision detection.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    XX.XX.XX JT Created robomimic gym environment registrations
"""

from gymnasium.envs.registration import register

# Robomimic Lift Task with Human Safety
register(
    id="RobomimicLiftHuman-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "LiftHumanEnv",
        "robots": "Panda",
        "id": "RobomimicLiftHuman-v0",
    },
    max_episode_steps=400,
)

# Robomimic Can Task with Human Safety
register(
    id="RobomimicCanHuman-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "CanHumanEnv",
        "robots": "Panda",
        "id": "RobomimicCanHuman-v0",
    },
    max_episode_steps=400,
)

# Robomimic Square Task with Human Safety
register(
    id="RobomimicSquareHuman-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "SquareHumanEnv",
        "robots": "Panda",
        "id": "RobomimicSquareHuman-v0",
    },
    max_episode_steps=400,
)

# Robomimic Transport Task with Human Safety
register(
    id="RobomimicTransportHuman-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "TransportHumanEnv",
        "robots": "Panda",
        "id": "RobomimicTransportHuman-v0",
    },
    max_episode_steps=700,  # Longer horizon for transport task
)

# Robomimic Tool Hang Task with Human Safety
register(
    id="RobomimicToolHangHuman-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "ToolHangHumanEnv",
        "robots": "Panda",
        "id": "RobomimicToolHangHuman-v0",
    },
    max_episode_steps=700,  # Longer horizon for tool hang task
)

# Additional variants with different robots
register(
    id="RobomimicLiftHumanSchunk-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "LiftHumanEnv",
        "robots": "Schunk",
        "id": "RobomimicLiftHumanSchunk-v0",
    },
    max_episode_steps=400,
)

register(
    id="RobomimicCanHumanSchunk-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "CanHumanEnv",
        "robots": "Schunk",
        "id": "RobomimicCanHumanSchunk-v0",
    },
    max_episode_steps=400,
)

# Environment variants with different safety settings
register(
    id="RobomimicLiftHumanNoShield-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "LiftHumanEnv",
        "robots": "Panda",
        "id": "RobomimicLiftHumanNoShield-v0",
        "shield_type": "OFF",  # Disable safety shield for comparison
    },
    max_episode_steps=400,
)

register(
    id="RobomimicCanHumanPFL-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "CanHumanEnv",
        "robots": "Panda",
        "id": "RobomimicCanHumanPFL-v0",
        "shield_type": "PFL",  # Use PFL safety mode
    },
    max_episode_steps=400,
)

# High-frequency human animation variants
register(
    id="RobomimicLiftHumanFast-v0",
    entry_point="human_robot_gym.environments.gym_envs.make_gym:make_gym",
    kwargs={
        "env": "LiftHumanEnv",
        "robots": "Panda",
        "id": "RobomimicLiftHumanFast-v0",
        "human_animation_freq": 240,  # Faster human motion
        "n_animations_sampled_per_100_steps": 10,  # More animation variety
    },
    max_episode_steps=400,
)
