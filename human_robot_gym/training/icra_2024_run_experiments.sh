#! /bin/bash

# Run environment evaluations
human_robot_gym/training/icra_2024_environment_evaluation.sh R HumanReach 100 1000000 100 8 800
human_robot_gym/training/icra_2024_environment_evaluation.sh PP PickAndPlace 500 3000000 1000 8 8000
human_robot_gym/training/icra_2024_environment_evaluation.sh CL CollaborativeLifting 100 1000000 5000 8 8000
human_robot_gym/training/icra_2024_environment_evaluation.sh RHH RobotHumanHandover 500 3000000 1000 8 8000
human_robot_gym/training/icra_2024_environment_evaluation.sh HRH HumanRobotHandover 500 3000000 1000 8 8000
human_robot_gym/training/icra_2024_environment_evaluation.sh CS CollaborativeStacking 500 3000000 3000 8 8000

# Run ablation study on overfitting on human animations
human_robot_gym/training/icra_2024_animation_overfitting.sh
