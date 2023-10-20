#!/bin/bash
./build_docker_train.sh user
./run_docker_train.sh user "human_robot_gym/training/icra_2024_run_experiments.sh"
