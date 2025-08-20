#!/bin/bash

# This script can be used to evaluate how much trained policies overfit to limited human animation datasets.
# It performs a cross evaluation by generating 5 different 7-2 splits of the animation dataset.
# Each split is used to train policies on 5 random seeds and evaluate it on the training and test episodes of the split.
# The expert policy is evaluated on the training and test episodes of each split as well.
# The evaluation results are stored in .csv files at csv/evaluation/<env_long>_animation_overfitting.
# The script can be executed with the following command:
#   ./human_robot_gym/training/icra_2024_animation_overfitting.sh (<env>) (<env_long>) (<method>) (<n_dataset_episodes>) (<n_training_steps>) (<horizon>) (<n_training_envs>) (<training_log_interval>) (<run_type>) (<n_test_episodes>)
# Args:
#   <env>: acronym of the environment:
#       R for HumanReach
#       PP for PickPlaceHuman
#       CL for CollaborativeLifting
#       RHH for RobotHumanHandover
#       HRH for HumanRobotHandover
#       CS for CollaborativeStacking
#   <env_long>: name of the environment, name of the subfolder in which the environment is stored
#   <method>: acronym of the RL method used for training:
#       SAC for soft actor-critic
#       RSI for reference state initialization
#       AIR for action-based expert imitation reward
#       SIR for state-based expert imitation reward
#   <n_dataset_episodes>: number of episodes to generate for the dataset
#   <n_training_steps>: number of training steps
#   <horizon>: maximum number of steps per episode
#   <n_training_envs>: number of parallel environments used for training
#   <training_log_interval>: interval in steps at which training logs are stored
#   <run_type>: type of the run:
#       wandb for logging with weights and biases (https://wandb.ai/)
#       tensorboard for tensorboard logging
#       debug for debugging (no logging, no models stored)
#   <n_test_episodes>: number of episodes to evaluate the models on
# By default, the study is performed on the CollaborativeLifting environment.
# Using another environments requires adjusting the dataset splits in the script.
env=${1:-"CL"}
env_long=${2:-"CollaborativeLifting"}
method=${3:-"SAC"}
n_dataset_episodes=${4:-"100"}
n_training_steps=${5:-"1000000"}
horizon=${6:-"5000"}
n_training_envs=${7:-"8"}
training_log_interval=${8:-"10000"}
run_type=${9:-"tensorboard"}
n_test_episodes=${10:-"20"}
model_save_interval=50000
max_eval_threads=50

project_name=${env_long}_animation_overfitting

evaluation_data_csv_folder="csv/evaluation/${project_name}"

seeds=(0 1 2 3 4)

dataset_name_0=${env_long}-split-0
dataset_name_1=${env_long}-split-1
dataset_name_2=${env_long}-split-2
dataset_name_3=${env_long}-split-3
dataset_name_4=${env_long}-split-4



# =============================================================================
# ======================== Dataset Split Definitions ==========================
# =============================================================================

train_0="[${env_long}/2,${env_long}/3,${env_long}/4,${env_long}/5,${env_long}/6,${env_long}/9,${env_long}/10]"
test_0="[${env_long}/7,${env_long}/8]"

train_1="[${env_long}/3,${env_long}/4,${env_long}/6,${env_long}/7,${env_long}/8,${env_long}/9,${env_long}/10]"
test_1="[${env_long}/2,${env_long}/5]"

train_2="[${env_long}/2,${env_long}/4,${env_long}/5,${env_long}/6,${env_long}/8,${env_long}/9,${env_long}/10]"
test_2="[${env_long}/3,${env_long}/7]"

train_3="[${env_long}/3,${env_long}/4,${env_long}/5,${env_long}/6,${env_long}/7,${env_long}/8,${env_long}/9]"
test_3="[${env_long}/2,${env_long}/10]"

train_4="[${env_long}/2,${env_long}/3,${env_long}/5,${env_long}/6,${env_long}/7,${env_long}/8,${env_long}/10]"
test_4="[${env_long}/4,${env_long}/9]"


# =============================================================================
# ======================== Function Definitions ===============================
# =============================================================================

# Join an array with a separator.
# Usage:
#   join_by <sep> <array>
# Example:
#   join_by , (a b c)
#   -> a,b,c
join_by () {
    local IFS="$1"
    shift
    echo "$*"
}

# Print a line in green color to the console
# Usage:
#   print_green <msg>
# Args:
#   <msg>: string to print
print_green () {
    printf '%s%s%s\n' $(tput setaf 2) "$1" $(tput sgr0)
}

# Cleanup any existing data to avoid issues
# Usage:
#   cleanup_existing_data
cleanup_existing_data () {
    for dataset_name in ${dataset_name_0} ${dataset_name_1} ${dataset_name_2} ${dataset_name_3} ${dataset_name_4}
    do
        if [ -d "datasets/${dataset_name}" ]; then
            echo "Overwriting existing dataset"
            rm -r datasets/${dataset_name}
        fi
    done

    if [ -d ${evaluation_data_csv_folder} ]; then
        echo "Overwriting existing evaluation csv data at ${evaluation_data_csv_folder}"
        rm -r ${evaluation_data_csv_folder}
    fi

    if [ -d "runs/${project_name}" ]; then
        echo "Overwriting existing training data at runs/${project_name}"
        rm -r runs/${project_name}
    fi

    if [ -d "models/${project_name}" ]; then
        echo "Overwriting existing models at models/${project_name}"
        rm -r models/${project_name}
    fi
}

# Generate a expert dataset
# Usage:
#   generate_dataset <dataset_name> <human_animations>
# Args:
#   <dataset_name>: name of the dataset, name of the subfolder in which the dataset is stored
#   <human_animations>: names of the human animation files
generate_dataset () {
    local dataset_name=$1
    local human_animations=$2
    (
        set -o xtrace
        python human_robot_gym/training/create_expert_dataset.py \
            -cp config_icra_2024/environment_evaluation/dataset_creation \
            -cn ${env} \
            dataset_name=${dataset_name} n_episodes=${n_dataset_episodes} \
            environment.horizon=${horizon} environment.human_animation_names=${human_animations} environment.verbose=False
    )
}

# Execute a training run
# Usage:
#   train <dataset_name> <animations> (<group>)
# Args:
#   <dataset_name>: name of the dataset, name of the subfolder in which the dataset is stored
#   <animations>: names of the human animation files
#   <group>: name of the run group, name of the subfolders in which the runs and models are stored, run group for wandb.
#       Defaults to <dataset_name>
train () {
    local dataset_name=$1
    local human_animations=$2
    local group=${3:-"${dataset_name}"}
    local run_seed_arg=$(join_by , ${seeds[@]})
    (
        set -o xtrace
        python human_robot_gym/training/train_SB3.py --multirun \
            -cp config_icra_2024/environment_evaluation/training \
            -cn ${env}-${method} \
            hydra/launcher=ray \
            run.type=tensorboard run.n_steps=${n_training_steps} \
            wandb_run.project=${project_name} wandb_run.group=${group} \
            run.seed=${run_seed_arg} run.n_envs=${n_training_envs} run.dataset_name=${dataset_name} "run.log_interval=[${training_log_interval},'step']" \
            run.type=${run_type} run.save_freq=${model_save_interval} \
            environment.horizon=${horizon} environment.human_animation_names=${human_animations} environment.verbose=False
    )
}

# Evaluate models from snapshots during training and store statistics into .csv files
# Usage:
#   evaluate <training_dataset_name> <training_group> <evaluation_group> <human_animations>
# Args:
#   <training_dataset_name>: name of the dataset that was used for training, name of the subfolder in which the dataset is stored
#   <training_group>: name of the training run group, name of the subfolders in which the models are stored.
#   <evaluation_group>: name of the evaluation run group, name of the subfolders in which the evaluation csv files should be stored.
#   <human_animations>: names of the human animation files
evaluate () {
    local training_dataset_name=$1
    local training_group=$2
    local evaluation_group=$3
    local human_animations=$4
    local run_ids=[$(join_by , $(for seed in ${seeds[@]}; do echo ${project_name}/${training_group}/run_${seed}; done))]
    (
        set -o xtrace
        python human_robot_gym/training/evaluate_models_to_csv_SB3.py \
            -cp config_icra_2024/environment_evaluation/evaluation \
            -cn ${env} \
            group_name=${project_name}/${evaluation_group} max_parallel_runs=${max_eval_threads} \
            run.n_steps=${n_training_steps} run.save_freq=${model_save_interval} "run.id=${run_ids}" \
            run.load_step=final run.n_test_episodes=${n_test_episodes} \
            environment.horizon=${horizon} environment.human_animation_names=${human_animations} \
            wrappers.dataset_obs_norm.dataset_name=${training_dataset_name} environment.verbose=False
    )
}

# Evaluate the expert policy on the test episodes and store statistics into a .csv file
# Usage:
#   evaluate_expert <training_dataset_name> <group_name> <human_animations>
# Args:
#   <training_dataset_name>: name of the dataset that was used for training, name of the subfolder in which the dataset is stored
#   <group_name>: name of the run group, name of the subfolders in which the models and csv files are stored.
#   <human_animations>: names of the human animation files
evaluate_expert () {
    local training_dataset_name=$1
    local group_name=$2
    local human_animations=$3
    (
        set -o xtrace
        python human_robot_gym/training/evaluate_models_to_csv_SB3.py \
            -cp config_icra_2024/environment_evaluation/evaluation \
            -cn ${env} \
            "run.id=null" \
            group_name=${project_name}/${group_name} \
            run.load_step=final run.n_test_episodes=${n_test_episodes} \
            environment.horizon=${horizon} environment.human_animation_names=${human_animations} \
            wrappers.dataset_obs_norm.dataset_name=${training_dataset_name} environment.verbose=False
    )
}


# =============================================================================
# ======================== Cleanup of Existing Data ===========================
# =============================================================================

# Comment if you do not want to override any data
cleanup_existing_data


# =============================================================================
# ======================== Dataset Generation =================================
# =============================================================================

print_green "Generating datasets..."

# Generate datasets in parallel
generate_dataset ${dataset_name_0} ${train_0} &
generate_dataset ${dataset_name_1} ${train_1} &
generate_dataset ${dataset_name_2} ${train_2} &
generate_dataset ${dataset_name_3} ${train_3} &
generate_dataset ${dataset_name_4} ${train_4} &

wait


# =============================================================================
# ======================== Run Training =======================================
# =============================================================================

print_green "Datasets created, proceeding with training..."

train ${dataset_name_0} ${train_0}
train ${dataset_name_1} ${train_1}
train ${dataset_name_2} ${train_2}
train ${dataset_name_3} ${train_3}
train ${dataset_name_4} ${train_4}

if [ ${run_type} == debug ]
then
    print_green "Debug training done."
    # Terminate early, no log data or models to evaluate stored
    exit 0
fi


# =============================================================================
# ======================== Evaluate Model Snapshots ===========================
# =============================================================================

print_green "Training done, proceeding with evaluation..."

mkdir -p ${evaluation_data_csv_folder}

# Evaluate models on training episodes
evaluate ${dataset_name_0} ${dataset_name_0} ${dataset_name_0}-${method}-on-training-set ${train_0}
evaluate ${dataset_name_1} ${dataset_name_1} ${dataset_name_1}-${method}-on-training-set ${train_1}
evaluate ${dataset_name_2} ${dataset_name_2} ${dataset_name_2}-${method}-on-training-set ${train_2}
evaluate ${dataset_name_3} ${dataset_name_3} ${dataset_name_3}-${method}-on-training-set ${train_3}
evaluate ${dataset_name_4} ${dataset_name_4} ${dataset_name_4}-${method}-on-training-set ${train_4}

# Evaluate models on test episodes
evaluate ${dataset_name_0} ${dataset_name_0} ${dataset_name_0}-${method}-on-test-set ${test_0}
evaluate ${dataset_name_1} ${dataset_name_1} ${dataset_name_1}-${method}-on-test-set ${test_1}
evaluate ${dataset_name_2} ${dataset_name_2} ${dataset_name_2}-${method}-on-test-set ${test_2}
evaluate ${dataset_name_3} ${dataset_name_3} ${dataset_name_3}-${method}-on-test-set ${test_3}
evaluate ${dataset_name_4} ${dataset_name_4} ${dataset_name_4}-${method}-on-test-set ${test_4}

# Evaluate expert policy on training episodes
evaluate_expert ${dataset_name_0} ${dataset_name_0}-expert-on-training-set ${train_0}
evaluate_expert ${dataset_name_1} ${dataset_name_1}-expert-on-training-set ${train_1}
evaluate_expert ${dataset_name_2} ${dataset_name_2}-expert-on-training-set ${train_2}
evaluate_expert ${dataset_name_3} ${dataset_name_3}-expert-on-training-set ${train_3}
evaluate_expert ${dataset_name_4} ${dataset_name_4}-expert-on-training-set ${train_4}

# Evaluate expert policy on test episodes
evaluate_expert ${dataset_name_0} ${dataset_name_0}-expert-on-test-set ${test_0}
evaluate_expert ${dataset_name_1} ${dataset_name_1}-expert-on-test-set ${test_1}
evaluate_expert ${dataset_name_2} ${dataset_name_2}-expert-on-test-set ${test_2}
evaluate_expert ${dataset_name_3} ${dataset_name_3}-expert-on-test-set ${test_3}
evaluate_expert ${dataset_name_4} ${dataset_name_4}-expert-on-test-set ${test_4}

print_green "Done."

echo "Evaluation data stored in ${evaluation_data_csv_folder}"
echo "Tensorboard data stored in runs/${project_name}"
echo "Models stored in models/${project_name}"
