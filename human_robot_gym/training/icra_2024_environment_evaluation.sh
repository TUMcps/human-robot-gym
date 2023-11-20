#!/bin/bash

# This script can be used to run the full evaluation pipeline for a given environment.
# By default, training is performed on 5 seeds with 8 parallel environments each -> up to 40 parallel threads!
# This script will create a dataset, train the models, evaluate them, and save the results in csv files.
# The files will be saved in the following locations:
#   - csv/training/<env_long> for the training data
#   - csv/evaluation/<env_long> for the evaluation data
# The script can be run with the following command:
# ./environment_evaluation.sh <env> <env_long> <n_dataset_episodes> <n_training_steps> <horizon> <n_envs> <log_interval> <run_type>
# where:
# <env> is the acronym of the environment:
#   R for HumanReach
#   PP for PickPlaceHuman
#   CL for CollaborativeLifting
#   RHH for RobotHumanHandover
#   HRH for HumanRobotHandover
#   CS for CollaborativeStacking
# <env_long> is a readable name of the environment, e.g. PickPlaceHuman
# <n_dataset_episodes> is the number of episodes in the expert dataset
# <n_training_steps> is the number of training steps for each model
# <horizon> is the maximum episode length
# <n_envs> is the number of parallel environments
# <log_interval> is the stride between tensorboard log entries
# <run_type> defines whether training logging should be either done
#   - in wandb (run_type='wandb'),
#   - in tensorboard only (run_type='tensorboard'), or
#   - not at all (run_type='debug')
#   Default: 'tensorboard'
env=$1
env_long=$2
n_dataset_episodes=${3}
n_steps=${4}
horizon=${5}
n_envs=${6}  # Parallel environments for training
log_interval=${7}
run_type=${8:-"tensorboard"}
n_test_episodes=20
model_save_interval=50000  # Model saving interval
granularity=$((${log_interval}*3))  # Logged data is averaged over this many steps
window_size=9  # Window size for the moving average
max_eval_threads=50  # Maximum number of parallel evaluation threads

project_name="${env_long}_environment_evaluation"

training_data_csv_folder="csv/training/${project_name}"
evaluation_data_csv_folder="csv/evaluation/${project_name}"

delete_intermediate_data=true  # Whether to only keep the statistics and delete the raw csv data of the training and evaluation.

seeds=(0 1 2 3 4)

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
    if [ -d "datasets/${env_long}" ]; then
        echo "Overwriting existing dataset"
        rm -r datasets/${env_long}
    fi

    if [ -d ${training_data_csv_folder} ]; then
        echo "Overwriting existing training csv data at ${training_data_csv_folder}"
        rm -r $training_data_csv_folder
    fi

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
#   generate_dataset
generate_dataset () {
    (
        set -o xtrace
        python human_robot_gym/training/create_expert_dataset.py \
            -cp config_icra_2024/environment_evaluation/dataset_creation \
            -cn ${env} \
            dataset_name=${env_long} n_episodes=${n_dataset_episodes} \
            environment.horizon=${horizon} environment.verbose=False
    )
}

# Execute a training run
# Usage:
#   train <method> (<group>)
# Args:
#   <method>: name of the method, either
#       - AIR for action-based expert imitation reward
#       - SIR for state-based expert imitation reward
#       - RSI for reference state initialization
#       - SAC for vanilla soft actor-critic
#   <group>: name of the run group, name of the subfolders in which the runs and models are stored, run group for wandb. Defaults to <method>.
train () {
    local method=$1
    local group=${2:-${method}}
    local run_seed_arg=$(join_by , ${seeds[@]})
    (
        set -o xtrace
        python human_robot_gym/training/train_SB3.py --multirun \
            -cp config_icra_2024/environment_evaluation/training \
            -cn ${env}-${method} \
            hydra/launcher=ray \
            wandb_run.project=${project_name} wandb_run.group=${group} \
            run.type=${run_type} run.n_steps=${n_steps} "run.seed=${run_seed_arg}" run.n_envs=${n_envs} \
            run.dataset_name=${env_long} "run.log_interval=[${log_interval},'step']" run.save_freq=${model_save_interval} \
            environment.horizon=${horizon} environment.verbose=False
    )
}

# Pipeline for extracting data logged during training from tensorboard log files and store statistics into .csv files
# Usage:
#   training_data_pipeline <method> (<group>)
# Args:
#   <method>: name of the method, either
#       - AIR for action-based expert imitation reward
#       - SIR for state-based expert imitation reward
#       - RSI for reference state initialization
#       - SAC for vanilla soft actor-critic
#   <group>: name of the run group, name of the subfolders in which the runs and csv files are stored, run group for wandb. Defaults to <method>.
training_data_pipeline () {
    local method=$1
    local group=${2:-${method}}
    local runs=$(for seed in ${seeds[@]}; do echo "run_${seed}"; done)
    (
        set -o xtrace
        python human_robot_gym/utils/data_pipeline.py \
            ${runs} \
            -i runs/${project_name}/${group} \
            -o csv/training/${project_name}/${group} \
            -n ${n_steps} -g ${granularity} -w ${window_size} -y
    )
}

# Evaluate models from snapshots during training and store statistics into .csv files
# Usage:
#   evaluate <group>
# Args:
#   <group>: name of the run group, name of the subfolders in which the models and csv files are stored.
evaluate () {
    local group=$1
    local run_ids=[$(join_by , $(for seed in ${seeds[@]}; do echo ${project_name}/${group}/run_${seed}; done))]
    (
        set -o xtrace
        python human_robot_gym/training/evaluate_models_to_csv_SB3.py \
            -cp config_icra_2024/environment_evaluation/evaluation \
            -cn ${env} \
            "run.id=${run_ids}" \
            group_name=${project_name}/${group} max_parallel_runs=${max_eval_threads} \
            run.load_step=all run.n_test_episodes=${n_test_episodes} run.n_steps=${n_steps} run.save_freq=${model_save_interval} \
            environment.horizon=${horizon} environment.verbose=False \
            wrappers.dataset_obs_norm.dataset_name=${env_long}
    )
}

# Evaluate the expert policy on the test episodes and store statistics into a .csv file
# Usage:
#   evaluate_expert
evaluate_expert () {
    (
        set -o xtrace
        python human_robot_gym/training/evaluate_models_to_csv_SB3.py \
            -cp config_icra_2024/environment_evaluation/evaluation \
            -cn ${env} \
            group_name=${project_name}/expert \
            run.id=null run.load_step=final run.n_test_episodes=${n_test_episodes} \
            environment.horizon=${horizon} environment.verbose=False \
            wrappers.dataset_obs_norm.dataset_name=${env_long}
    )
}


# =============================================================================
# ======================== Cleanup of Existing Data ===========================
# =============================================================================

# Comment if you do not want to override any data
cleanup_existing_data


# =============================================================================
# ======================== Generate an Expert Dataset =========================
# =============================================================================

print_green "Generating dataset..."
generate_dataset

# Store the expert statistics into the training data csv folder
if [ ${run_type} != debug ]
then
    mkdir -p ${training_data_csv_folder}
    cp "datasets/${env_long}/stats.csv" "${training_data_csv_folder}/expert.csv"
fi


# =============================================================================
# ======================== Run Training =======================================
# =============================================================================

print_green "Dataset created, proceeding with training..."

train AIR  # Soft actor-critic with reference state initialization and action-based expert imitation reward
train SIR  # Soft actor-critic with reference state initialization and state-based expert imitation reward
train RSI  # Soft actor-critic with reference state initialization
train SAC  # Soft-actor critic

if [ ${run_type} = debug ]
then
    print_green "Debug training done."
    # Terminate early, no log data or models to evaluate stored
    exit 0
fi


# =============================================================================
# ======================== Obtain Training Statistics =========================
# =============================================================================

print_green "Training done, obtaining data statistics..."

training_data_pipeline AIR
training_data_pipeline SIR
training_data_pipeline RSI
training_data_pipeline SAC

# Cleanup the csv data
if $delete_intermediate_data
then
    for method in AIR SIR RSI SAC
    do
        mv ${training_data_csv_folder}/${method}/stats/stats.csv ${training_data_csv_folder}/${method}.csv
        rm -r ${training_data_csv_folder}/${method}
    done
fi


# =============================================================================
# ======================== Evaluate Model Snapshots ===========================
# =============================================================================

print_green "Data statistics obtained, proceeding with evaluation..."

evaluate AIR
evaluate SIR
evaluate RSI
evaluate SAC

evaluate_expert

# Cleanup the evaluation data
mkdir -p ${evaluation_data_csv_folder}

if $delete_intermediate_data
then
    for method in AIR SIR RSI SAC expert
    do
        mv ${evaluation_data_csv_folder}/${method}/stats.csv ${evaluation_data_csv_folder}/${method}.csv
        rm -r ${evaluation_data_csv_folder}/${method}
    done
fi


print_green "Done."

echo "Models saved at models/${project_name}"
echo "Tensorboard log files saved at runs/${project_name}"
echo "Training results saved at ${training_data_csv_folder}"
echo "Evaluation data saved at ${evaluation_data_csv_folder}"
