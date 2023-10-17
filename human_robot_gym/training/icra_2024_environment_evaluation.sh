#!/bin/bash

# This script can be used to run the full evaluation pipeline for a given environment.
# By default, training is performed on 5 seeds with 8 parallel environments each -> up to 40 parallel threads!
# This script will create a dataset, train the models, evaluate them, and save the results in csv files.
# The files will be saved in the following locations:
#   - csv/training/<env_long> for the training data
#   - csv/evaluation/<env_long> for the evaluation data
# The script can be run with the following command:
# ./environment_evaluation.sh <env> <env_long> <n_dataset_episodes> <n_training_steps>
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

env=$1
env_long=$2
n_dataset_episodes=$3
n_steps=$4
n_test_episodes=20
granularity=24000  # Logged data is averaged over this many steps
n_envs=8  # Parallel environments for training
window_size=9  # Window size for the moving average

project_name="${env_long}_evaluation"

training_data_csv_folder="csv/training/${project_name}"
evaluation_data_csv_folder="csv/evaluation/${project_name}"

green='\033[0;32m'
NC='\033[0m' # No Color


# Cleanup any existing data to avoid issues
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

# Comment if you do not want to override any data
cleanup_existing_data


echo "${green}Generating dataset...${NC}"

# Generate a dataset
python human_robot_gym/training/create_expert_dataset.py -cn ${env}_dataset_creation dataset_name=${env_long} n_episodes=${n_dataset_episodes}
# Store the expert statistics on the dataset
mkdir -p "${training_data_csv_folder}/expert"
cp "datasets/${env_long}/stats.csv" "${training_data_csv_folder}/expert/stats.csv"

echo "${green}Dataset created, proceeding with training...${NC}"

train () {
    local method=$1
    local group=${2:-${method}}
    python human_robot_gym/training/train_SB3.py --multirun -cn ${env}_train_${method} hydra/launcher=ray run.type=tensorboard run.n_steps=${n_steps} wandb_run.project=${project_name} wandb_run.group=${group} "run.seed=0,1,2,3,4" run.n_envs=${n_envs} run.dataset_name=${env_long} run.resetting_interval=${resetting_interval}
}

# Train the models
train air
train sir
train sac_rsi
train sac

if $compare_with_resetting; then
    train air air_resetting 600000
fi

echo "${green}Training done, obtaining data statistics...${NC}"

training_data_pipeline () {
    local method=$1
    local group=${2:-${method}}
    python human_robot_gym/utils/data_pipeline.py ${method}_0 ${method}_1 ${method}_2 ${method}_3 ${method}_4 -i runs/${project_name}/${group} -o csv/training/${project_name}/${group} -n ${n_steps} -g ${granularity} -w ${window_size}
}

# Obtain the training statistics
training_data_pipeline air
training_data_pipeline sir
training_data_pipeline sac_rsi
training_data_pipeline sac

if $compare_with_resetting; then
    training_data_pipeline air air_resetting
fi

# Cleanup the csv data
# Comment these lines if you want to keep the raw csv data (tensorboard logs are kept no matter what)
rm -r ${training_data_csv_folder}/air/raw ${training_data_csv_folder}/air/rastered
rm -r ${training_data_csv_folder}/sir/raw ${training_data_csv_folder}/sir/rastered
rm -r ${training_data_csv_folder}/sac_rsi/raw ${training_data_csv_folder}/sac_rsi/rastered
rm -r ${training_data_csv_folder}/sac/raw ${training_data_csv_folder}/sac/rastered

if $compare_with_resetting; then
    rm -r ${training_data_csv_folder}/air_resetting/raw ${training_data_csv_folder}/air_resetting/rastered
fi

mv ${training_data_csv_folder}/air/stats/* ${training_data_csv_folder}/air
mv ${training_data_csv_folder}/sir/stats/* ${training_data_csv_folder}/sir
mv ${training_data_csv_folder}/sac_rsi/stats/* ${training_data_csv_folder}/sac_rsi
mv ${training_data_csv_folder}/sac/stats/* ${training_data_csv_folder}/sac

if $compare_with_resetting; then
    mv ${training_data_csv_folder}/air_resetting/stats/* ${training_data_csv_folder}/air_resetting
    rm -r ${training_data_csv_folder}/air_resetting/stats
fi

rm -r ${training_data_csv_folder}/air/stats ${training_data_csv_folder}/sir/stats ${training_data_csv_folder}/sac_rsi/stats ${training_data_csv_folder}/sac/stats


echo "${green}Data statistics obtained, proceeding with evaluation...${NC}"

evaluate () {
    local group=$1
    assemble_evaluation_run_id () {
        local run_index=$1
        echo "${project_name}/${group}/${method}_${run_index}"
    }
    local evaluation_run_ids="[$(assemble_evaluation_run_id 0),$(assemble_evaluation_run_id 1),$(assemble_evaluation_run_id 2),$(assemble_evaluation_run_id 3),$(assemble_evaluation_run_id 4)]"
    python human_robot_gym/training/evaluate_models_to_csv.py -cn ${env}_eval_to_csv "run.id=${evaluation_run_ids}" group_name=${project_name}/${group} wrappers.dataset_obs_norm.dataset_name=${env_long} run.load_step=all run.n_test_episodes=${n_test_episodes} &
}

# Evaluate the models
evaluate air
evaluate sir
evaluate sac_rsi
evaluate sac

if $compare_with_resetting; then
    evaluate air_resetting
fi

# Evaluate the expert
python human_robot_gym/training/evaluate_models_to_csv.py -cn ${env}_eval_to_csv "run.id=null" group_name=${env_long}/expert wrappers.dataset_obs_norm.dataset_name=${env_long} run.load_step=final run.n_test_episodes=${n_test_episodes} & 

wait

# Cleanup the evaluation data
mkdir -p ${evaluation_data_csv_folder}
mv csv/evaluation/stats/${project_name}/expert/stats.csv ${evaluation_data_csv_folder}/expert.csv
mv csv/evaluation/stats/${project_name}/air/stats.csv ${evaluation_data_csv_folder}/air.csv
mv csv/evaluation/stats/${project_name}/sir/stats.csv ${evaluation_data_csv_folder}/sir.csv
mv csv/evaluation/stats/${project_name}/sac_rsi/stats.csv ${evaluation_data_csv_folder}/sac_rsi.csv
mv csv/evaluation/stats/${project_name}/sac/stats.csv ${evaluation_data_csv_folder}/sac.csv

if $compare_with_resetting; then
    mv csv/evaluation/stats/${project_name}/air_resetting/stats.csv ${evaluation_data_csv_folder}/air_resetting.csv
fi

rm -r csv/evaluation/stats
rm -r csv/evaluation/raw


echo "${green}Done.${NC}"

echo "Training results saved at csv/training/${env_long}"
echo "Evaluation data saved at csv/evaluation/${env_long}"
