# Results ICRA 2024

Here are the steps to recreate the results from our ICRA 2024 paper.

## 1. Training -> Save to tensorboard files
To reproduce the results described in the [human-robot-gym paper](https://arxiv.org/pdf/2310.06208.pdf), execute

```
./icra_2024_run_experiments_in_docker.sh
```

This will
- Create a docker image,
- Run a container, and
- Run all experiments within the container.

## 2. Convert tensorboard files to csv files
This uses the human-robot-gym script `python human_robot_gym/utils/data_pipeline.py`.

The pipeline consists of the following steps:
  1. Scrape tensorboard data from the runs folder
  2. Average all datapoints in a given time interval (rastering)
  3. Determine statistics (running mean, std and bootstrapped 95% confidence intervals)

After every step, the data is saved to .csv files. The output folder contains the following subfolders:
    - raw: Contains the raw data from tensorboard
    - rastered: Contains the rastered data
    - stats: Contains the statistics

Example usage:

```
python human_robot_gym/utils/data_pipeline.py <run_id_1> ... <run_id_x> -y \
    -i <tb_log_folder> -o <output_folder> -t <tag_1> ... <tag_y> \
    -n <n_steps> -g <raster_granularity> -w <window_size> -b <bootstrap_samples>
```

The arguments are:
    - <run_id_1> ... <run_id_n>: The run ids of the experiments. The run id is the name of the folder containing the
        tensorboard files.
    - -y (optional): If specified, the script will not ask for confirmation before overwriting existing files.
    - -i <tb_log_folder>: Folder in which the tensorboard log files are located.
        May also be a remote folder in which case the files will be first copied to a local folder.
    - -o <output_folder>: Path to output folder. The csv files will be saved in this folder.
    - -t <tag_1> ... <tag_m>: List of metrics to include in the csv file.
        If not specified, the following tags will be included:
        - "rollout/ep_env_rew_mean": episode return
        - "rollout/ep_len_mean": episode length
        - "rollout/n_goal_reached": success rate
    - -n <n_steps>: relevant for rastering, step in training at which to stop rastering. If not specified, the last step
        of the training will be used.
    - -g <raster_granularity>: relevant for rastering, granularity of rastering. Size of the step interval in which
        the data is averaged.
    - -w <window_size>: relevant for statistics extraction, window size for moving average filter.
    - -b <bootstrap_samples>: relevant for statistics extraction, number of bootstrap samples.

## 3. Generate the plotting tex files
Run `python generate_files.py` here.
This will generate the plotting files in `output/*.tex`.
You can then use `pdflatex` to run these files.

## 4. Run the plotting files 
We provide scripts to compile all plotting files using `run_reward_plots_train.sh`, `run_success_plots_train.sh`, and `run_ablation_study.sh`.