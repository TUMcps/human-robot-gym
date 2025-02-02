# run this code after the training has finished and all the log files have been saved, such as the tensorboard log files.

python human_robot_gym/utils/data_pipeline.py <run_id_1> ... <run_id_x> -y \
    -i <tb_log_folder> -o <output_folder> -t <tag_1> ... <tag_y> \
    -n <n_steps> -g <raster_granularity> -w <window_size> -b <bootstrap_samples>

python generate_files.py