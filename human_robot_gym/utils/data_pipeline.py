"""This file can be used to extract training log data from tensorboard logs
determine statistics (running mean, std and bootstrapped 95% confidence intervals),
and save them to .csv files.

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

Author:
    Felix Trost (FT)

Changelog:
    13.10.23 FT File created
"""
from typing import List, Optional
import os
import shutil

import numpy as np
import pandas as pd

from scipy.stats import bootstrap
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

from human_robot_gym import human_robot_gym_root


def save_df_to_csv(df: pd.DataFrame, output_path: str, override_automatically: bool = False):
    if os.path.exists(output_path):
        print(f"Output file {output_path} already exists!")
        print("Do you want to overwrite it? y/[n]")
        if override_automatically or input() == "y":
            print("Overwriting...")
            os.system(f"rm {output_path}")
        else:
            print("Aborting...")
            return

    df.to_csv(output_path, index=False)


def get_tb_folder_path(runs_folder: str, run_id: str, run_index: int = 1) -> str:
    run_folder = os.path.join(runs_folder, run_id)
    run_subfolders = [f.path for f in os.scandir(run_folder) if f.is_dir()]

    run_subfolders = [f for f in run_subfolders if f.endswith(f"_{run_index}")]
    assert len(run_subfolders) == 1, f"Run folder {run_folder} does not contain a single matching subfolder."
    return run_subfolders[0]


def scp_if_remote_folder(runs_folder: str, run_id: str) -> str:
    is_ssh_path = os.path.normpath(runs_folder).split(os.sep)[0].endswith(":")
    if is_ssh_path:
        dest_folder = "./runs"
        if os.path.exists(os.path.join(dest_folder, run_id)):
            shutil.rmtree(os.path.join(dest_folder, run_id))
        os.makedirs(dest_folder, exist_ok=True)
        print("Running scp to copy files from remote server...")
        command = f"scp -r {os.path.join(runs_folder, run_id)} {os.path.join(dest_folder, run_id)}"
        print(command)
        os.system(command)

        return dest_folder

    return runs_folder


def tb_tag_to_df(tb_folder_path: str, tag: str) -> pd.DataFrame:
    summary_iterator = EventAccumulator(tb_folder_path).Reload()

    if tag not in summary_iterator.Tags()["scalars"]:
        raise ValueError(
            f"Tag {tag} not found in tensorboard files. Available tags are: {summary_iterator.Tags()['scalars']}"
        )

    dataframe = pd.DataFrame({
        "step": pd.DataFrame.from_records(
            summary_iterator.Scalars(tag),
            columns=summary_iterator.Scalars(tag)[0]._fields,
        )["step"].values,
        "wall_time": pd.DataFrame.from_records(
            summary_iterator.Scalars(tag),
            columns=summary_iterator.Scalars(tag)[0]._fields,
        )["wall_time"].values,
        tag: pd.DataFrame.from_records(
            summary_iterator.Scalars(tag),
            columns=summary_iterator.Scalars(tag)[0]._fields,
        )["value"].values,
    })

    return dataframe


def tb_to_merged_df(tb_folder_path: str, tags: Optional[List[str]]) -> pd.DataFrame:
    summary_iterator = EventAccumulator(tb_folder_path).Reload()

    dataframe = pd.DataFrame()

    if tags is None:
        tags = summary_iterator.Tags()["scalars"]

    dataframe = pd.DataFrame({
        "step": pd.DataFrame.from_records(
            summary_iterator.Scalars(tags[0]),
            columns=summary_iterator.Scalars(tags[0])[0]._fields,
        )["step"].values,
        "wall_time": pd.DataFrame.from_records(
            summary_iterator.Scalars(tags[0]),
            columns=summary_iterator.Scalars(tags[0])[0]._fields,
        )["wall_time"].values,
    })

    n_steps = len(summary_iterator.Scalars(tags[0]))

    for tag in tags:
        scalar_dataframe = pd.DataFrame.from_records(
            summary_iterator.Scalars(tag),
            columns=summary_iterator.Scalars(tag)[0]._fields,
        )["value"].values

        if len(scalar_dataframe) != n_steps:
            print(f"Skipping {tag} because it has {len(scalar_dataframe)} entries instead of {n_steps}.")
        else:
            dataframe[tag] = scalar_dataframe

    return dataframe


def scrape_run(
    runs_folder: str,
    run_id: str,
    output_folder: str,
    tags: Optional[List[str]] = None,
    override_automatically: bool = False,
    run_index: int = 1,
):
    if not os.path.exists(output_folder):
        print(f"Creating output folder {output_folder}...")
        os.makedirs(output_folder)

    local_runs_folder = scp_if_remote_folder(runs_folder=runs_folder, run_id=run_id)

    tb_folder_path = get_tb_folder_path(
        runs_folder=local_runs_folder,
        run_id=run_id,
        run_index=run_index,
    )

    df = tb_to_merged_df(tb_folder_path=tb_folder_path, tags=tags)
    save_df_to_csv(
        df,
        output_path=os.path.join(output_folder, f"{run_id}.csv"),
        override_automatically=override_automatically
    )


def scrape_runs(
    runs_folder: str,
    run_ids: List[str],
    output_folder: str,
    tags: Optional[List[str]] = None,
    override_automatically: bool = False,
    run_index: int = 1,
):
    for run_id in run_ids:
        try:
            scrape_run(
                runs_folder=runs_folder,
                run_id=run_id,
                output_folder=output_folder,
                tags=tags,
                override_automatically=override_automatically,
                run_index=run_index,
            )
        except Exception as e:
            print(f"Error while scraping {run_id}: {e.with_traceback()}")


def raster_data_frame(
    df_in: pd.DataFrame,
    granularity: int = 8000,
    n_steps: Optional[int] = None,
) -> pd.DataFrame:
    steps = df_in.step.values

    if n_steps is None:
        n_steps = steps[-1] + granularity

    df_out = pd.DataFrame(columns=df_in.columns)
    bins = [
        i * granularity
        for i in range(n_steps // granularity)
    ]
    for bin in bins:
        df_in_bin = df_in[steps >= bin]
        df_in_bin = df_in_bin[df_in_bin.step < bin + granularity]
        df_out = pd.concat(
            [
                df_out,
                df_in_bin.mean().to_frame().T,
            ],
            ignore_index=True,
        )

    df_out.step = [bin + granularity for bin in bins]

    return df_out


def raster_csv(
    input_folder: str,
    output_folder: str,
    run_id: str,
    granularity: int = 5000,
    n_steps: Optional[int] = None,
    override_automatically: bool = False,
):
    assert os.path.exists(input_folder), f"Input folder {input_folder} does not exist."

    if not os.path.exists(output_folder):
        print(f"Creating output folder {output_folder}...")
        os.makedirs(output_folder)

    in_filepath = os.path.join(input_folder, f"{run_id}.csv")
    in_df = pd.read_csv(in_filepath, comment="#")
    out_df = raster_data_frame(df_in=in_df, granularity=granularity, n_steps=n_steps)

    output_file = os.path.join(output_folder, f"{run_id}.csv")

    save_df_to_csv(df=out_df, output_path=output_file, override_automatically=override_automatically)


def raster_csvs(
    input_folder: str,
    output_folder: str,
    filenames: Optional[List[str]],
    granularity: int = 5000,
    n_steps: Optional[int] = None,
):
    if filenames is None or len(filenames) == 0:
        filenames = [f[:-4] for f in os.listdir(input_folder) if f.endswith(".csv")]

    for run_id in filenames:
        raster_csv(
            input_folder=input_folder,
            output_folder=output_folder,
            run_id=run_id,
            granularity=granularity,
            n_steps=n_steps
        )


def smooth_stats_data_frame(
    dfs_in: List[pd.DataFrame],
    window_size: int = 9,
    bootstrap_samples: int = 10000,
) -> pd.DataFrame:
    for df_in in dfs_in:
        assert np.all(df_in.step % (df_in.step[1] - df_in.step[0]) == 0), "Steps must be equidistant!"

    assert window_size % 2 == 1, "Window size must be odd!"

    half_window_size = (window_size - 1) // 2

    out_df = pd.DataFrame({
        "step": [
            dfs_in[0].step[i]
            for i in range(half_window_size, dfs_in[0].shape[0] - half_window_size)
        ],
        "wall_time": [
            dfs_in[0].wall_time[i]
            for i in range(half_window_size, dfs_in[0].shape[0] - half_window_size)
        ]
    })

    tags = [c for c in dfs_in[0].columns if c not in ["step", "wall_time"]]

    for tag in tags:
        values = np.stack([df_in[tag].values for df_in in dfs_in], axis=0)
        mean = np.zeros(df_in.shape[0] - 2 * half_window_size)
        std = np.zeros(df_in.shape[0] - 2 * half_window_size)
        bootstrap025 = np.zeros(df_in.shape[0] - 2 * half_window_size)
        bootstrap975 = np.zeros(df_in.shape[0] - 2 * half_window_size)

        for i in range(half_window_size, df_in.shape[0] - half_window_size):
            window = values[:, i - half_window_size:i + half_window_size]
            mean[i - half_window_size] = np.nanmean(window)
            std[i - half_window_size] = np.nanstd(window)
            flat_window = np.concatenate(window, axis=0)
            flat_window = flat_window[~np.isnan(flat_window)]
            conf = bootstrap(
                data=flat_window[np.newaxis, :],
                statistic=np.mean,
                confidence_level=0.95,
                axis=0,
                n_resamples=bootstrap_samples,
            ).confidence_interval
            bootstrap025[i - half_window_size] = np.nan_to_num(conf.low, mean[i - half_window_size])
            bootstrap975[i - half_window_size] = np.nan_to_num(conf.high, mean[i - half_window_size])

            out_df[f"{tag}_mean"] = mean
            out_df[f"{tag}_std"] = std
            out_df[f"{tag}_bootstrap025"] = bootstrap025
            out_df[f"{tag}_bootstrap975"] = bootstrap975

    return out_df


def smooth_stats_csvs(
    input_folder: str,
    output_folder: str,
    window_size: int = 9,
    bootstrap_samples: int = 10000,
):
    assert os.path.exists(input_folder), f"Input folder {input_folder} does not exist."

    if not os.path.exists(output_folder):
        print(f"Creating output folder {output_folder}...")
        os.makedirs(output_folder)

    dfs_in = [
        pd.read_csv(f"{input_folder}/{f}", comment="#")
        for f in os.listdir(input_folder)
        if os.path.isfile(f) and f.endswith(".csv")
    ]

    out_df = smooth_stats_data_frame(
        dfs_in=dfs_in,
        window_size=window_size,
        bootstrap_samples=bootstrap_samples,
    )

    save_df_to_csv(df=out_df, output_path=f"{output_folder}/stats.csv")


def main(
    run_ids: List[str],
    input_folder: str,
    output_folder: str,
    tags: List[str],
    raster_granularity=24000,
    n_steps=3_000_000,
    window_size=9,
    bootstrap_samples=10000,
    override_automatically: bool = False,
):
    raw_folder = f"{output_folder}/raw"
    rastered_folder = f"{output_folder}/rastered"
    stats_folder = f"{output_folder}/stats"

    if os.path.exists(output_folder):
        print(f"Output folder {output_folder} already exists.")
        print("Do you want to overwrite it? y/[n]")
        if override_automatically or input() == "y":
            print("Overwriting...")
            os.system(f"rm -r {output_folder}")
        else:
            print("Aborting...")
            return

    scrape_runs(
        runs_folder=input_folder,
        run_ids=run_ids,
        output_folder=raw_folder,
        tags=tags,
        override_automatically=override_automatically,
        run_index=1,
    )

    raster_csvs(
        input_folder=raw_folder,
        output_folder=rastered_folder,
        filenames=None,
        granularity=raster_granularity,
        n_steps=n_steps,
    )

    smooth_stats_csvs(
        input_folder=raw_folder,
        output_folder=stats_folder,
        tags=tags,
        window_size=window_size,
        bootstrap_samples=bootstrap_samples,
    )


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Convert tensorboard data to statistics csv.')
    parser.add_argument(
        "run_ids",
        type=str,
        nargs="+",
        help="Run ids of the experiments. The run id is the name of the folder containing the tensorboard files.",
    )
    parser.add_argument(
        "--input-folder",
        "-i",
        type=str,
        default=os.path.join(os.path.dirname(human_robot_gym_root), "runs"),
        help="Folder in which the tensorboard files are located.",
    )
    parser.add_argument(
        "--output-folder",
        "-o",
        type=str,
        default=os.path.join(os.path.dirname(human_robot_gym_root), "csv"),
        help="Path to output folder. The csv files will be saved in this folder.",
    )
    parser.add_argument(
        "--tags",
        "-t",
        type=str,
        nargs="*",
        default=["rollout/ep_env_rew_mean", "rollout/ep_len_mean", "rollout/n_goal_reached"],
        help="List of tags to include in the csv file. If not specified, the plotted tags will be included.",
    )
    parser.add_argument(
        "--n-steps",
        "-n",
        type=int,
        default=None,
        help="Number of steps to consider for rastering.",
    )
    parser.add_argument(
        "--granularity",
        "-g",
        type=int,
        default=24000,
        help="Granularity of rastering.",
    )
    parser.add_argument(
        "--window-size",
        "-w",
        type=int,
        default=9,
        help="Window size for moving average filter.",
    )
    parser.add_argument(
        "--bootstrap-samples",
        "-b",
        type=int,
        default=10000,
        help="Number of bootstrap samples.",
    )
    parser.add_argument(
        "--override-automatically",
        "-y",
        action="store_true",
        help="If specified, the script will not ask for confirmation before overwriting existing files.",
    )
    args = parser.parse_args()

    main(
        run_ids=args.run_ids,
        input_folder=args.input_folder,
        output_folder=args.output_folder,
        tags=args.tags,
        raster_granularity=args.granularity,
        n_steps=args.n_steps,
        window_size=args.window_size,
        bootstrap_samples=args.bootstrap_samples,
        override_automatically=args.override_automatically,
    )
