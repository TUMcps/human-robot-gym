import argparse
import glob
import os
import csv
import numpy as np


MIN_REWARD = {
    'reach': -50,
    'pp': -1000,
    'cl': 100,
    'rhh': -1000,
    'hrh': -1000,
    'cs': -3000
}

parser = argparse.ArgumentParser()
parser.add_argument("--type", help="Type of plots to create. E.g. 'train' or 'eval'", default="train")
args = parser.parse_args()
mode = args.type
print(mode)

def find_expert_values(experiment: str, metric: str):
    """Find the expert value for a given experiment.

    Read in csv file in ../data/train/experiment/expert/[some_file_name].csv and 
    return the first value of the column [metric]_mean.
    File format:
        success_mean,success_025,success_975,ep_len_mean,ep_len_025,ep_len_975,ep_rew_mean,ep_rew_025,ep_rew_975
        0.5699999999999999,0.46,0.65,410.27,378.1954566361617,444.02,316.5490937069618,281.31005195768574,353.00448443742545

    Args:
        experiment (str): Name of the experiment. E.g. 'reach', 'pp', 'cl', 'rhh', 'hrh', 'cs'
        metric (str): Name of the metric. E.g. 'success', 'ep_len', 'ep_rew'
    Returns:
        expert_values List[float]: [mean, lower_bound, upper_bound]
    """
    assert experiment in MIN_REWARD.keys(), "Invalid experiment name."
    assert metric in ['success', 'ep_len', 'ep_rew'], "Invalid metric name."
    this_dir = os.path.dirname(os.path.abspath(__file__))
    # Find the first csv file in the folder "../data/train/[experiment]/expert/"
    csv_file = glob.glob(this_dir + "/data/train/" + experiment + "/expert/*.csv")[0]
    # Read in the csv file
    with open(csv_file, newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        header = csvreader.__next__()
        data = csvreader.__next__()
        expert_values = []
        for col in ["mean", "025", "975"]:
            # Find the index of the metric
            metric_index = header.index(metric + '_' + col)
            # Find the expert value
            expert_value = float(data[metric_index])
            expert_values.append(expert_value)
    csvfile.close()
    return expert_values


def write_reward_plot_train(exp_type: str):
    """Write the tex file for the reward plot of a given experiment."""
    expert_values = find_expert_values(exp_type, 'ep_rew')
    min_reward = MIN_REWARD[exp_type]
    this_dir = os.path.dirname(os.path.abspath(__file__))
    f = open(this_dir + "/output/reward_" + exp_type + "_train.tex", "w")
    f.write("% !TeX root = reward_" + exp_type + "_train\n")
    f.write("% border={<left> <bottom> <right> <top>} left and right should be same.\n")
    if exp_type == list(MIN_REWARD.keys())[0]:
        f.write("\documentclass[border={-0.05cm -0.11cm -0.07cm -0.00cm},tikz]{standalone}\n")
    else:
        f.write("\documentclass[border={-0.12cm -0.11cm -0.07cm -0.00cm},tikz]{standalone}\n")
    f.write("\input{../plotstyle}\n")
    f.write("\n")
    f.write("\\begin{document}\n")
    plot_type = "Short" if exp_type == "reach" else ""
    f.write("	\plotNormalizedReward"  + plot_type + "{" + exp_type +
            "}{" + str(exp_type == list(MIN_REWARD.keys())[0]) +
            "}{" + str(min_reward) +
            "}{" + str(expert_values[0]) +
            "}{" + str(expert_values[1]) +
            "}{" + str(expert_values[2]) + "}\n")
    f.write("\end{document}\n")
    f.close()


def write_success_plot_train(exp_type: str):
    """Write the tex file for the success plot of a given experiment."""
    expert_values = find_expert_values(exp_type, 'success')
    this_dir = os.path.dirname(os.path.abspath(__file__))
    f = open(this_dir + "/output/success_" + exp_type + "_train.tex", "w")
    f.write("% !TeX root = succes_" + exp_type + "_train\n")
    f.write("% border={<left> <bottom> <right> <top>} left and right should be same.\n")
    if exp_type == list(MIN_REWARD.keys())[0]:
        f.write("\documentclass[border={-0.05cm -0.11cm -0.07cm -0.00cm},tikz]{standalone}\n")
    else:
        f.write("\documentclass[border={-0.12cm -0.11cm -0.07cm -0.00cm},tikz]{standalone}\n")
    f.write("\input{../plotstyle}\n")
    f.write("\n")
    f.write("\\begin{document}\n")
    plot_type = "Short" if exp_type == "reach" else ""
    f.write("	\plotSuccess"  + plot_type + "{" + exp_type +
            "}{" + str(exp_type == list(MIN_REWARD.keys())[0]) +
            "}{" + str(expert_values[0]) +
            "}{" + str(expert_values[1]) +
            "}{" + str(expert_values[2]) + "}\n")
    f.write("\end{document}\n")
    f.close()


def get_ablation_data(exp_type: str, file: str):
    """Return the train and test set mean, 025 and 975 data for a given metric."""
    with open(file, newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        header = csvreader.__next__()
        trainset_mean_index = header.index("trainset_mean")
        trainset_025_index = header.index("trainset_025")
        trainset_975_index = header.index("trainset_975")
        testset_mean_index = header.index("testset_mean")
        testset_025_index = header.index("testset_025")
        testset_975_index = header.index("testset_975")
        trainset_mean = []
        trainset_025 = []
        trainset_975 = []
        testset_mean = []
        testset_025 = []
        testset_975 = []
        for row in csvreader:
            trainset_mean.append(float(row[trainset_mean_index]))
            trainset_025.append(float(row[trainset_025_index]))
            trainset_975.append(float(row[trainset_975_index]))
            testset_mean.append(float(row[testset_mean_index]))
            testset_025.append(float(row[testset_025_index]))
            testset_975.append(float(row[testset_975_index]))
    csvfile.close()
    return [sum(trainset_mean)/len(trainset_mean),
            sum(trainset_025)/len(trainset_025),
            sum(trainset_975)/len(trainset_975),
            sum(testset_mean)/len(testset_mean),
            sum(testset_025)/len(testset_025),
            sum(testset_975)/len(testset_975)]


def create_ablation_summary(exp_type: str):
    """Create a summary of the ablation study for a given experiment."""
    expert_values = find_expert_values(exp_type, 'ep_rew')
    min_reward = MIN_REWARD[exp_type]
    this_dir = os.path.dirname(os.path.abspath(__file__))
    # Find the first csv file in the folder "../data/train/[experiment]/expert/"
    folder = this_dir + "/data/eval/" + exp_type + "_splits/"
    reward_file = folder + "ep_rew_total.csv"
    success_file = folder + "success_total.csv"
    [trainset_reward_mean, trainset_reward_025, trainset_reward_975, testset_reward_mean, testset_reward_025, testset_reward_975] = get_ablation_data(exp_type, reward_file)
    trainset_reward_mean = (trainset_reward_mean - min_reward) / (expert_values[0] - min_reward)
    trainset_reward_025 = (trainset_reward_025 - min_reward) / (expert_values[0] - min_reward)
    trainset_reward_975 = (trainset_reward_975 - min_reward) / (expert_values[0] - min_reward)
    testset_reward_mean = (testset_reward_mean - min_reward) / (expert_values[0] - min_reward)
    testset_reward_025 = (testset_reward_025 - min_reward) / (expert_values[0] - min_reward)
    testset_reward_975 = (testset_reward_975 - min_reward) / (expert_values[0] - min_reward)
    [trainset_success_mean, trainset_success_025, trainset_success_975, testset_success_mean, testset_success_025, testset_success_975] = get_ablation_data(exp_type, success_file)
    # Write the new csv file with header:
    # metric, trainset_mean, trainset_025, trainset_975, testset_mean, testset_025, testset_975
    with open(folder + "ablation_summary.csv", 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerow(["metric", "trainset_mean", "trainset_025", "trainset_975", "testset_mean", "testset_025", "testset_975"])
        csvwriter.writerow(["Normalized reward", trainset_reward_mean, trainset_reward_025, trainset_reward_975, testset_reward_mean, testset_reward_025, testset_reward_975])
        csvwriter.writerow(["Success rate", trainset_success_mean, trainset_success_025, trainset_success_975, testset_success_mean, testset_success_025, testset_success_975])
    csvfile.close()

# --- Figure 1 ----
# -- Rewards --
# create run_reward_plots_train.sh file
f = open("run_reward_plots_" + mode + ".sh", "w")
f.write("(cd output && pdflatex -synctex=1 -interaction=nonstopmode 'legend1'.tex)\n")
for exp_type in MIN_REWARD.keys():
    f.write("(cd output && pdflatex -synctex=1 -interaction=nonstopmode 'reward_" + exp_type + "_" + mode + "'.tex)\n")
f.close()
# create tex files.
for exp_type in MIN_REWARD.keys():
    write_reward_plot_train(exp_type)

# -- Success --
# create run_success_plots_train.sh file
f = open("run_success_plots_" + mode + ".sh", "w")
for exp_type in MIN_REWARD.keys():
    f.write("(cd output && pdflatex -synctex=1 -interaction=nonstopmode 'success_" + exp_type + "_" + mode + "'.tex)\n")
f.close()
# create tex files.
for exp_type in MIN_REWARD.keys():
    write_success_plot_train(exp_type)

# --- Figure 2 ----
# create run_ablation_study.sh file
f = open("run_ablation_study.sh", "w")
f.write("(cd output && pdflatex -synctex=1 -interaction=nonstopmode ablation_study.tex)\n")
f.close()
exp_type = 'cl'
create_ablation_summary(exp_type)

this_dir = os.path.dirname(os.path.abspath(__file__))
f = open(this_dir + "/output/ablation_study.tex", "w")
f.write("% !TeX root = ablation_study\n")
f.write("% border={<left> <bottom> <right> <top>} left and right should be same.\n")
f.write("\documentclass[border={-0.05cm -0.08cm -0.00cm -0.00cm},tikz]{standalone}\n")
f.write("\input{../plotstyle}\n")
f.write("\n")
f.write("\\begin{document}\n")
f.write("	\plotAblation{" + exp_type + "}\n")
f.write("\end{document}\n")
f.close()