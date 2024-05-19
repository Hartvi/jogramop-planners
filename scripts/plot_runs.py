import numpy as np
import json
import os
import sys
from typing import *
import argparse

from matplotlib import pyplot as plt

from run_script import PLANNER_NAME_TO_ID, ALL_SCENARIOS


def get_planner(planner_id: str, filelist: list[str]):
    return [i for i in filelist if (f"p{planner_id}" in i)]


def get_scenario(scenario_id: str, filelist: list[str]):
    return [i for i in filelist if (f"s{scenario_id}" in i)]


def get_extension(extension: str, filelist: list[str]):
    return [i for i in filelist if i[-len(extension):] == extension]


def get_planners_on_scenarios(planner_ids: list[str], scenario_ids: list[str], filelist: list[str]):
    columns = [[] for _ in range(len(planner_ids))]
    # height: number of runs, width: number of planners
    for i in range(len(planner_ids)):
        for k in range(len(scenario_ids)):
            scenario_results = get_scenario(scenario_ids[k], filelist)
            columns[i].extend(get_planner(
                PLANNER_NAME_TO_ID[planner_ids[i]], scenario_results))
    return columns


if __name__ == "__main__":

    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Process the results")

    parser.add_argument('-results', nargs=1,
                        help='Results directory')

    args = parser.parse_args()
    if not args.results:
        print("ENTER RESULTS DIRECTORY e.g. `-results path/to/results/dir`")
        exit(1)
    my_results = args.results[0]

    filelist = os.listdir(my_results)
    txt_extension = ".txt"
    txts = get_extension(txt_extension, filelist)
    planners = ["jrrt", "jrbt", "ikrrt"]
    # planners = ["jrbt", "jrbtposrot"]
    # planners = ["ikrrt"]
    linestyles = ["-.", "--", ":"]
    planner_colours = [(1, 0, 0), (1, 0.5, 0), (0, 0.8, 0)]
    for scenario in ALL_SCENARIOS:
        planners_on_scenarios = get_planners_on_scenarios(
            planners, [scenario], txts)

        # skip scenarios that haven't been run
        try:
            if np.array(planners_on_scenarios).flatten().size == 0:
                continue
        except:
            print("TRYING TO PLOT WRONG RUNS: ")
            print("EXCEPTION:", planners_on_scenarios)
            exit(1)

        fig, ax = plt.subplots()

        result_paths = []
        for l, p in enumerate(planners_on_scenarios):
            print("PLANNER:", p)

            times_vals = np.zeros((len(p), 2))
            for k, n in enumerate(p):
                with open(os.path.join(my_results, n)) as f:
                    d = json.load(f)
                    times_vals[k, 0] = d["time"]
                    times_vals[k, 1] = d["sr"]
            print("PLANNER:", planners[l], "\n")
            plot_vals = times_vals[np.argsort(times_vals[:, 0])]
            plot_vals[:, 1] = np.cumsum(plot_vals[:, 1]) / len(p) * 100
            ax.plot(np.concatenate([[0], plot_vals[:, 0]]), np.concatenate([[0], plot_vals[:, 1]]),
                    label=planners[l], linestyle=linestyles[l % len(linestyles)], linewidth=2)
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Success rate [%]")

        ax.legend(loc='upper right')  # Changed here
        ax.set_title("Scenario "+str(scenario))
        fig.savefig(os.path.join(my_results, "scenario-"+str(scenario)+".png"))
    planners_on_scenarios = get_planners_on_scenarios(
        planners, ALL_SCENARIOS, txts)

    fig, ax = plt.subplots()

    result_paths = []
    for l, p in enumerate(planners_on_scenarios):
        print("PLANNER:", p)

        times_vals = np.zeros((len(p), 2))
        for k, n in enumerate(p):
            with open(os.path.join(my_results, n)) as f:
                d = json.load(f)
                times_vals[k, 0] = d["time"]
                times_vals[k, 1] = d["sr"]
        print("PLANNER:", planners[l], "\n")
        plot_vals = times_vals[np.argsort(times_vals[:, 0])]
        plot_vals[:, 1] = np.cumsum(plot_vals[:, 1]) / len(p) * 100
        ax.plot(np.concatenate([[0], plot_vals[:, 0]]), np.concatenate([[0], plot_vals[:, 1]]),
                label=planners[l], linestyle=linestyles[l % len(linestyles)], linewidth=2)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Success rate [%]")

    ax.legend(loc='upper left')  # Changed here
    fig.savefig(os.path.join(my_results, "scenario-ALL.png"))
