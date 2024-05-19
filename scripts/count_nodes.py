import numpy as np
from matplotlib import pyplot as plt

from plot_runs import *

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
    # planners = ["jrbt", "jrrt"]
    # planners = ["jrbt", "jrbtposrot"]
    # planners = ["ikrrt"]
    linestyles = ["-.", "--", ":"]
    planner_colours = [(1, 0, 0), (1, 0.5, 0), (0, 0.8, 0)]

    for planner in planners:
        planners_on_scenarios = get_planners_on_scenarios(
            [planner], ALL_SCENARIOS, txts)
        # print("planner", planner, "on scenarios:", planners_on_scenarios)
        print("planner", planner)

        for l, p in enumerate(planners_on_scenarios):
            # print("PLANNER:", p)

            times_vals = np.zeros((len(p), 2))
            for k, n in enumerate(p):
                with open(os.path.join(my_results, n)) as f:
                    d = json.load(f)
                    times_vals[k, 0] = d["time"]
                    times_vals[k, 1] = d["treesize"]
            means = np.mean(times_vals, axis=0)
            # print(means)
            print("NODE GROWTH:", means[1] / means[0])
            # print("PLANNER:", planners[l], "\n")
            # plot_vals = times_vals[np.argsort(times_vals[:, 0])]
            # plot_vals[:, 1] = np.cumsum(plot_vals[:, 1]) / len(p) * 100
            # ax.plot(np.concatenate([[0], plot_vals[:, 0]]), np.concatenate([[0], plot_vals[:, 1]]),
            #         label=planners[l], linestyle=linestyles[l % len(linestyles)], linewidth=2)
            # ax.set_xlabel("Time [s]")
            # ax.set_ylabel("Success rate [%]")
