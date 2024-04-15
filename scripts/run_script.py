import os
import sys
from typing import *
import argparse
import datetime


def get_timestamp() -> str:
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y-%m-%d-%H-%M-%S")
    return timestamp


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


scenarios_dir = "jogramop/scenarios/"

grasp_path = scenarios_dir + "{}/export/grasps.csv"
obstacles_path = scenarios_dir + "{}/export/obstacles.obj"
start_config_path = scenarios_dir + "{}/export/robot_start_conf.csv"
target_configs_path = scenarios_dir + "{}/export/grasp_IK_solutions.csv"

scenario_dependent_params = [
    "-grasp", "-obstacle", "-start_config", "-target_configs"]

prefix_str = "-target_prefix"
planner_str = "-planner"
seed_str = "-seed"

urdf_path = "jogramop/robots/franka_panda/mobile_panda.urdf"
vis_script_path = "scripts/animate_scene.py"

params = {}
params["-grasp"] = grasp_path
params["-urdf"] = urdf_path
params["-obstacle"] = obstacles_path
params["-start_config"] = start_config_path
params[planner_str] = 0
params["-max_iters"] = 100000
params["-d_crit"] = 0.1
params["-delta_q"] = 3.1415
params["-epsilon_q"] = 0.1
params["-num_spikes"] = 7
params["-p_close_enough"] = 50.0
params["-prob_steer"] = 0.01
params["-q_resolution"] = 0.1
params[seed_str] = -1
params["-use_rot"] = 100
params["-rot_ratio"] = 0.5
params["-groundLevel"] = 0.0
params["-minColSegIdx"] = 6
params[prefix_str] = "{}"
params["-target_configs"] = target_configs_path
params["-render"] = 0
params["-vis_script"] = vis_script_path
params["-cx"] = -3
params["-cy"] = 3
params["-cz"] = 5
params["-render_tree"] = 0
params["-collision_resolution"] = 0.00

param_prepends = "timeout {}s {}"
param_appends = "> {}.stdout"


PLANNER_NAME_TO_ID = {}
PLANNER_NAME_TO_ID["jrrt"] = 0
PLANNER_NAME_TO_ID["jrbt"] = 1
# PLANNER_NAME_TO_ID["rrt"]=2  ## USELESS
PLANNER_NAME_TO_ID["rbt"] = 3
PLANNER_NAME_TO_ID["rrt"] = 4
PLANNER_NAME_TO_ID["ikrrt"] = 5
PLANNER_NAME_TO_ID["rbte"] = 6

TOTAL_SCENARIO_DIFFICULTIES = 5
TOTAL_SCENARIOS = 4
ALL_SCENARIOS = [("0" + str(i // 5 + 1) + str(i % 5 + 1))
                 for i in range(TOTAL_SCENARIO_DIFFICULTIES * TOTAL_SCENARIOS)]

# TODO: vary planner id, vary outputprefix, vary scenario folder, rerout output, add timeout


def get_run_stamp(scenario: str, seed: Union[str, int]):
    return scenario + "_" + str(seed)


def standard_run(binary_path: str, tmp_params: dict, planner_id: int, scenario: str, seed: int, t: int, directory: str) -> str:
    """
    seed: random seed
    t: time limit
    """
    command_str = ""

    prefix = f"{directory}s{scenario}-p{planner_id}-s{seed}"
    command_str = command_str + param_prepends.format(t, binary_path) + " "

    for k in (params | tmp_params):
        val = ""
        # set scenario dependent parameters
        if k in scenario_dependent_params:
            val = params[k].format(scenario)
        elif prefix_str == k:
            # set prefix for output files
            val = prefix
        elif k in tmp_params:
            print("CUSTOM PARAM:", k)
            # use the custom parameter over the default parameter
            val = tmp_params[k]
        elif k == planner_str:
            val = str(planner_id)
        elif k == seed_str:
            val = seed
        else:
            # otherwise use the default parameter
            val = params[k]
        command_str = command_str + k + " " + str(val) + " "

    command_str = command_str + \
        param_appends.format(prefix) + " "
    return command_str


def n_runs(binary_path, tmp_params: dict, planner_id: int, scenario: str, n: int, t: int, directory: str):
    return "\n".join([standard_run(binary_path, tmp_params, planner_id, scenario, i+1, t, directory) for i in range(n)])


# CUSTOM FUNCTIONS
def run_ALL_SCENARIOS(binary_path, tmp_params: dict, planners: list[str], n: int = 100, t: int = 30, directory: str = "results/"):
    assert (directory[-1] == "/")
    dirs = directory.split("/")
    os.makedirs(directory, exist_ok=True)

    command_str = ""
    for planner in planners:
        command_str += "\n".join([n_runs(binary_path, tmp_params,
                                 PLANNER_NAME_TO_ID[planner], s, n, t, directory) for s in ALL_SCENARIOS])
        command_str += "\n"
    return command_str


def run_scenarios(binary_path, tmp_params: dict, planners: list[str], scenarios: list[str], n: int = 100, t: int = 30, directory: str = "results/"):
    assert (directory[-1] == "/")
    os.makedirs(directory, exist_ok=True)

    command_str = ""
    for planner in planners:
        command_str += "\n".join([n_runs(binary_path, tmp_params,
                                 PLANNER_NAME_TO_ID[planner], s, n, t, directory) for s in scenarios])
        command_str += "\n"
    return command_str


# Example usage of run_planners
if __name__ == "__main__":
    # Initialize the argparse.ArgumentParser
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Process the planners")

    # Add the 'planners' argument
    # The 'nargs="+"' configuration allows for one or more arguments to be consumed
    parser.add_argument('-planners', nargs='+', help='List of planner names')
    parser.add_argument('-scenarios', nargs='+', help='List of scenario IDs')
    parser.add_argument('-num_runs', nargs=1,
                        help='Number of runs/scenario/planner')
    parser.add_argument(
        '-params', nargs='+', required=False, help="Extra parameters to pass to planners e.g. \"collision_resolution 0.005 \" ")

    # Parse the arguments
    args = parser.parse_args()

    # Store the planners in a list, if any planners were provided
    my_planners = args.planners if args.planners is not None else []
    if not my_planners:
        print("NO -planners SELECTED")
        exit(1)

    my_scenarios = args.scenarios if args.scenarios is not None else []
    if not my_scenarios:
        print("NO -scenarios SELECTED")
        exit(1)

    my_num_runs = int(args.num_runs[0])
    if not my_num_runs:
        print("NO -num_runs SELECTED")
        exit(1)

    my_params = args.params
    if not my_params:
        print("OPTIONAL: NO -params SELECTED")
        my_params = {}
    else:
        my_params = dict([("-"+my_params[i], my_params[i+1])
                          for i in range(0, len(my_params)-1, 2)])

    print("MY PARAMS", my_params)
    timestamp_str: str = get_timestamp()

    my_binary_path = "./build/jogramop-planners test"
    my_dir = f"results-{timestamp_str}/"
    my_commands = f"all{timestamp_str}.sh"

    f = open(my_commands, "w")
    f.write(run_scenarios(my_binary_path,
            my_params, my_planners, my_scenarios, n=my_num_runs, directory=my_dir))
    f.close()

    print(f"{bcolors.WARNING}-----------------------------------{bcolors.ENDC}")
    print(f" {bcolors.OKBLUE}Create run script:{bcolors.ENDC}",
          f"'{my_commands}'")
    print(f" {bcolors.OKCYAN}Created directory: {bcolors.ENDC}", f"'{my_dir}'",
          f"\n {bcolors.OKGREEN}Total runs scheduled: {bcolors.ENDC}", my_num_runs *
          len(my_scenarios)*len(my_planners),
          f"\n {bcolors.OKBLUE}Planners: {bcolors.ENDC}", *my_planners,
          f"\n {bcolors.OKCYAN}Scenarios: {bcolors.ENDC}", *my_scenarios,)
    print(f"{bcolors.WARNING}-----------------------------------{bcolors.ENDC}")
