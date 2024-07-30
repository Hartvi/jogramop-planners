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


def printc(txt: str, col: bcolors = bcolors.HEADER, **kwargs):
    print(col + txt + bcolors.ENDC, **kwargs)


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

# urdf_path = "jogramop/robots/franka_panda/mobile_panda.urdf"
urdf_path = "jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf"
vis_script_path = "scripts/animate_scene.py"

params = {}
params["-grasp"] = grasp_path
params["-urdf"] = urdf_path
params["-obstacle"] = obstacles_path
params["-start_config"] = start_config_path
params[planner_str] = 0
params["-max_iters"] = 100000
params["-d_crit"] = 0.03
params["-delta_q"] = 6.2
params["-epsilon_q"] = 0.1
params["-num_spikes"] = 7
params["-p_close_enough"] = 50.0
params["-prob_steer"] = 0.05
params["-q_resolution"] = 0.1
params[seed_str] = -1
params["-use_rot"] = 100
params["-rot_ratio"] = 0.5
params["-ground_level"] = 0.0
params["-min_col_seg_idx"] = 7
params[prefix_str] = "{}"
params["-target_configs"] = target_configs_path
params["-render_tree"] = 0
params["-collision_resolution"] = 0.005

param_prepends = "timeout {}s {}"
param_appends = "> {}.stdout"


PLANNER_NAME_TO_ID = {}
PLANNER_NAME_TO_ID["jrrt"] = 0
PLANNER_NAME_TO_ID["jrbt"] = 1
# PLANNER_NAME_TO_ID["rrt"]=2  ## USELESS
# PLANNER_NAME_TO_ID["rbt"] = 3
# PLANNER_NAME_TO_ID["rrt"] = 4
PLANNER_NAME_TO_ID["ikrrt"] = 5
# PLANNER_NAME_TO_ID["rbte"] = 6
# PLANNER_NAME_TO_ID["jrbtpos"] = 7
# PLANNER_NAME_TO_ID["jrbtposrot"] = 8
# PLANNER_NAME_TO_ID["jrbtprojrot"] = 9

TOTAL_SCENARIO_DIFFICULTIES = 5
TOTAL_SCENARIOS = 4
ALL_SCENARIOS = [("0" + str(i // 5 + 1) + str(i % 5 + 1))
                 for i in range(TOTAL_SCENARIO_DIFFICULTIES * TOTAL_SCENARIOS)]

# TODO: vary planner id, vary outputprefix, vary scenario folder, rerout output, add timeout


def get_run_stamp(scenario: str, seed: Union[str, int]):
    """
    Generates a run stamp based on a given scenario and seed.

    Parameters
    ----------
    scenario : str
        The name of the scenario.
    seed : Union[str, int]
        The seed value.

    Returns
    -------
    str
        A concatenated string of the scenario and seed.
    """
    return scenario + "_" + str(seed)


def standard_run(path_to_executable: str, tmp_params: dict, planner_id: int, scenario: str, seed: int, t: int, directory: str) -> str:
    """
    Constructs a command string for a single run of the executable with the specified parameters.

    Parameters
    ----------
    path_to_executable : str
        Path to the executable file.
    tmp_params : dict
        Dictionary of temporary parameters to override default parameters.
    planner_id : int
        ID of the planner to be used.
    scenario : str
        Name of the scenario.
    seed : int
        Seed value for the run.
    t : int
        Time limit for the run.
    directory : str
        Directory where results will be stored.

    Returns
    -------
    str
        The constructed command string.
    """
    command_str = ""

    prefix = f"{directory}s{scenario}-p{planner_id}-s{seed}"
    command_str = command_str + \
        param_prepends.format(t, path_to_executable) + " "

    for k in (params | tmp_params):
        val = ""
        # set scenario dependent parameters
        if k in scenario_dependent_params:
            val = params[k].format(scenario)
        elif prefix_str == k:
            # set prefix for output files
            val = prefix
        elif k in tmp_params:
            # print("CUSTOM PARAM:", k)
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
    """
    Generates command strings for multiple runs of the executable, iterating over different seeds.

    Parameters
    ----------
    binary_path : str
        Path to the executable file.
    tmp_params : dict
        Dictionary of temporary parameters to override default parameters.
    planner_id : int
        ID of the planner to be used.
    scenario : str
        Name of the scenario.
    n : int
        Number of runs.
    t : int
        Time limit for each run.
    directory : str
        Directory where results will be stored.

    Returns
    -------
    str
        A concatenation of command strings for all runs, separated by newlines.
    """
    return "\n".join([standard_run(binary_path, tmp_params, planner_id, scenario, i+1, t, directory) for i in range(n)])


# CUSTOM FUNCTIONS
def run_ALL_SCENARIOS(binary_path, tmp_params: dict, planners: list[str], n: int = 100, t: int = 60, directory: str = "results/"):
    """
    Generates command strings to run all scenarios for each planner specified, with multiple runs per scenario.

    Parameters
    ----------
    binary_path : str
        Path to the executable file.
    tmp_params : dict
        Dictionary of temporary parameters to override default parameters.
    planners : list[str]
        List of planners to be used.
    n : int, optional
        Number of runs per scenario. Default is 100.
    t : int, optional
        Time limit for each run. Default is 60.
    directory : str, optional
        Directory where results will be stored. Default is "results/".

    Returns
    -------
    str
        A concatenation of command strings for all scenarios and planners, separated by newlines.
    """
    assert (directory[-1] == "/")
    dirs = directory.split("/")
    os.makedirs(directory, exist_ok=True)

    command_str = ""
    for planner in planners:
        command_str += "\n".join([n_runs(binary_path, tmp_params,
                                 PLANNER_NAME_TO_ID[planner], s, n, t, directory) for s in ALL_SCENARIOS])
        command_str += "\n"
    return command_str


def run_scenarios(binary_path, tmp_params: dict, planners: list[str], scenarios: list[str], n: int = 100, t: int = 60, directory: str = "results/"):
    """
    Generates command strings to run a specified list of scenarios for each planner, with multiple runs per scenario.

    Parameters
    ----------
    binary_path : str
        Path to the executable file.
    tmp_params : dict
        Dictionary of temporary parameters to override default parameters.
    planners : list[str]
        List of planners to be used.
    scenarios : list[str]
        List of scenarios to be run.
    n : int, optional
        Number of runs per scenario. Default is 100.
    t : int, optional
        Time limit for each run. Default is 60.
    directory : str, optional
        Directory where results will be stored. Default is "results/".

    Returns
    -------
    str
        A concatenation of command strings for the specified scenarios and planners, separated by newlines.
    """
    assert (directory[-1] == "/")
    os.makedirs(directory, exist_ok=True)

    command_str = ""
    for planner in planners:
        command_str += "\n".join([n_runs(binary_path, tmp_params,
                                 PLANNER_NAME_TO_ID[planner], s, n, t, directory) for s in scenarios])
        command_str += "\n"
    return command_str


if __name__ == "__main__":
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Process the planners")

    parser.add_argument('--planners', nargs='+', help='List of planner names')
    parser.add_argument('--scenarios', nargs='+', help='List of scenario IDs')
    parser.add_argument('--num_runs', nargs=1,
                        help='Number of runs/scenario/planner')
    parser.add_argument(
        '--params', nargs='+', required=False, help="Extra parameters to pass to planners e.g. \"collision_resolution 0.005 \" ")
    parser.add_argument('--exec_path', nargs="?", type=str,
                        default="./build/jogramop-planners", help="How the binary is executed, e.g., './a.out'")
    parser.add_argument('--target_dir', nargs="?", type=str,
                        default="", help="The directory where to save the run script and results directory")

    # Parse the arguments
    args = parser.parse_args()

    # Store the planners in a list, if any planners were provided
    args_ok = True
    my_planners = args.planners if args.planners is not None else []
    if not my_planners:
        printc("NO --planners SELECTED", bcolors.FAIL)
        args_ok = False

    my_scenarios = args.scenarios if args.scenarios is not None else []
    if not my_scenarios:
        printc("NO --scenarios SELECTED", bcolors.FAIL)
        args_ok = False

    if args.num_runs is not None:
        my_num_runs = int(args.num_runs[0])
        print("NUM RUNS:", my_num_runs)
        if not my_num_runs:
            printc("NO --num_runs SELECTED", bcolors.FAIL)
            args_ok = False
    else:
        printc("NO --num_runs SELECTED", bcolors.FAIL)
        args_ok = False

    my_params = args.params
    if not my_params:
        printc("OPTIONAL: NO --params SELECTED", bcolors.WARNING)
        my_params = {}
    else:
        # Params in the form of "-name1 value1 -name2 value2"
        my_params = dict([("-" + my_params[i], my_params[i + 1])
                          for i in range(0, len(my_params) - 1, 2)])
        printc(f"OPTIONAL: MY PARAMS {my_params}")

    if not args_ok:
        printc("ERROR: args not ok", bcolors.FAIL)
        exit(1)

    timestamp_str: str = get_timestamp()
    my_binary_path = args.exec_path
    my_dir = os.path.join(args.target_dir, f"results-{timestamp_str}/")
    my_commands = os.path.join(args.target_dir, f"all-{timestamp_str}.sh")

    # IF TARGET DIRECTORY DOESN'T EXIST THEN TRY TO CREATE IT
    target_dir_name = os.path.dirname(my_commands)
    if not os.path.isdir(target_dir_name):
        printc(
            f"{os.getcwd()}/{target_dir_name}/ DIRECTORY DOES NOT EXIST", bcolors.FAIL)
        print(f"Create directory '{target_dir_name}/'? (Y/n)")
        inp = input()
        if inp.lower() == "y" or inp == "":
            os.mkdir(target_dir_name)
            printc(f"Created dir '{target_dir_name}'", bcolors.OKGREEN)
        else:
            print(f"Did not create dir '{target_dir_name}'")
            exit(1)

    f = open(my_commands, "w")
    f.write(run_scenarios(my_binary_path,
            my_params, my_planners, my_scenarios, n=my_num_runs, directory=my_dir))
    f.close()

    print(f"{bcolors.WARNING}-----------------------------------{bcolors.ENDC}")
    print(f" {bcolors.OKBLUE}Created run script:{bcolors.ENDC}",
          f"'{my_commands}'")
    print(f" {bcolors.OKCYAN}Created directory: {bcolors.ENDC}", f"'{my_dir}'",
          f"\n {bcolors.OKGREEN}Total runs scheduled: {bcolors.ENDC}", my_num_runs *
          len(my_scenarios)*len(my_planners),
          f"\n {bcolors.OKBLUE}Planners: {bcolors.ENDC}", *my_planners,
          f"\n {bcolors.OKCYAN}Scenarios: {bcolors.ENDC}", *my_scenarios,)
    print(f"{bcolors.WARNING}-----------------------------------{bcolors.ENDC}")
