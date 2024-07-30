import sys
import os
import argparse
import subprocess
from pathlib import Path

from run_script import printc, bcolors

# python3 scripts/run_script.py -planners jrrt jrbt ikrrt -scenarios 011 012 013 014 015 021 022 023 024 025 031 032 033 034 035 041 042 043 044 045 -num_runs 10 -params collision_resolution 0.005 max_iters 1000000


if __name__ == "__main__":
    one_run_all = """python3 scripts/run_script.py 
--planners jrrt jrbt ikrrt 
--scenarios 011 012 013 014 015 021 022 023 024 025 031 032 033 034 035 041 042 043 044 045 
--num_runs 1
--params collision_resolution 0.005 max_iters 1000000"""

    ten_runs_all = """python3 scripts/run_script.py 
--planners jrrt jrbt ikrrt 
--scenarios 011 012 013 014 015 021 022 023 024 025 031 032 033 034 035 041 042 043 044 045 
--num_runs 10 
--params collision_resolution 0.005 max_iters 1000000"""

    hundred_runs_all = """python3 scripts/run_script.py 
--planners jrrt jrbt ikrrt 
--scenarios 011 012 013 014 015 021 022 023 024 025 031 032 033 034 035 041 042 043 044 045 
--num_runs 100
--params collision_resolution 0.005 max_iters 1000000"""

    setups = [
        one_run_all,
        ten_runs_all,
        hundred_runs_all
    ]
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Process the planners")

    parser.add_argument('--setup_id', nargs=1, type=int, default=0,
                        help='Planner timing setup')

    # Where to put bash script `all-YYYY-MM-DD-etc.sh` and directory `results-YYYY-MM-DD-etc/`
    target_dir_arg_name = "--target_dir"
    default_target_dir = "results/"
    parser.add_argument(target_dir_arg_name, nargs="?", type=str, default=default_target_dir,
                        help='Directory where to save the run script and results directory')

    args = parser.parse_args()
    if args.setup_id is None:
        printc("'--setup_id' argument is missing!", bcolors.FAIL)
        printc(
            f"See the current file {bcolors.ENDC}{bcolors.WARNING}({Path(__file__).name}){bcolors.ENDC}{bcolors.HEADER} to see available planning setups", bcolors.HEADER)
        exit(1)

    setup_id = args.setup_id[0]
    assert isinstance(setup_id, int), "args.setup_id is NOT an 'int'"
    assert setup_id < len(setups)
    # setup many runs to run in parallel
    setup_str = setups[setup_id]
    setup_str = " ".join([setup_str, target_dir_arg_name, args.target_dir])
    final_command = setup_str.split()
    print("FINAL COMMAND:\n", final_command)
    subprocess.call(final_command)
    # create run scripts
    # run the scripts

    # TODO: then run the visualization script on some of them to check
