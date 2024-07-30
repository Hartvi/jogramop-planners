from typing import *
import subprocess
import json
import os
import sys
import argparse
from animate_scene import *
from setup_runs import printc, bcolors


TXT_EXT = ".txt"
VIS_EXT = ".vis"
TRY_EXT = ".try"


def main():
    parser = argparse.ArgumentParser(
        description="Render environment with optional grasps and tree visualization.")

    parser.add_argument("--results_dir", type=str, nargs=1, required=True,
                        default=None, help="Path to the folder with results")
    parser.add_argument("--scenarios", type=str, nargs="+", required=True,
                        help="Which scenarios to animate")
    parser.add_argument("--successful", default=True, type=bool,
                        help="Whether to animate successful paths only")
    parser.add_argument("--number", default=1, type=int,
                        help="How many solutions to render for selected scenarios")
    parser.add_argument(CAMX_ARG, type=float, default=-
                        3, help="Camera X position")
    parser.add_argument(CAMY_ARG, type=float, default=2,
                        help="Camera Y position")
    parser.add_argument(CAMZ_ARG, type=float, default=3,
                        help="Camera Z position")
    parser.add_argument(GRASP_ARG, type=str, default=None,
                        help="File for grasps visualization")
    parser.add_argument(TREE_ARG, type=str, default=None,
                        help="File for tree visualization")

    args = parser.parse_args()

    print(f"Scenarios: {args.scenarios}")
    results_dir = args.results_dir[0]
    print(f"Results directory: {results_dir}")

    if args.grasp_file:
        print(f"visualizing grasps {args.grasp_file}")

    if args.tree_file:
        print(f"visualizing tree from {args.tree_file}")

    # s012-p0-s1
    results_files: List[str] = [os.path.join(results_dir, x)
                                for x in os.listdir(results_dir)]
    txt_result_files: List[str] = [x for x in results_files if TXT_EXT in x]
    chosen_result_files: List[int] = [
        i for i in range(len(txt_result_files)) if any(y in txt_result_files[i] for y in args.scenarios)]

    total_valid_videos = args.number * len(args.scenarios)
    final_animated_files = []
    # .txt as json => "sr": 1 for successful runs
    for i in chosen_result_files:
        with open(txt_result_files[i], 'r') as fp:
            jdict = json.load(fp=fp)
            # if successful then create animation
            if args.successful:
                if jdict["sr"] > 0:
                    final_animated_files.append(
                        txt_result_files[i].replace(TXT_EXT, VIS_EXT))
            else:
                final_animated_files.append(
                    txt_result_files[i].replace(TXT_EXT, VIS_EXT))
        if len(final_animated_files) >= total_valid_videos:
            break

    script_dir = os.path.dirname(__file__)
    common_command = ["python3", os.path.join(
        script_dir, "animate_scene.py"), CAMX_ARG, args.camx, CAMY_ARG, args.camy, CAMZ_ARG, args.camz, GRASP_ARG, args.grasp_file, TREE_ARG, args.tree_file]
    for i in range(len(final_animated_files)):
        tmp_command: str = " ".join(map(str, common_command)) + " " + \
            f"{FILE_ARG}" + " " + final_animated_files[i]
        printc(f"CMD: {tmp_command}", bcolors.OKCYAN)
        printc("animate_scene.py...\n\n", bcolors.OKGREEN)
        subprocess.call(tmp_command.split())


if __name__ == "__main__":
    main()
    # EXAMPLE:
    """python3 scripts/animate_scenarios.py --results_dir results/results-2024-07-20-18-24-35/ --scenarios 011 --number 10 --success 1"""
