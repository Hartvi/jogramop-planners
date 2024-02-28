# Baseline planners

## Installation

1. Clone this repository into a directory
```
git clone git@github.com:Hartvi/jogramop-planners.git
```
2. Clone [PQP](https://github.com/GammaUNC/PQP) into the same directory
  - because in `CMakeLists.txt` there is: `add_subdirectory(../PQP PQP_build)` 
  - or move it where you like and change the `CMakeLists.txt` as well
```
git clone git@github.com:GammaUNC/PQP.git
```
3. Install the following libraries:
- **urdfdom** - `sudo apt-get install liburdfdom-dev`
- **KDL** - `sudo apt-get install libkdl-parser-dev`
- **Eigen** https://gitlab.com/libeigen/eigen/ / `sudo apt install libeigen3-dev`
  - The version used is 3.3.7-2
- **FLANN** https://github.com/flann-lib/flann / `sudo apt install libflann-dev`
- **LZ4** https://github.com/lz4/lz4 / `sudo apt install liblz4-dev`
4. Clone [jogramop](https://github.com/mrudorfer/jogramop) into this directory
```
cd jogramop-planners
git clone git@github.com:mrudorfer/jogramop.git
```
4. Then you can build it
```
mkdir build
cd build
cmake ..
make
```
5. Run planners using one of the `P{i}_{planner}.sh` scripts. There you can set parameters as you need.
6. Run `./show_options.sh` to show all parameters options and their meanings

The code was tested on Ubuntu 20.04, with C++17.

## Output format
The planner outputs three files (with a prefix specified by -target_prefix PREFIX
- `PREFIX.try` - this the trajectory, a list of configurations on each line
- `PREFIX.vis` - the file containing the Frame of the obstacles and sobot segments to visualize the path in 3D
- `PREFIX.txt` - this file contains measurements. Success, distance to target, time, number of iterations, tree size. e.g. { "sr":1,"dtg":43.0474,"time":0.130713,"iters":118,"treesize": 120}

## How to run experiments
- **NOTE**: At the moment only available on the `master` branch (main branch coming soon). Run `git checkout master` to perform experiments.

For statistical evaluation of the planners, it's necessary to run the planners several time (on each scene or for each planner settings).
The basic testing can be achieved in this way:

> python3 runExperiments.py  

The script will create all-cmds.sh, which contains command lines for each planner/scenario. 
The planners can be run either sequentially (i.e., running `sh all-cmds.sh`), or by parallel

> parallel < all-cmds.sh

After experiments finish, the resuls are in the results/ directory.
Basic processing (drawing success rate curves into png) can be realized by>

> python3 makeSucessRate



