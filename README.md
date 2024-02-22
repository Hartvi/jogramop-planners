# Burs

## Installation

1. Clone this repository into a directory
```
git clone git@github.com:Hartvi/Burs.git
```
2. Clone [PQP](https://github.com/GammaUNC/PQP) into the same directory
  - because in `CMakeLists.txt` there is: `add_subdirectory(../PQP PQP_build)` 
  - or move it where you like and change the `CMakeLists.txt` as well
```
git clone git@github.com:GammaUNC/PQP.git
```
3. Install the following libraries  `` and `KDL`:
- **urdfdom** - `sudo apt-get install liburdfdom-dev`
- **KDL** - `sudo apt-get install libkdl-parser-dev`
- **Eigen** if you don't have it: ```sudo apt install libeigen3-dev```
  - The version used is 3.3.7-2
- **FLANN** https://github.com/flann-lib/flann, `sudo apt install libflann-dev`
- **LZ4** https://github.com/lz4/lz4, `sudo apt install liblz4-dev`
4. Clone [jogramop](https://github.com/mrudorfer/jogramop) into the Burs directory
```
cd Burs
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

The code was tested on Ubuntu 20.04

## Output format
 `.try`, `.vis`, `.txt`
