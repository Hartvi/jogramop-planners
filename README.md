# Burs

## Installation

1. Clone this repository into a directory
2. Clone [PQP](https://github.com/GammaUNC/PQP) into the same directory
  - because in `CMakeLists.txt` there is: `add_subdirectory(../PQP PQP_build)` 
  - or move it where you like and change the `CMakeLists.txt` as well
3. Install the following libraries `urdfdom` and `KDL`:
```
sudo apt-get install liburdfdom-dev
sudo apt-get install libkdl-parser-dev
```
4. Then you can build it
```
mkdir build
cd build
cmake ..
make
```
5. Run planners using one of the `P{i}_planner.sh` scripts. There you can set parameters as you need.
6. Run `./show_options.sh` to show all parameters options and their meanings

