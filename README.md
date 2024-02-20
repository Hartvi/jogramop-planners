# Burs

## Installation

1. Clone this repository into a directory
2. Clone [PQP](https://github.com/GammaUNC/PQP) into the same directory
  - In CMakeLists.txt: `add_subdirectory(../PQP PQP_build)` 
  - Or move it where you like and change the CMakeLists as well
3. Install the following libraries `urdfdom` and `KDL`
```
sudo apt-get install liburdfdom-dev
sudo apt-get install libkdl-parser-dev
```
4. then you can build it
```
mkdir build
cd build
cmake ..
make
```
5. Run using one of the `P{i}_planner.sh` scripts


