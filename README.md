# Burs

## How to run
Assuming you have CMake setup for this project:
- requires PQP library in directory above `burs_of_free_space/` and others installed according to the `CMakeLists.txt`
```
find_package(urdfdom REQUIRED)
# find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


# Include directories
include_directories(include /usr/local/include /usr/include /usr/local/include/eigen3)

# Add subdirectory for PQP
add_subdirectory(../PQP PQP_build)
```

```
mkdir build
cd build
cmake ..
make
```
- Run test: `./burs_of_free_space test`


## TODO:
- Verify correctness of algorithm
- `bur_related/bur_tree.h`: 
  - Update only changed parts of tree
  - Prevent double deletion
  - Maybe use smart pointers instead of basic `new` and `delete`
- Add pseudo-inverse Jacobian-directed bur creation
- For `TransX`, `TransY`, `TransZ` joints, the radius should just be the projection of the vector from joint `i` to joints `i+k`, `k=1..n-i` on the joint axis, not on the plane of an axis orthogonal to the joint axis
- Test it with jogramop environment

### Done:
- Done: Already doing this: use relative steps in q because of different min-max values for each joint
- Done: interpolate path by small steps
- Done: create python binding
  - expose:
    - robot class: `robot_class(urdf_path)`, `add_obstacle(path, R, t)`, `plan(start, goal)`

- Done: add function to accept `model path`, `R`, `t` to add as a scenario
- Done: formalize correctly the forward kinematic functions
  - this should enable me to check if the algorithm has been implemented correctly


```
sudo apt-get install liburdfdom-dev
sudo apt-get install libkdl-parser-dev
```

`./burs_of_free_space ../../Models/stick_robot.obj ../../Models/cube.obj 5 5 5 ../jogramop/robots/franka_panda/panda.urdf `
