# Burs

## TODO:
- Test it with jogramop environment
- Already doing this: use relative steps in q because of different min-max values for each joint
- Done: interpolate path by small steps
- Done: create python binding
  - expose:
    - robot class: robot_class(urdf_path), add_obstacle(path, R, t), plan(start, goal)

- Done: add function to accept `model path`, `R`, `t` to add as a scenario
- Done: formalize correctly the forward kinematic functions
  - this should enable me to check if the algorithm has been implemented correctly


`sudo apt-get install liburdfdom-dev`
`sudo apt-get install libkdl-parser-dev`

`./burs_of_free_space ../../Models/stick_robot.obj ../../Models/cube.obj 5 5 5 ../jogramop/robots/franka_panda/panda.urdf `
