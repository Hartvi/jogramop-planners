# TODOs
- write tests for .obj loading:
  - add the option to offset the robot by some 4x4 transformation
            <!-- [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.05],
            [0.0, 0.0, 0.0, 1.0], -->
  - Scenarios are not centered atm, loading them from BURG turns out shifted off-center
  - add it as an argument 'test'
  - check if collisions work between a .obj file that contains multiple sub-objects
  - check that multiple objects in the .obj file are loaded as multiple meshes into the environment
  - 
- make it possible to load scenes from BURG directly using its description
- rewrite RBT and make it cleaner
- clean up C++ code
  - DONE: remove runscripts for useless planners

## DONE
- write a tutorial in README how to run the planners