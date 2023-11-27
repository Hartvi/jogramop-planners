// tinyobjloader must be first
#define TINYOBJLOADER_IMPLEMENTATION
#include "model_related/tiny_obj_loader.h"

#include <iostream>
// #include <stdio.h>
#include <Eigen/Dense>
#include "model_related/pqp_load.h"
#include "model_related/rt_model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_related/bur_tree.h"
#include "bur_related/base_planner.h"
#include "test_related/test.h"

using namespace std;

int main(int argc, char **argv)
{
    // run: make clean && make && valgrind --leak-check=full ./load_obj_test ../Models/cube.obj ../Models/cube.obj out.txt
    std::cout << "argc: ";
    std::cout << argc;
    std::cout << "\n";

    test::test_forward(argc, argv);

    // if (argc < 5)
    // {
    //     std::cout << "Please enter two paths to an obj model to load and an output text file path and number of tree nodes" << std::endl;
    //     return 1;
    // }

    // auto model1 = std::make_shared<RtModel>(argv[1]);

    // double world_bound = 10;
    // std::string name(argv[3]);

    return 0;
}
