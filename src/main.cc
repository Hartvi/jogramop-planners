// tinyobjloader must be first
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <iostream>
// #include <stdio.h>
#include <Eigen/Dense>
#include "pqp_load.h"
#include "rt_model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "base_planner.h"
#include "test.h"

using namespace std;

int main(int argc, char **argv)
{
    // run: make clean && make && valgrind --leak-check=full ./load_obj_test ../Models/cube.obj ../Models/cube.obj out.txt

    std::cout << "Arg 1: " << argv[1] << std::endl;
    std::string arg1 = std::string(argv[1]);
    if (arg1 == "test")
    {
        std::cout << "ARGUMENT 1 WAS TEST" << std::endl;
        // TODO: test: load test yaml or sth
        // test::test_forward(argc, argv);
        test::main_test();
        std::cout << "END TEST" << std::endl;
    }
    std::cout << "argc: ";
    std::cout << argc;
    std::cout << "\n";

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
