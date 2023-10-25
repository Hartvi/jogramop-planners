// tinyobjloader must be first
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <iostream>
// #include <stdio.h>
#include <Eigen/Dense>
#include "load.h"
#include "model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "bur_algorithm.h"
#include "test.h"

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

    // auto model1 = std::make_shared<TrPQPModel>(argv[1]);

    // double world_bound = 10;
    // std::string name(argv[3]);

    return 0;
}
