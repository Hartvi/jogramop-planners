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
#include "CParseArgs.h"


using namespace std;

int main(int argc, char **argv)
{

    // PARAMETERS of the command line. For each parameter (e.g. -file), make one variable and one o.addOption<type>(), see bellow example:

    char *graspFile;
    char *urdfFile;
    char *obstacleFile;
    {
        CmdOptions o;

        o.addOption(Option<char *>("grasp",&graspFile, "filename with grasps (.csv)"));
        o.addOption(Option<char *>("urdf",&urdfFile, "filename with URDF of the robot (.urdf)"));
        o.addOption(Option<char *>("obstacle",&obstacleFile, "filename with obstacles (.obj)"));
		if (!o.parse(argc,argv)) {
			cerr << o.makeCmdLine() << "\n";
			cerr << o.printHelp() << "\n";
			exit(0);
		}
    }

    // the cmd-line parameters are now loaded into the variables
    std::cout << "Planner will load: \n";
    std::cout << "Grasps from " << graspFile << "\n";
    std::cout << "URDF from " << urdfFile << "\n";
    std::cout << "Obstacles from " << obstacleFile << "\n";

    // run: make clean && make && valgrind --leak-check=full ./load_obj_test ../Models/cube.obj ../Models/cube.obj out.txt

    std::cout << "Arg 1: " << argv[1] << std::endl;
    std::string arg1 = std::string(argv[1]);
    if (arg1 == "test")
    {
        std::cout << "ARGUMENT 1 WAS TEST" << std::endl;
        // TODO: test: load test yaml or sth
        // test::test_forward(argc, argv);
        test::main_test(graspFile, urdfFile, obstacleFile);
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
