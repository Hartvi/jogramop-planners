
// tinyobjloader must be first
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <ctime>
#include <cstdlib>
#include <iostream>
// #include <stdio.h>
#include <Eigen/Dense>
#include "pqp_load.h"
#include "rt_model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "base_planner.h"
// #include "test.h"
#include "CParseArgs.h"
#include "j_rbt_planner.h"
#include "ut.h"
// #include "j_plus_rbt_planner.h"

using namespace std;
using namespace Burs;

std::string joinWithCurrentDirectory(const std::string &filename)
{
    std::filesystem::path currentDir = std::filesystem::current_path();
    std::filesystem::path fullPath = currentDir / filename;
    return fullPath.string();
}

std::string getCurrentTimestamp()
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm buf;
    localtime_r(&in_time_t, &buf); // Using localtime_r for thread safety

    // Format the time as a string
    std::ostringstream ss;
    ss << std::put_time(&buf, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

int main(int argc, char **argv)
{
    // IMPORTANT
    // set time-dependent seed for Eigen random matrix generation so it doesn't run the same every time

    // PARAMETERS of the command line. For each parameter (e.g. -file), make one variable and one o.addOption<type>(), see bellow example:

    char *graspFile;
    char *urdfFile;
    char *obstacleFile;
    char *startConfigFile;
    int plannerType;
    int seed;

    int maxIters;
    double dCrit;
    double deltaQ;
    double epsilonQ;
    int numSpikes;
    double pCloseEnough;
    double pSteerToTarget;

    char *targetPrefixFile;

    double groundLevel;
    int minColSegmentIdx; // minimum segment idx able to collide with ground

    char *targetConfigsFile;

    int ikSolIndex;
    double qResolution;

    int useRotation;
    double rotationDistRatio;

    {
        CmdOptions o;

        o.addOption(Option<char *>("grasp", &graspFile, "filename with grasps (.csv)"));
        o.addOption(Option<char *>("urdf", &urdfFile, "filename with URDF of the robot (.urdf)"));
        o.addOption(Option<char *>("obstacle", &obstacleFile, "filename with obstacles (.obj)"));
        o.addOption(Option<char *>("start_config", &startConfigFile, "filename with initial configuration (.csv)"));
        o.addOption(Option<int>("planner", &plannerType, "planner to choose (int)"));
        o.addOption(Option<int>("max_iters", &maxIters, "max number of iterations"));
        o.addOption(Option<double>("d_crit", &dCrit, "critical distance under which to switch to basic RRT"));
        o.addOption(Option<double>("delta_q", &deltaQ, "max distance for bur extensions"));
        o.addOption(Option<double>("epsilon_q", &epsilonQ, "step size RRT"));
        o.addOption(Option<int>("num_spikes", &numSpikes, "number of bur spikes"));
        o.addOption(Option<double>("p_close_enough", &pCloseEnough, "end-effector is close enough to target"));
        o.addOption(Option<double>("prob_steer", &pSteerToTarget, "end-effector is close enough to target"));

        o.addOption(Option<char *>("target_configs", &targetConfigsFile, "defaultValue", "target IK solutions for the grasps"));
        o.addOption(Option<double>("groundLevel", &groundLevel, "ground z coodinate"));
        o.addOption(Option<int>("minColSegIdx", &minColSegmentIdx, "segment id from which it can collide with ground"));

        o.addOption(Option<int>("ik_index", &ikSolIndex, 0, "max iters for all ik solutions")); // default value is 0
        o.addOption(Option<double>("q_resolution", &qResolution, "resolution of individual steps in rbt"));

        o.addOption(Option<int>("use_rot", &useRotation, 0, "rotation bias threshold when to start using it (mm+deg)"));
        o.addOption(Option<double>("rot_ratio", &rotationDistRatio, 0.5, "ratio of rotation in distance metric (mm+deg)"));

        o.addOption(Option<char *>("target_prefix", &targetPrefixFile, "file in which to save measurements, separated by keywords"));

        o.addOption(Option<int>("seed", &seed, -1, "random seed or time (if seed = -1)")); // default value is -1 -> seed is from time

        if (!o.parse(argc, argv))
        {
            cerr << o.makeCmdLine() << "\n";
            cerr << o.printHelp() << "\n";
            exit(0);
        }
    }
    int usedSeed = seed;
    if (seed == -1)
    {
        usedSeed = std::time(NULL);
        std::srand(usedSeed);
    }
    else
    {
        std::srand(seed);
    }
    std::cout << "setting seed " << seed << ", usedSeed: " << usedSeed << "\n";
    std::cout << "CMDLINE params:\n";
    for (int i = 0; i < argc; i++)
    {
        std::cout << argv[i] << "\n";
    }

    // the cmd-line parameters are now loaded into the variables
    std::cout << "Planner will load: \n";
    std::cout << "Grasps from " << graspFile << "\n";
    std::cout << "URDF from " << urdfFile << "\n";
    std::cout << "Obstacles from " << obstacleFile << "\n";
    std::cout << "Planner: " << plannerType << "\n";

    std::shared_ptr<JRbtPlanner> jprbt = std::make_shared<JRbtPlanner>(std::string(urdfFile));
    // 1. Set obstacles in urdfenv
    // 2. Setup parameters
    // 3. Plan
    auto &env = jprbt->env;
    env->AddObstacle(obstacleFile);
    env->SetGroundLevel(groundLevel, minColSegmentIdx);

    std::string grasp_path(graspFile);

    std::vector<Grasp> grasps = Grasp::LoadGrasps(grasp_path);

    Eigen::VectorXd start_config = RobotBase::parseCSVToVectorXd(startConfigFile);

    PlanningResult planning_result;

    std::optional<std::vector<Eigen::VectorXd>> path;
    std::vector<Eigen::VectorXd> final_path;

    JPlusRbtParameters params(maxIters, dCrit, deltaQ, epsilonQ, numSpikes, pCloseEnough, pSteerToTarget, grasps, qResolution);
    params.visualize_tree = 0;
    params.seed = usedSeed;
    params.preheat_ratio = 0;
    params.preheat_type = 0;

    params.use_rotation = useRotation;
    params.rotation_dist_ratio = rotationDistRatio;
    params.bias_calculation_type = 0;

    { // output .txt file with results also before the planner runs, so in the case of planner failure/killing the program, there
        // will be (empty) outputfile, which indicates the failure
        char fname[2000];
        snprintf(fname, sizeof(fname), "%s.txt", targetPrefixFile);
        ofstream ofs(fname);
        // ofs << planning_result.toCSVString() << "\n";
        ofs << planning_result.toJSON() << "\n";
        ofs.close();
    }

    switch (plannerType)
    {
    case 0:
    {
        std::cout << "PLANNING J+ BIASED RRT\n";

        struct rusage t1, t2;
        getTime(&t1);
        path = jprbt->JRRT(start_config, params, planning_result);
        getTime(&t2);

        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 1:
    {
        std::cout << "PLANNING J+RBT basic\n";

        struct rusage t1, t2;
        getTime(&t1);
        path = jprbt->JRbtBasic(start_config, params, planning_result);

        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 2:
    {
        std::cout << "PLANNING IK RRT\n";

        std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

        struct rusage t1, t2;
        getTime(&t1);
        if (goals.size() == 0)
        {
            std::cout << "NO INVERSE KINEMATICS SOLUTIONS PRESENT\n\n";
            exit(1);
        }
        path = jprbt->RRTConnect(start_config, goals[ikSolIndex], params, planning_result);
        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 3:
    {
        std::cout << "PLANNING IK RBT\n";

        std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

        if (goals.size() == 0)
        {
            std::cout << "NO INVERSE KINEMATICS SOLUTIONS PRESENT\n\n";
            exit(1);
        }
        struct rusage t1, t2;
        getTime(&t1);
        path = jprbt->RbtConnectDenseBurs(start_config, goals[ikSolIndex], params, planning_result);
        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    default:
    {
        break;
    }
    } // end switch

    char fname[2000];
    {
        snprintf(fname, sizeof(fname), "%s.txt", targetPrefixFile);
        ofstream ofs(fname);
        ofs << planning_result.toJSON() << "\n";
        ofs.close();
    }
    {
        snprintf(fname, sizeof(fname), "%s.try", targetPrefixFile);
        ofstream ofs(fname);
        ofs << jprbt->ConfigsToString(final_path) << "\n";
        ofs.close();
    }
    {
        snprintf(fname, sizeof(fname), "%s.vis", targetPrefixFile);
        ofstream ofs(fname);
        ofs << jprbt->StringifyPath(final_path);
        ofs.close();
    }

    std::cout << "planning result " << planning_result.toCSVString() << "\n";
    std::cout << "\n";
    return 0;
}
