
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

    int max_iters;
    double d_crit;
    double delta_q;
    double epsilon_q;
    int num_spikes;
    double p_close_enough;
    double probability_to_steer_to_target;

    char *targetPrefixFile;

    int renderVideo;

    char *visualizationScriptFile;

    int camX, camY, camZ;

    double groundLevel;
    int minColSegmentIdx; // minimum segment idx able to collide with ground

    char *targetConfigsFile;

    double goal_bias_radius;
    double goal_bias_probability;

    int ik_index_in_target_configs;
    double q_resolution;

    int render_tree;
    double preheat_ratio;
    int useRotation;
    int preheat_type;

    {
        CmdOptions o;

        o.addOption(Option<char *>("grasp", &graspFile, "filename with grasps (.csv)"));
        o.addOption(Option<char *>("urdf", &urdfFile, "filename with URDF of the robot (.urdf)"));
        o.addOption(Option<char *>("obstacle", &obstacleFile, "filename with obstacles (.obj)"));
        o.addOption(Option<char *>("start_config", &startConfigFile, "filename with initial configuration (.csv)"));
        o.addOption(Option<int>("planner", &plannerType, "planner to choose (int)"));
        o.addOption(Option<int>("max_iters", &max_iters, "max number of iterations"));
        o.addOption(Option<double>("d_crit", &d_crit, "critical distance under which to switch to basic RRT"));
        o.addOption(Option<double>("delta_q", &delta_q, "max difference in configuration step"));
        o.addOption(Option<double>("epsilon_q", &epsilon_q, "step size when reverted to RRT"));
        o.addOption(Option<int>("num_spikes", &num_spikes, "number of bur spikes"));
        o.addOption(Option<double>("p_close_enough", &p_close_enough, "end-effector is close enough to target"));
        o.addOption(Option<double>("prob_steer", &probability_to_steer_to_target, "end-effector is close enough to target"));

        o.addOption(Option<char *>("target_configs", &targetConfigsFile, "defaultValue", "target IK solutions for the grasps"));
        o.addOption(Option<double>("groundLevel", &groundLevel, "ground z coodinate"));
        o.addOption(Option<int>("minColSegIdx", &minColSegmentIdx, "segment id from which it can collide with ground"));

        o.addOption(Option<int>("ik_index", &ik_index_in_target_configs, 0, "max iters for all ik solutions")); // default value is 0
        o.addOption(Option<double>("goal_bias_radius", &goal_bias_radius, "radius to start turning towards goal"));
        o.addOption(Option<double>("goal_bias_prob", &goal_bias_probability, "probability to turn to goal when close to goal"));
        o.addOption(Option<double>("q_resolution", &q_resolution, "resolution of individual steps in rbt"));

        o.addOption(Option<int>("use_rot", &useRotation, 0, "rotation bias threshold when to start using it (mm*deg)"));
        o.addOption(Option<double>("preheat_ratio", &preheat_ratio, 0.1, "ratio of iterations to use for preheating"));
        o.addOption(Option<int>("preheat_type", &preheat_type, 0, "type of preheating (0,1) so far"));
        o.addOption(Option<char *>("target_prefix", &targetPrefixFile, "file in which to save measurements, separated by keywords"));

        o.addOption(Option<int>("render", &renderVideo, "whether to render video"));
        o.addOption(Option<char *>("vis_script", &visualizationScriptFile, "script to visualize with"));
        o.addOption(Option<int>("cx", &camX, "camera x coordinate"));
        o.addOption(Option<int>("cy", &camY, "camera y coordinate"));
        o.addOption(Option<int>("cz", &camZ, "camera z coordinate"));

        o.addOption(Option<int>("seed", &seed, -1, "random seed or time (if seed = -1)")); // default value is -1 -> seed is from time

        o.addOption(Option<int>("render_tree", &render_tree, 0, "whether to render the tree")); // default value is 0

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

    // run: make clean && make && valgrind --leak-check=full ./load_obj_test ../Models/cube.obj ../Models/cube.obj out.txt

    // std::cout << "Arg 1: " << argv[1] << std::endl;
    std::string arg1 = std::string(argv[1]);
    if (arg1 == "test")
    {
        // BEGIN COMMON SETTINGS ------------------------------------------------------------------------------------------------------------
        std::shared_ptr<JRbtPlanner> jprbt = std::make_shared<JRbtPlanner>(std::string(urdfFile));
        // 1. Set obstacles in urdfenv
        // 2. Setup parameters
        // 3. Plan
        auto &env = jprbt->env;
        std::cout << "REMOVED OBSTACLE TEMPORARILY\n";
        env->AddObstacle(obstacleFile);
        env->SetGroundLevel(groundLevel, minColSegmentIdx);

        std::string grasp_path(graspFile);

        std::vector<Grasp> grasps = Grasp::LoadGrasps(grasp_path);
        // std::cout << "grasps: " << grasps.size() << std::endl;
        // for (unsigned int i = 0; i < grasps.size(); ++i)
        // {
        //     std::cout << "Grasp:\n"
        //               << grasps[i].frame << std::endl;
        // }

        Eigen::VectorXd start_config = RobotBase::parseCSVToVectorXd(startConfigFile);
        std::cout << "start config " << start_config.transpose() << "\n";

        // auto grasp_frames = Grasp::GraspsToFrames(grasps);

        PlanningResult planning_result;

        std::optional<std::vector<Eigen::VectorXd>> path;
        std::vector<Eigen::VectorXd> final_path;

        // distance metric is euclidean squared
        // double p_close_sqr = p_close_enough * p_close_enough;
        // double goal_bias_radius_sqr = goal_bias_radius * goal_bias_radius;

        JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasps, goal_bias_radius, goal_bias_probability, q_resolution);
        params.visualize_tree = render_tree;
        params.seed = usedSeed;
        params.preheat_ratio = preheat_ratio;
        params.use_rotation = useRotation;
        params.preheat_type = preheat_type;
        // END COMMON SETTINGS ------------------------------------------------------------------------------------------------------------

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
        case 0: // rbt
        {
            std::cout << "PLANNING BLIND RBT\n";
            // only difference is that it doesnt use probability_to_steer_to_target
            params.probability_to_steer_to_target = 0.0;
            params.goal_bias_probability = 0.0;

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->JRbt(start_config, params, planning_result);
            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);

            final_path = path.value();
            break;
        }
        case 1: // j+rbt
        {
            std::cout << "PLANNING J+RBT\n";
            // extended with steer towards

            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames);
            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->JRbt(start_config, params, planning_result);

            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 2: // BASIC RRT
        {
            std::cout << "PLANNING BASIC RRT\n";

            // JPlusRbtParameters params(max_iters, 1e10, delta_q, epsilon_q, num_spikes, p_close_enough, 0.0, grasp_frames);
            // JPlusRbtParameters params(max_iters, 1e10, delta_q, epsilon_q, num_spikes, p_close_enough, 0.0, grasp_frames, goal_bias_radius, goal_bias_probability);
            // d_crit is infinite => always RRT
            // probability to steer with J+ => 0
            params.d_crit = 1e10;
            params.probability_to_steer_to_target = 0.0;
            params.goal_bias_probability = 0.0;

            struct rusage t1,
                t2;
            getTime(&t1);
            path = jprbt->JRRT(start_config, params, planning_result);
            getTime(&t2);

            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 3: // J+RRT
        {
            std::cout << "PLANNING J+ BIASED RRT\n";
            // extended with steer towards

            // JPlusRbtParameters params(max_iters, 1e10, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames);
            // JPlusRbtParameters params(max_iters, 1e10, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);
            // only RRT switch:
            // params.d_crit = 1e10;

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->JRRT(start_config, params, planning_result);
            getTime(&t2);

            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 4:
        {
            std::cout << "PLANNING IK RBT\n";
            // only difference is that it doesnt use probability_to_steer_to_target

            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, 0.0, grasp_frames);
            // JPlusRbtParameters params(max_iters, 1e10, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);

            std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

            struct rusage t1, t2;
            getTime(&t1);
            // auto path = jprbt->RbtConnect(start_config, goals[0], params);
            // auto path = jprbt->RbtMultiGoal(start_config, goals, params, planning_result, max_iter_for_all_ik);
            // for (auto &g : goals)
            // {
            //     std::cout << "goal: " << g.transpose() << "\n";
            // }
            // std::cout << "TARGET IK IDX: " << ik_index_in_target_configs << "\n";
            // std::cout << "selected goal: " << goals[ik_index_in_target_configs].transpose() << "\n";
            // exit(1);
            path = jprbt->RbtConnectDenseBurs(start_config, goals[ik_index_in_target_configs], params, planning_result);
            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 5:
        {
            std::cout << "PLANNING IK RRT\n";
            // only difference is that it doesnt use probability_to_steer_to_target

            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, 0.0, grasp_frames);
            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);

            std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

            struct rusage t1, t2;
            getTime(&t1);
            // auto path = jprbt->RbtConnect(start_config, goals[0], params);
            // auto path = jprbt->RRTMultiGoal(start_config, goals, params, planning_result, max_iter_for_all_ik);
            path = jprbt->RRTConnect(start_config, goals[ik_index_in_target_configs], params, planning_result);
            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 6:
        {
            std::cout << "PLANNING J+RBTS\n";
            // extended with steer towards

            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames);
            // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->JRbtS(start_config, params, planning_result);

            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 7:
        {
            std::cout << "PLANNING IK RBT\n";

            std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->RbtConnectDenseBurs(start_config, goals[ik_index_in_target_configs], params, planning_result);
            getTime(&t2);
            planning_result.time_taken = getTime(t1, t2);
            final_path = path.value();

            break;
        }
        case 8:
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
        case 98:
        {
            std::cout << "TEST COLLISION VS DISTANCE SPEED\n";
            std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);
            path = jprbt->TestCollisionVsDistanceTime(start_config, params, planning_result);
            exit(0);
            break;
        }
        case 99:
        {
            std::cout << "TEST SAMPLING RANDOM CONFIGS\n";
            // only difference is that it doesnt use probability_to_steer_to_target

            std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(targetConfigsFile);

            struct rusage t1, t2;
            getTime(&t1);
            path = jprbt->TestSampling(start_config, params, planning_result);
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
            // ofs << planning_result.toCSVString() << "\n";
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

            // std::ifstream ifs(fname);

            // if (!ifs)
            // {
            //     std::cerr << "Failed to open file: " << fname << std::endl;
            //     return 1; // or handle error in a way suitable for your application
            // }

            // std::string line;
            // while (std::getline(ifs, line))
            // {
            //     std::cout << line << '\n';
            // }
        }
        // std::cout << "\nvisualization: \n"
        //   << jprbt->StringifyPath(final_path) << "\n\n";

        if (renderVideo)
        {
            // ROBOT:
            std::string vis_file_name = std::string(targetPrefixFile) + ".vis";

            // std::ofstream vis_file(vis_file_name);

            // if (vis_file.is_open())
            // {
            //     vis_file << jprbt->StringifyPath(final_path);
            //     vis_file.close();
            // }

            std::string path_name = joinWithCurrentDirectory(vis_file_name);

            // needs `pip install bpy` for python 3.10, numpy
            std::string vis_args = path_name + " " + std::to_string(camX) + " " + std::to_string(camY) + " " + std::to_string(camZ) + " " + grasp_path;
            // TREE:
            std::cout << "RENDER TREE: " << render_tree << "\n";
            if (render_tree)
            {
                std::string tree_file_name = std::string(targetPrefixFile) + ".tree";

                std::ofstream tree_file(tree_file_name);
                // std::cout << "tree csv: " << jprbt->tree_csv << "\n";

                if (tree_file.is_open())
                {
                    tree_file << jprbt->tree_csv;
                    tree_file.close();
                }
                std::string tree_path_name = joinWithCurrentDirectory(tree_file_name);
                std::cout << "tree file " << tree_path_name << "\n";
                vis_args = vis_args + " " + tree_path_name;
            }

            std::string str_command = "python3.10 " + std::string(visualizationScriptFile) + " " + vis_args;

            const char *command = str_command.c_str();
            int result = system(command);

            if (result != 0)
            {
                std::cout << "Calling `" << command << "` failed" << std::endl;
            }
        }

        std::cout << "planning result " << planning_result.toCSVString() << "\n";
        // std::cout << "argc: ";
        // std::cout << argc;
        std::cout << "\n";
    }
    else if (arg1 == "rotation")
    {
        // BEGIN COMMON SETTINGS ------------------------------------------------------------------------------------------------------------
        std::shared_ptr<JRbtPlanner> jprbt = std::make_shared<JRbtPlanner>(std::string(urdfFile));
        // 1. Set obstacles in urdfenv
        // 2. Setup parameters
        // 3. Plan
        auto &env = jprbt->env;
        env->SetGroundLevel(groundLevel, minColSegmentIdx);

        std::string grasp_path(graspFile);

        std::vector<Grasp> grasps = Grasp::LoadGrasps(grasp_path);

        Eigen::VectorXd start_config = RobotBase::parseCSVToVectorXd(startConfigFile);
        std::cout << "start config " << start_config.transpose() << "\n";

        // auto grasp_frames = Grasp::GraspsToFrames(grasps);

        PlanningResult planning_result;

        std::optional<std::vector<Eigen::VectorXd>> path;
        std::vector<Eigen::VectorXd> final_path;

        // distance metric is euclidean squared
        // double p_close_sqr = p_close_enough * p_close_enough;
        // double goal_bias_radius_sqr = goal_bias_radius * goal_bias_radius;

        JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasps, goal_bias_radius, goal_bias_probability, q_resolution);
        params.visualize_tree = render_tree;
        params.seed = usedSeed;
        params.preheat_ratio = preheat_ratio;
        // END COMMON SETTINGS ------------------------------------------------------------------------------------------------------------

        std::cout << "ROTATION TESTING\n";
        // extended with steer towards

        // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames);
        // JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasp_frames, goal_bias_radius, goal_bias_probability);

        struct rusage t1, t2;
        getTime(&t1);
        path = jprbt->RotTest(start_config, params, planning_result);

        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        char fname[2000];
        {
            snprintf(fname, sizeof(fname), "%s.txt", targetPrefixFile);
            ofstream ofs(fname);
            // ofs << planning_result.toCSVString() << "\n";
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

        if (renderVideo)
        {
            // ROBOT:
            std::string vis_file_name = std::string(targetPrefixFile) + ".vis";

            // std::ofstream vis_file(vis_file_name);

            // if (vis_file.is_open())
            // {
            //     vis_file << jprbt->StringifyPath(final_path);
            //     vis_file.close();
            // }

            std::string path_name = joinWithCurrentDirectory(vis_file_name);

            // needs `pip install bpy` for python 3.10, numpy
            std::string vis_args = path_name + " " + std::to_string(camX) + " " + std::to_string(camY) + " " + std::to_string(camZ) + " " + grasp_path;
            // TREE:
            std::cout << "RENDER TREE: " << render_tree << "\n";
            if (render_tree)
            {
                std::string tree_file_name = std::string(targetPrefixFile) + ".tree";

                std::ofstream tree_file(tree_file_name);
                // std::cout << "tree csv: " << jprbt->tree_csv << "\n";

                if (tree_file.is_open())
                {
                    tree_file << jprbt->tree_csv;
                    tree_file.close();
                }
                std::string tree_path_name = joinWithCurrentDirectory(tree_file_name);
                std::cout << "tree file " << tree_path_name << "\n";
                vis_args = vis_args + " " + tree_path_name;
            }

            std::string str_command = "python3.10 " + std::string(visualizationScriptFile) + " " + vis_args;

            const char *command = str_command.c_str();
            int result = system(command);

            if (result != 0)
            {
                std::cout << "Calling `" << command << "` failed" << std::endl;
            }
        }
    }
    return 0;
}
