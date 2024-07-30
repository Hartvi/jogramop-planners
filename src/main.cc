
// tinyobjloader must be first
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include "pqp_load.h"
#include "rt_model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "base_planner.h"
#include "CParseArgs.h"
#include "j_rbt_planner.h"
#include "ut.h"
#include "burg_scene_loader.h"

#include <signal.h>

using namespace std;
using namespace Burs;

/* for catching signal when planner needs to be turned off
 */
static int *signalVariable = NULL;
void catchSignal(int sig)
{
    std::cerr << "Caught signal " << sig << "\n";
    if (sig == SIGTERM)
    {
        std::cerr << "Terminating due to catched SIGTERM\n";
        if (signalVariable)
        {
            (*signalVariable)++;
            std::cerr << "Increasing signal variable to " << (int)(*signalVariable) << "\n";
        }
    }
}

void registerSignal(int &signalVar)
{
    signalVariable = &signalVar;
    std::cerr << "Registering catching of signal " << SIGTERM << "\n";
    signal(SIGTERM, catchSignal);
}

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
    int run_tests;

    char *robot_offset;
    char *grasp_file;
    char *urdf_file;
    char *obstacle_file;
    char *start_config_file;
    int planner_type;
    int seed;

    int max_iters;
    double d_crit;
    double delta_q;
    double epsilon_q;
    int num_spikes;
    double p_close_enough;
    double probability_to_steer_to_target;

    char *target_prefix_file;

    double ground_level;
    int min_col_segment_idx; // minimum segment idx able to collide with ground

    char *target_configs_file;

    double goal_bias_radius;
    double goal_bias_probability;

    int ik_index_in_target_configs;

    int render_tree;
    double preheat_ratio;
    int use_rotation;
    double rotation_dist_ratio;

    double collision_resolution;
    int max_extensions;
    int jrbtOption;
    int use_joint_limits;

    int distance_estimate_type;

    {
        CmdOptions o;

        o.addOption(Option<int>("run_tests", &run_tests, 0, "Run tests"));

        o.addOption(Option<char *>("robot_offset", &robot_offset, "", "How the robot is transformed relative to the environment (.csv)"));
        o.addOption(Option<char *>("grasp", &grasp_file, "filename with grasps (.csv)"));
        o.addOption(Option<char *>("urdf", &urdf_file, "filename with URDF of the robot (.urdf)"));
        o.addOption(Option<char *>("obstacle", &obstacle_file, "filename with obstacles (.obj) or .yaml scene with .obj file links"));
        o.addOption(Option<char *>("start_config", &start_config_file, "filename with initial configuration (.csv)"));
        o.addOption(Option<int>("planner", &planner_type, "planner to choose (int)"));
        o.addOption(Option<int>("max_iters", &max_iters, "max number of iterations"));
        o.addOption(Option<double>("d_crit", &d_crit, "critical distance under which to switch to basic RRT"));
        o.addOption(Option<double>("delta_q", &delta_q, "max difference in configuration step"));
        o.addOption(Option<double>("epsilon_q", &epsilon_q, "step size when reverted to RRT"));
        o.addOption(Option<int>("num_spikes", &num_spikes, "number of bur spikes"));
        o.addOption(Option<double>("p_close_enough", &p_close_enough, "end-effector is close enough to target"));
        o.addOption(Option<double>("prob_steer", &probability_to_steer_to_target, "end-effector is close enough to target"));
        o.addOption(Option<char *>("target_configs", &target_configs_file, "defaultValue", "target IK solutions for the grasps"));
        o.addOption(Option<double>("ground_level", &ground_level, "ground z coodinate"));
        o.addOption(Option<int>("min_col_seg_idx", &min_col_segment_idx, "segment id from which it can collide with ground"));
        o.addOption(Option<int>("ik_index", &ik_index_in_target_configs, 0, "max iters for all ik solutions")); // default value is 0
        o.addOption(Option<int>("use_rot", &use_rotation, 0, "rotation bias threshold when to start using it (mm+deg)"));
        o.addOption(Option<double>("rot_ratio", &rotation_dist_ratio, 0.5, "ratio of rotation in distance metric (mm+deg)"));
        o.addOption(Option<char *>("target_prefix", &target_prefix_file, "planner_output", "file in which to save measurements, separated by keywords"));
        o.addOption(Option<int>("seed", &seed, -1, "random seed or time (if seed = -1)"));      // default value is -1 -> seed is from time
        o.addOption(Option<int>("render_tree", &render_tree, 0, "whether to render the tree")); // default value is 0
        o.addOption(Option<double>("collision_resolution", &collision_resolution, 0.0045, "resolution at which to check for collisions"));
        o.addOption(Option<int>("max_extensions", &max_extensions, 50, "max num of extend to goal steps"));
        o.addOption(Option<int>("jrbt_option", &jrbtOption, 0, "Which type of rbt to run. 0=default 1=extended non-convex"));
        o.addOption(Option<int>("use_joint_limits", &use_joint_limits, 1, "Use joint limits in inverse kinematics. 0=default 1=extended non-convex"));
        o.addOption(Option<int>("distance_type", &distance_estimate_type, 2, "Distance estimate type for RBT. 0=FULL translate + rotate jacobian, 1=jacobian radii, 2=projection radii, 3=projection radi + mesh rotation"));

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
    std::cout << "Grasps from " << grasp_file << "\n";
    std::cout << "URDF from " << urdf_file << "\n";
    std::cout << "Obstacles from " << obstacle_file << "\n";
    std::cout << "Planner: " << planner_type << "\n";

    // run: make clean && make && valgrind --leak-check=full ./load_obj_test ../Models/cube.obj ../Models/cube.obj out.txt

    std::string arg1 = std::string(argv[1]);
    // BEGIN COMMON SETTINGS ------------------------------------------------------------------------------------------------------------
    std::shared_ptr<JRbtPlanner> jprbt = std::make_shared<JRbtPlanner>(std::string(urdf_file));
    // 1. Set obstacles in urdfenv
    // 2. Setup parameters
    // 3. Plan
    auto &env = jprbt->env;
    if (strstr(obstacle_file, ".obj"))
    {
        env->AddObstacle(obstacle_file);
    }
    else
    {
        BurgLoader bburg = BurgLoader(std::string(obstacle_file));
        auto obstacles = bburg.GetObstacles();
        for (const auto &t : obstacles)
        {
            std::string n = std::get<0>(t);
            Eigen::MatrixXd T = std::get<1>(t);
            Eigen::Vector3d v = T.col(3).head<3>();
            // Eigen::Vector3d zaxis = Eigen::Vector3d(0, 0, 1);
            // double angle = -M_PI / 2;
            // auto aa = Eigen::AngleAxisd(angle, zaxis);
            // aa.matrix()
            // v = Eigen::Vector3d(v(1), v(0), v(2));
            Eigen::Matrix3d R = T.block<3, 3>(0, 0);
            // R.col(0).swap(R.col(1));
            // R = aa * R;
            // v = aa * v;
            env->AddObstacle(n, R, v);
            std::cout << "added obstacle: " << n << "\nR:\n"
                      << R << "\nt:\n"
                      << v.transpose() << "\n";
        }
        ///////////////////// TODO PROCESS YAML SCENE FROM BURG
    }
    // if there is an offset, transform obstacles by the inverse to plan
    if (robot_offset != "")
    {
        auto offset_transform = Grasp::LoadGrasps(std::string(robot_offset))[0];
        auto env_transform = offset_transform.data.inverse();
        for (size_t i = 0; i < env->obstacle_models.size(); ++i)
        {
            // TODO TRANSFORM MODEL BY THE INVERSE
            auto invr = env_transform.block<3, 3>(0, 0);
            auto invt = env_transform.col(3).head<3>();

            // env->SetObstacleRotation(i, invr * env->obstacle_models[i]->R, invt + env->obstacle_models[i]->t);
            env->SetObstacleRotation(i, invr * env->obstacle_models[i]->R, invt + (invr * env->obstacle_models[i]->t));
        }
    }
    env->SetGroundLevel(ground_level, min_col_segment_idx);

    std::string grasp_path(grasp_file);

    std::vector<Grasp> grasps = Grasp::LoadGrasps(grasp_path);

    Eigen::VectorXd start_config = RobotBase::parseCSVToVectorXd(start_config_file);
    std::cout << "start config " << start_config.transpose() << "\n";

    PlanningResult planning_result;

    std::optional<std::vector<Eigen::VectorXd>> path;
    std::vector<Eigen::VectorXd> final_path;

    JPlusRbtParameters params(max_iters, d_crit, delta_q, epsilon_q, num_spikes, p_close_enough, probability_to_steer_to_target, grasps);
    params.visualize_tree = render_tree;
    params.seed = usedSeed;
    params.ik_use_joint_limits = use_joint_limits;

    params.use_rotation = use_rotation;
    params.rotation_dist_ratio = rotation_dist_ratio;
    params.collision_resolution = collision_resolution;
    params.max_extensions = max_extensions;
    params.minCollisionIdx = min_col_segment_idx;
    params.distanceEstimateType = (DistanceEstimateType)distance_estimate_type;

    // END COMMON SETTINGS ------------------------------------------------------------------------------------------------------------

    { // output .txt file with results also before the planner runs, so in the case of planner failure/killing the program, there
        // will be (empty) outputfile, which indicates the failure
        char fname[2000];
        snprintf(fname, sizeof(fname), "%s.txt", target_prefix_file);
        ofstream ofs(fname);
        // ofs << planning_result.toCSVString() << "\n";
        ofs << planning_result.toJSON() << "\n";
        ofs.close();
    }

    // register signal and signal variable (if this sigman coms, the variable is increased and planners should terminate
    jprbt->globalTrigger = 0;
    registerSignal(jprbt->globalTrigger);

    if (run_tests)
    {
        std::cout << "OBJ TEST\n";
    }

    switch (planner_type)
    {
    case 0: // J+RRT
    {
        std::cout << "PLANNING J+ BIASED RRT\n";
        // extended with steer towards

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
        std::cout << "PLANNING J+RBT basic + proj\n";

        struct rusage t1, t2;
        getTime(&t1);
        params.distanceEstimateType = (DistanceEstimateType)2;
        path = jprbt->JRbtBasic(start_config, params, planning_result);

        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 4:
    {
        std::cout << "PLANNING RRT detailed\n";

        std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(target_configs_file);

        struct rusage t1, t2;
        getTime(&t1);
        if (goals.size() == 0)
        {
            std::cout << "NO INVERSE KINEMATICS SOLUTIONS PRESENT\n\n";
            exit(1);
        }
        path = jprbt->RRTConnectQStep(start_config, goals[ik_index_in_target_configs], params, planning_result);
        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 5:
    {
        std::cout << "PLANNING IKRRT detailed\n";

        std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(target_configs_file);

        struct rusage t1, t2;
        getTime(&t1);
        path = jprbt->IKRRT(start_config, params, planning_result);
        getTime(&t2);
        planning_result.time_taken = getTime(t1, t2);
        final_path = path.value();

        break;
    }
    case 95:
    {
        std::cout << " testing forward kinematics speeds\n";
        struct rusage t1, t2;
        getTime(&t1);

        auto chain = jprbt->env->robot->kdl_chain;
        for (size_t i = 0; i < params.max_iters; ++i)
        {
            KDL::JntArray q_kdl;
            q_kdl.data = start_config;

            KDL::Frame p_out;

            KDL::ChainFkSolverPos_recursive fk_solver(chain);
            for (size_t k = 0; k < chain.getNrOfSegments(); ++k)
            {
                if (fk_solver.JntToCart(q_kdl, p_out, k + 1) < 0)
                {
                    throw std::runtime_error("RobotBase::ForwardPass failed.");
                }
            }
        }
        getTime(&t2);
        std::cout << "time of bad forward kinematics: " << getTime(t1, t2) << "\n";

        getTime(&t1);
        for (size_t i = 0; i < params.max_iters; ++i)
        {
            KDL::JntArray q_kdl;
            q_kdl.data = start_config;

            std::vector<KDL::Frame> p_out(chain.getNrOfSegments());

            KDL::ChainFkSolverPos_recursive fk_solver(chain);
            if (fk_solver.JntToCart(q_kdl, p_out) < 0)
            {
                throw std::runtime_error("RobotBase::ForwardPass failed.");
            }
        }
        getTime(&t2);
        std::cout << "time of good forward kinematics: " << getTime(t1, t2) << "\n";

        /*
        distance check always:
        simple mesh:
        6,
        5,
        4.5,
        4.1
        full mesh:
        31.8,
        19.2,
        18.4,
        17.7
        */
        /*
        distance check once:
        simple mesh:
        2.5,
        1.5,
        1.2,
        0.76
        full mesh:
        14,
        1.6,
        1.2,
        0.76
        */
        exit(0);
        break;
    }
    case 96:
    {
        std::cout << " testing distance estimate + bur speeds\n";
        struct rusage t1, t2;
        RS start_state = jprbt->NewState(start_config, DistanceEstimateType::JacPosRot);
        for (int j = 0; j < 3; ++j)
        {
            getTime(&t1);
            auto [dci, dc] = jprbt->GetClosestDistances(start_state);
            start_state.closest_distance_ids = dci;
            start_state.closest_dists = dc;
            start_state.hasClosestDists = true;
            for (int i = 0; i < params.max_iters; ++i)
            {
                Eigen::MatrixXd rand_configs = jprbt->GetRandomQ(params.num_spikes);
                for (int i = 0; i < params.num_spikes; ++i)
                {
                    rand_configs.col(i) = start_state.config + params.delta_q * rand_configs.col(i).normalized();
                }
                std::vector<RS> new_states = jprbt->NewStates(rand_configs, DistanceEstimateType::None);
                jprbt->GetEndpointsGeneral(start_state, new_states, (DistanceEstimateType)j, false);
            }
            getTime(&t2);
            double t = getTime(t1, t2);
            std::cout << "time of " << j << ": " << t << "\n";
        }
        getTime(&t1);
        std::shared_ptr<BurTree> t = std::make_shared<BurTree>(start_state, jprbt->q_dim);
        for (int i = 0; i < params.max_iters; ++i)
        {
            for (int k = 0; k < params.num_spikes; ++k)
            {
                Eigen::VectorXd rand_config = jprbt->GetRandomQ(1);
                RS rand_state = jprbt->NewState(rand_config);
                jprbt->RRTStepInQ(t, 0, rand_state, params.epsilon_q, params.collision_resolution);
            }
        }
        getTime(&t2);
        double measured_time = getTime(t1, t2);
        std::cout << "time of rrt steps: " << measured_time << "\n";

        /*
        distance check always:
        simple mesh:
        6,
        5,
        4.5,
        4.1
        full mesh:
        31.8,
        19.2,
        18.4,
        17.7
        */
        /*
        distance check once:
        simple mesh:
        2.5,
        1.5,
        1.2,
        0.76
        full mesh:
        14,
        1.6,
        1.2,
        0.76
        */
        exit(0);
        break;
    }
    case 97:
    {
        std::cout << "TEST YAML\n";
        BurgLoader b2();
        BurgLoader bburg = BurgLoader(std::string(obstacle_file));
        bburg.GetObstacles();
        std::cout << "path: " << bburg.path << "\n";
        std::cout << "tested YAML\n";
        break;
    }
    case 98:
    {
        std::cout << "TEST COLLISION VS DISTANCE SPEED\n";
        std::vector<Eigen::VectorXd> goals = RobotBase::parseCSVToVectors(target_configs_file);
        path = jprbt->TestCollisionVsDistanceTime(start_config, params, planning_result);
        exit(0);
        break;
    }
    default:
    {
        break;
    }
    } // end switch

    if (jprbt->globalTrigger == 0)
    {
        std::cerr << "Planning finished in time\n";
        std::cout << "Planning finished in time\n";
        planning_result.finished_in_time = 1;
    }
    try
    {

        char fname[2000];
        {
            snprintf(fname, sizeof(fname), "%s.txt", target_prefix_file);
            ofstream ofs(fname);
            // ofs << planning_result.toCSVString() << "\n";
            ofs << planning_result.toJSON() << "\n";
            ofs.close();
        }
        {
            snprintf(fname, sizeof(fname), "%s.try", target_prefix_file);
            ofstream ofs(fname);
            ofs << jprbt->ConfigsToString(final_path) << "\n";
            ofs.close();
        }
        {
            snprintf(fname, sizeof(fname), "%s.vis", target_prefix_file);
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
    }
    catch (const std::exception &e)
    {
        std::cout << "FAILED TO SAVE RESULT: " << e.what() << "\n";
    }
    std::cout << "planning result " << planning_result.toCSVString() << "\n";
    std::cout << "\n";
    return 0;
}
