#include "ut.h"
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "pqp_load.h"
#include "rt_model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "base_planner.h"
#include <functional>
#include "math.h"
#include "bur_funcs.h"
#include "stick_robot.h"
#include <fstream>
// #include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <time.h>
#include "grasps.h"
#include "j_plus_rbt_planner.h"

#include "rbt_parameters.h"
#include "j_plus_rbt_parameters.h"
#include "test_urdf.h"

namespace test
{
    using namespace std;
    using namespace Burs;

    Eigen::Matrix3d eulerToRotationMatrix(double x, double y, double z)
    {
        Eigen::Matrix3d Rx, Ry, Rz;

        // Rotation matrix around the X-axis
        Rx << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x);

        // Rotation matrix around the Y-axis
        Ry << cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y);

        // Rotation matrix around the Z-axis
        Rz << cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1;

        // Combined rotation matrix, R = Rz * Ry * Rx
        Eigen::Matrix3d R = Rz * Ry * Rx;

        return R;
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

    std::string joinWithCurrentDirectory(const std::string &filename)
    {
        std::filesystem::path currentDir = std::filesystem::current_path();
        std::filesystem::path fullPath = currentDir / filename;
        return fullPath.string();
    }

    // return random 2D vector in the inter-circular area of two concentric circles in the origin with radii `min` and `max`
    Eigen::Vector3d GetRandom2DPosition(double min, double max)
    {
        Eigen::Vector2d a = Eigen::Vector2d::Random();
        double random_angle = a(0) * M_PI;
        double random_distance = (a(1) + 1) / 2.0 * (max - min) + min;
        Eigen::Vector3d v = Eigen::Vector3d(random_distance * cos(random_angle), random_distance * sin(random_angle), 0.0);
        return v;
    }

    JPlusRbtPlanner GenerateRandomScenario(const unsigned int &num_obstacles)
    {
        std::string obstacle_path = "/home/hartvi/Documents/CVUT/diploma_thesis/Models/cube.obj";
        std::string robot_urdf_path = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda.urdf";
        int max_iters = 1000;
        Meters d_crit = 0.1;
        Relative delta_q = 1.0;
        Relative epsilon_q = 0.028;
        int num_spikes = 7;

        double minDistanceFromCenter = 1.5; // cube is 1 meter in radius => this will be 0.5 from the center, 0.3 for diagonal positions
        double maxDistanceFromCenter = 2.0;

        JPlusRbtPlanner urdf_planner(robot_urdf_path);

        for (unsigned int i = 0; i < num_obstacles; ++i)
        {
            // random position not colliding with the robot by default (I hope)
            Eigen::Vector3d obstacle_position = GetRandom2DPosition(minDistanceFromCenter, maxDistanceFromCenter);

            // std::cout << "random obstacle position: " << obstacle_position.transpose() << std::endl;
            urdf_planner.AddObstacle(obstacle_path, Eigen::Matrix3d::Identity(), obstacle_position);
        }

        Eigen::MatrixXd start_goal = urdf_planner.GetRandomQ(2);

        // plan path
        // Eigen::VectorXd start = start_goal.col(0);
        // Eigen::VectorXd goal = start_goal.col(1);
        return urdf_planner;
    }

    std::string GenerateFailingCase(const unsigned int &num_obstacles)
    {
        //     auto urdf_planner = GenerateRandomScenario(num_obstacles);
        //     Eigen::MatrixXd start_goal = urdf_planner.GetRandomQ(2);

        //     Eigen::VectorXd start = start_goal.col(0);
        //     Eigen::VectorXd goal = start_goal.col(1);

        //     auto path_opt = urdf_planner.PlanPath(start, goal);
        //     if (!path_opt)
        //     {

        //         std::string test_file_name = "fail_file_" + getCurrentTimestamp() + ".txt";
        //         std::vector<Eigen::VectorXd> path(2);
        //         path[0] = start;
        //         path[1] = goal;
        //         if (!urdf_planner.IsColliding(start) && !urdf_planner.IsColliding(goal))
        //         {
        //             std::ofstream test_file(test_file_name);

        //             if (test_file.is_open())
        //             {
        //                 // test_file << urdf_planner.StringifyPath(path);
        //                 test_file << urdf_planner.GetEnv<URDFEnv>()->ToScenarioString(start, goal);
        //                 test_file.close();
        //             }
        //             std::cout << "Path planning failed" << std::endl;
        //             return test_file_name;
        //         }
        //         else
        //         {
        //             return "";
        //         }
        //     }
        //     else
        //     {
        //         return "";
        //     }
    }

    std::string GenerateRandomExperiment(const unsigned int &num_obstacles)
    {
        /*
        auto urdf_planner = GenerateRandomScenario(num_obstacles);
        Eigen::MatrixXd start_goal = urdf_planner.GetRandomQ(2);
        // // plan path
        Eigen::VectorXd start = start_goal.col(0);
        Eigen::VectorXd goal = start_goal.col(1);

        auto path_opt = urdf_planner.PlanPath(start, goal);
        if (path_opt)
        {
            auto path = path_opt.value();
            path = Burs::JPlusRbtPlanner::InterpolatePath(path, 0.1);

            std::string test_file_name = "file_" + getCurrentTimestamp() + ".txt";

            std::ofstream test_file(test_file_name);

            if (test_file.is_open())
            {
                test_file << urdf_planner.StringifyPath(path);
                test_file.close();
            }
            // return "";
            return test_file_name;

            // TODO: create random cubes in positions in this range and check if start and goal are collision free, then try to plan a path and then VISUALIZE
        }
        else
        {

            std::string test_file_name = "fail_file_" + getCurrentTimestamp() + ".txt";
            std::vector<Eigen::VectorXd> path(2);
            path[0] = start;
            path[1] = goal;
            if (!urdf_planner.IsColliding(start) && !urdf_planner.IsColliding(goal))
            {
                std::ofstream test_file(test_file_name);

                if (test_file.is_open())
                {
                    test_file << urdf_planner.StringifyPath(path);
                    test_file.close();
                }
                std::cout << "Path planning failed" << std::endl;
                return "";
                // return test_file_name;
            }
            else
            {
                return "";
            }
        }
        */
    }


    void main_test(const char *graspFile, const char *urdfFile, const char *obstacleFile, const int plannerType, const char *paramsFile)
    {
        std::string grasp_path(graspFile);

        std::vector<Grasp> grasps = Grasp::LoadGrasps(grasp_path);
        std::cout << "grasps: " << grasps.size() << std::endl;
        for (unsigned int i = 0; i < grasps.size(); ++i)
        {
            std::cout << "Grasp:\n"
                    << grasps[i].ToFrame() << std::endl;
        }
        auto params = RbtParameters(paramsFile);
        std::cout << "Planning params: " << params.toString() << "\n";
        auto paramsjplusrbt = JPlusRbtParameters(paramsFile);
        std::cout << "Planning params: " << params.toString() << "\n";

        // JPlusRbtPlanner(std::string urdf_file);
        std::shared_ptr<JPlusRbtPlanner> jprbt = std::make_shared<JPlusRbtPlanner>(std::string(urdfFile));
        // 1. Set obstacles in urdfenv
        // 2. Setup parameters
        // 3. Plan

        Eigen::VectorXd random_start_q = jprbt->GetRandomQ(1);
        jprbt->JPlusRbt(random_start_q, paramsjplusrbt);

        // std::optional<std::vector<Eigen::VectorXd>>
        // JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters);

        // for (unsigned int i = 0; i < 1000; ++i)
        // {
        //     unsigned int num_obstacles = 3;

        //     struct rusage t1, t2;
        //     // in terminal: time ./burs_of_free_space
        //     getTime(&t1);
        //     std::string file_name = GenerateRandomExperiment(num_obstacles);
        //     getTime(&t2);
        //     std::cout << "Time: " << getTime(t1, t2) << std::endl;
        //     return;

        //     // std::string file_name = GenerateFailingCase(num_obstacles);
        //     // std::cout << "Just planned a path" << std::endl;

        //     // if (file_name != "")
        //     // {
        //     //     std::cout << "Created fail case" << std::endl;
        //     // }
        //     // continue;
        //     if (file_name != "")
        //     {
        //         std::string path_name = joinWithCurrentDirectory(file_name);
        //         std::string str_command = "python3.10 ../scripts/animate_scene.py " + path_name;
        //         // std::string str_command = "python3.10 ../scripts/render_path_positions.py " + path_name;
        //         const char *command = str_command.c_str();
        //         int result = system(command);

        //         if (result != 0)
        //         {
        //             std::cout << "Calling `" << command << "` failed" << std::endl;
        //         }
        //     }
        //     else
        //     {
        //         std::cout << "Path returned by path planner was invalid" << std::endl;
        //     }
        // }

        return;
    }
}
