#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "load.h"
#include "model.h"
#include <flann/flann.hpp>
#include "printing.h"
#include "bur_tree.h"
#include "bur_algorithm.h"
#include <functional>
#include "math.h"
#include "bur_funcs.h"
#include "stick_robot.h"
#include <fstream>

namespace test
{
    using namespace std;
    using namespace Burs;

    void testFunctions(int argc, char *argv[])
    {
        if (argc < 3)
        {
            std::cout << "Usage: ./exc robot_model.obj obstacle_model.obj" << std::endl;
            return;
        }

        std::shared_ptr<StickRobot> stick_robot = std::make_shared<StickRobot>();

        Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(stick_robot->NUM_SEGMENTS, 2);
        for (int i = 0; i < stick_robot->NUM_SEGMENTS; ++i)
        {
            bounds.col(0).setConstant(-M_PI);
            bounds.col(1).setConstant(M_PI);
        }
        std::cout << "bounds:\n"
                  << bounds << std::endl;

        // Number of tests
        const int numTests = 1;

        // Create random generator and uniform distributions for the angles
        Eigen::VectorXd randConfiguration(2);
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(-M_PI, M_PI); // Assume angles are in range [-pi, pi]

        for (int i = 0; i < numTests; i++)
        {
            // Generate random configuration
            randConfiguration << M_PI_2, 0.0;

            std::cout << "Test " << i + 1 << ":" << std::endl;
            std::cout << "Random Configuration: [" << randConfiguration(0) << ", " << randConfiguration(1) << "]" << std::endl;

            for (int ith_distal_point = 0; ith_distal_point <= 1; ith_distal_point++)
            {
                std::cout << "For ith_distal_point: " << ith_distal_point << std::endl;

                // ForwardJoint
                Eigen::Vector3d resultFK = stick_robot->ForwardJoint(ith_distal_point, randConfiguration);
                std::cout << "ForwardJoint Result: [" << resultFK(0) << ", " << resultFK(1) << ", " << resultFK(2) << "]" << std::endl;

                // RadiusFunc
                double resultRF = stick_robot->RadiusFunc(ith_distal_point, randConfiguration);
                std::cout << "RadiusFunc Result: " << resultRF << std::endl;

                // StickRt
                auto resultSRT = stick_robot->StickRt(randConfiguration);
                std::vector<Eigen::Matrix3d> rotations = std::get<0>(resultSRT);
                std::vector<Eigen::Vector3d> translations = std::get<1>(resultSRT);
                for (size_t j = 0; j < rotations.size(); j++)
                {
                    std::cout << "Rotation Matrix " << j + 1 << ":" << std::endl
                              << rotations[j] << std::endl;
                    std::cout << "Translation Vector " << j + 1 << ": [" << translations[j](0) << ", " << translations[j](1) << ", " << translations[j](2) << "]" << std::endl;
                }

                std::cout << "-----------------------" << std::endl;
            }
        }
    }

    void test_forward(int argc, char *argv[])
    {
        // std::cout << "TEST FUNCTIONS" << std::endl;
        // testFunctions(argc, argv);
        // std::cout << "END TEST FUNCTIONS" << std::endl;
        std::cout << std::endl;

        if (argc < 6)
        {
            std::cout << "Usage: ./exc robot_model.obj obstacle_model.obj x y z" << std::endl;
            return;
        }
        std::cout << "FUNCTION TESTING" << std::endl;

        std::shared_ptr<StickRobot> stick_robot = std::make_shared<StickRobot>();

        Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(stick_robot->NUM_SEGMENTS, 2);
        for (int i = 0; i < stick_robot->NUM_SEGMENTS; ++i)
        {
            bounds.col(0).setConstant(-10 * M_PI);
            bounds.col(1).setConstant(10 * M_PI);
        }
        std::cout << "bounds:\n"
                  << bounds << std::endl;

        // Burs::ForwardKinematics fk = ForwardDistal;
        // BurNodeWrapper(ForwardKinematics f, int num_segments, int max_iters, double d_crit, double delta_q, double epsilon_q);
        // Burs::ForwardKinematics fk = stick_robot->ForwardJoint;

        Burs::ForwardKinematics fk =
            [stick_robot](const int &ith_distal_point, const Eigen::VectorXd &configuration) -> Eigen::Vector3d
        {
            return stick_robot->ForwardJoint(ith_distal_point, configuration);
        };

        Burs::RadiusFunc rf = [stick_robot](const int &ith_distal_point, const Eigen::VectorXd &configuration) -> double
        {
            return stick_robot->RadiusFunc(ith_distal_point, configuration);
        };

        Burs::ForwardRt srt = [stick_robot](const Eigen::VectorXd &configuration) -> std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>
        {
            return stick_robot->StickRt(configuration);
        };

        int max_iters = 100;
        double d_crit = 0.1;
        double delta_q = M_PI;
        double epsilon_q = M_PI / 10.0;
        int num_spikes = 7;
        std::unique_ptr<Burs::BurAlgorithm> b = std::make_unique<Burs::BurAlgorithm>(stick_robot->NUM_SEGMENTS, fk, max_iters, d_crit, delta_q, epsilon_q, bounds, rf, num_spikes);

        int point = 1;
        VectorXd config(2);
        VectorXd config2(2);
        config << 0, 0;
        config2 << 4.5 * M_PI, 0;

        // std::cout << "config1: " << config.transpose() << std::endl;
        // std::cout << "config2: " << config2.transpose() << std::endl;

        Eigen::Vector3d result = b->ForwardEuclideanJoint(1, config); // Call the function through the std::function wrapper

        // Output
        std::cout << "Initial config distal point: " << result.transpose() << std::endl;

        std::cout << "END FUNCTION TESTING" << std::endl;
        std::cout << std::endl;
        // TESTING TOGETHER

        std::shared_ptr<Burs::BurEnv> b_env = std::make_shared<Burs::BurEnv>();

        std::shared_ptr<TrPQPModel> robot_model = std::make_shared<TrPQPModel>(argv[1]);
        std::shared_ptr<TrPQPModel> obstacle_model = std::make_shared<TrPQPModel>(argv[2]);

        b_env->AddRobotModel(robot_model);
        b_env->AddForwardRt(srt);
        b_env->AddObstacleModel(obstacle_model);

        MatrixXd m(6, 3);
        std::cout << "m: " << m << std::endl;
        Vector3d obstacle_position = Vector3d(std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]));
        for (int i = 0; i < b_env->obstacle_models.size(); i++)
        {
            // for cube
            std::cout << "Obstacle position: " << Eigen::Vector3d(b_env->obstacle_models[i]->getT()).transpose() << std::endl;
            b_env->obstacle_models[i]->SetTranslation(obstacle_position);
            std::cout << "Obstacle position AFTER: " << Eigen::Vector3d(b_env->obstacle_models[i]->getT()).transpose() << std::endl;
        }
        b->SetBurEnv(b_env);
        b_env->SetPoses(config);
        // add obstacles to b_env
        // add robot objects to b_env
        // freeze
        // load the forward kinematics functions:
        //  1. forwardRt(q)
        //  2. forward(i,q)
        //  3. r(i, q1, q2, t)

        // Eigen::MatrixXd a = MatrixXd::Zero(2, 1);
        // std::cout << std::endl
        //           << "TESTING BUR: " << std::endl;
        // Bur bur = b->GetBur(config, config2, 0.5);
        // std::cout << bur;
        // std::cout << "END TESTING BUR" << std::endl
        //           << std::endl;
        // std::cout << "START CHECK: CLOSEST DISTANCE: " << b->GetClosestDistance(config) << std::endl;
        // std::exit(1);
        std::cout << "CLOSEST DISTANCE TEST" << std::endl;
        double closest_distance_test = b->GetClosestDistance(Vector2d(0, 0));
        std::cout << "distance: " << closest_distance_test << std::endl;
        std::cout << "END CLOSEST DISTANCE TEST" << std::endl;
        std::cout << std::endl;

        std::cout << "TEST RBTCONNECT" << std::endl;
        // std::cout << "START CHECK: CLOSEST DISTANCE: " << b->GetClosestDistance(config) << std::endl;
        std::optional<MatrixXd> path = b->RbtConnect(config, config2);
        if (!path)
        {
            std::cout << "test_forward: path not found" << std::endl;
            return;
        }
        MatrixXd myPath = path.value();
        std::cout << "Path: " << myPath << std::endl;
        std::ofstream myFileA;

        myFileA.open("path.txt");
        std::string robot_name = argv[1];
        std::string obstacle_name = argv[2];
        myFileA << "robot," << robot_name << "," << std::endl;
        myFileA << "obstacle," << obstacle_name << "," << std::endl;
        myFileA << "obstacle_position," << std::endl;
        for (int i = 0; i < 3; ++i)
        {
            myFileA << obstacle_position(i) << ",";
        }
        myFileA << std::endl;

        std::cout
            << "Cols: " << myPath.cols() << "  Rows: " << myPath.rows() << std::endl;

        myFileA << "path," << std::endl;
        for (int i = 0; i < myPath.cols(); i++)
        {
            VectorXd result = myPath.col(i);
            for (int k = 0; k < myPath.rows(); ++k)
            {
                myFileA << result.row(k) << ",";
            }
            myFileA << std::endl;
        }
        myFileA.close();

        std::cout << std::endl;

        for (int i = 0; i < myPath.cols(); i++)
        {
            std::cout << "step " << i << ": " << myPath.col(i).transpose() << " delta distance [m]: " << fk(1, myPath.col(i)).transpose() << std::endl;
        }
        std::cout << "END TEST RBTCONNECT" << std::endl;
    }
}