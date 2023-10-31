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

namespace test
{
    using namespace std;
    using namespace Burs;

    const int q_dimensionality = 3;

    using VectorQd = Eigen::Matrix<double, q_dimensionality, 1>;

    // using ForwardKinematics = std::function<Eigen::Vector3d(const int &, const VectorQd &)>;

    Eigen::Vector3d TranslationForward(const VectorQd &configuration)
    {
        Eigen::Vector3d v;
        v << 0, 0, 1;
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx * v;
    }

    Eigen::Matrix3d RotationForward(const VectorQd &configuration)
    {
        Eigen::Vector3d v;
        v << 0, 0, 1;
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx;
    }

    std::tuple<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>> StickRt(const Vector3d &configuration)
    {
        return {{RotationForward(configuration)}, {TranslationForward(configuration)}};
    }

    Eigen::Vector3d ForwardDistal(const int &ith_distal_point, const VectorQd &configuration)
    {
        if (ith_distal_point > 2 || ith_distal_point < 0)
        {
            throw new std::runtime_error("Invalid index for joint/end effector");
        }
        Eigen::Vector3d v;
        v << 0, 0, 2 * (ith_distal_point == 2);
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx * v;
    }

    double StickRadiusFunc(const int &ith_distal_point, const VectorXd &q_k)
    {
        // radius of cylinder in joint axis that encompasses end effector
        if (ith_distal_point > 2 || ith_distal_point < 0)
        {
            throw new std::runtime_error("Invalid index for joint/end effector");
        }
        // radius of cylinder in joint axis that encompasses end effector
        return 2.0;
    }

    void test_forward(int argc, char *argv[])
    {
        if (argc < 3)
        {
            std::cout << "Usage: ./exc robot_model.obj obstacle_model.obj" << std::endl;
            return;
        }
        Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(q_dimensionality, 2);
        for (int i = 0; i < q_dimensionality; ++i)
        {
            bounds.col(0).setConstant(-M_PI);
            bounds.col(1).setConstant(M_PI);
        }
        std::cout << "bounds:" << bounds << std::endl;

        Burs::ForwardKinematics fk = ForwardDistal;
        // BurNodeWrapper(ForwardKinematics f, int num_segments, int max_iters, double d_crit, double delta_q, double epsilon_q);
        std::unique_ptr<Burs::BurAlgorithm> b = std::make_unique<Burs::BurAlgorithm>(q_dimensionality, fk, 3, 1, 0.1, M_PI, M_PI / 10.0, bounds, StickRadiusFunc, 7);

        int point = 1;
        VectorQd config = VectorQd::Random();
        VectorQd config2 = VectorQd::Random();

        std::cout << "config1: " << config.transpose() << std::endl;
        std::cout << "config2: " << config2.transpose() << std::endl;

        Eigen::Vector3d result = b->ForwardEuclideanJoint(2, config); // Call the function through the std::function wrapper

        // Output
        std::cout << "Result: " << result.transpose() << std::endl;

        // TESTING TOGETHER

        std::shared_ptr<Burs::BurEnv> b_env = std::make_shared<Burs::BurEnv>();

        std::shared_ptr<TrPQPModel> robot_model = std::make_shared<TrPQPModel>(argv[1]);
        std::shared_ptr<TrPQPModel> obstacle_model = std::make_shared<TrPQPModel>(argv[2]);

        b_env->AddRobotModel(robot_model);
        b_env->AddForwardRt(StickRt);
        b_env->AddObstacleModel(obstacle_model);

        for (int i = 0; i < b_env->obstacle_models.size(); i++)
        {
            // for cube
            std::cout << "Obstacle position: " << Eigen::Vector3d(b_env->obstacle_models[i]->getT()).transpose() << std::endl;
            b_env->obstacle_models[i]->SetTranslation(Vector3d(std::atoi(argv[3]), std::atoi(argv[4]), std::atoi(argv[5])));
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
        std::cout << "Run RbtConnect" << std::endl;
        auto path = b->RbtConnect(config, config2);
        std::cout << "Rbt connected" << std::endl;
        for (int i = 0; i < path.size(); i++)
        {
            std::cout << "step:" << i << path[i].transpose() << std::endl;
        }
    }

}