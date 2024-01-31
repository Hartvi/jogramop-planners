
#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "base_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"
#include "min_planner.h"

#include "robot_collision.h"
#include "collision_env.h"

namespace Burs
{
    using namespace Eigen;

    class BasePlanner : public MinPlanner
    {
    public:
        BasePlanner(std::string path_to_urdf_file);

        BasePlanner();

        virtual ~BasePlanner() = default;

        double
        GetDistToGoal(const VectorXd &q, const KDL::Vector &goal_pos) const;

        KDL::Frame
        GetEEPose(const VectorXd &q) const;

        double
        GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        MatrixXd
        GetEndpoints(const VectorXd &q_near, const MatrixXd &Q_e, double d_max) const;

        // EXAMPLE USAGE OF BASIC FUNCTIONS
        void
        ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal);

        double
        MaxMovedDistance(const VectorXd &q1, const VectorXd &q2) const;

        int
        AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t);

        void
        SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t);

        std::string
        ToString(const Eigen::VectorXd &q_in, bool include_obstacles);

        std::string
        StringifyPath(std::vector<Eigen::VectorXd> path);

        std::string
        ConfigsToString(const std::vector<Eigen::VectorXd> &path);

    public:
        std::shared_ptr<URDFEnv> myEnv;

        std::shared_ptr<RobotCollision> myRobot;

        RadiusFuncParallel radius_func;
        ForwardKinematicsParallel forwardKinematicsParallel;
    };
}

#endif