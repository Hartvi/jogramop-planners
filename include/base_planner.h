
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

        void
        SetEndpoints(MatrixXd &Qe, const VectorXd &q_near, const double &factor) const;

        VectorXd
        GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const;

        // EXAMPLE USAGE OF BASIC FUNCTIONS
        void
        ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal);

        double
        MaxMovedDistance(const VectorXd &q1, const VectorXd &q2) const;

    public:
        std::shared_ptr<URDFEnv> myEnv;

        std::shared_ptr<RobotCollision> myRobot;
    };
}

#endif