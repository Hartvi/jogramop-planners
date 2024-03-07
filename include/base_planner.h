
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
        GetDeltaTk(double phi_tk, double tk, const RS &end_state, const RS &k_state) const;

        // double
        // GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        std::vector<RS>
        QToStates(const MatrixXd &Q) const;

        std::vector<std::vector<RS>>
        GetEndpointsInterstates(const RS &state_near, const std::vector<RS> &rand_states, double d_max, double q_resolution) const;

        std::vector<RS>
        GetEndpoints(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const;

        // EXAMPLE USAGE OF BASIC FUNCTIONS
        void
        ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal);

        int
        AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t);

        void
        SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t);

        std::string
        ToString(const VectorXd &state, bool obstacles);

        std::string
        StringifyPath(std::vector<VectorXd> path);

        std::string
        ConfigsToString(const std::vector<VectorXd> &path);

        std::string
        TreePoints(const std::shared_ptr<BurTree> t, const int &one_out_of) const;

    public:
        std::string tree_csv;
    };
}

#endif