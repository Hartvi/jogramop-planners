
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

        double
        GetDeltaTkGeometry(double phi_tk, double tk, const RS &end_state, const RS &k_state) const;

        double
        GetDeltaTkGeneral(const RS &state_near, double tk, const RS &end_state, const RS &k_state, const DistanceEstimateType &det, const bool &expanded_bubble = true) const;

        std::vector<RS>
        GetEndpointsGeneral(RS &state_near, const std::vector<RS> &rand_states, const DistanceEstimateType &det, const size_t &max_iters = 5, const bool &expanded_bubble = true);

        std::vector<RS>
        GetEndpointsGeometry(const RS &state_near, const std::vector<RS> &rand_states, double d_max);

        std::vector<RS>
        GetEndpointsCollisionCheck(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const;

        std::vector<RS>
        GetEndpointsCollisionCheckOriginal(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const;

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
        RS last_state;
        bool finished = false;
    };
}

#endif