
#ifndef J_PLUS_RBT_PLANNER_H
#define J_PLUS_RBT_PLANNER_H

#include "rand_nums.h"
#include "base_planner.h"
#include "rbt_planner.h"
#include "collision_env.h"
#include "robot_base.h"
#include "bur_tree.h"
#include "j_plus_rbt_parameters.h"
#include "planning_result.h"

namespace Burs
{

    class JPlusRbtPlanner : public RbtPlanner
    {

    public:
        std::shared_ptr<RandomNumberGenerator> rng;

    public:
        JPlusRbtPlanner(std::string urdf_file);

        AlgorithmState
        CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const JPlusRbtParameters &planner_parameters, int &closest_index, double &distance_to_goal);

        std::optional<std::vector<Eigen::VectorXd>>
        JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        Bur
        ExtendTowardsCartesian(const VectorXd &q_near, const JPlusRbtParameters &planner_parameters, const double &closest_distance);

        std::vector<Eigen::VectorXd>
        ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id);

        int
        AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t);

        void
        SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t);

        std::string
        ToString(const Eigen::VectorXd &q_in, bool include_obstacles);

        std::string
        StringifyPath(std::vector<Eigen::VectorXd> path);

        static std::vector<Eigen::VectorXd>
        InterpolatePath(std::vector<Eigen::VectorXd> path, Qunit threshold = 1.0);
    };
}

#endif
