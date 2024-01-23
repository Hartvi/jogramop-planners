
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

        // AlgorithmState, VectorXd &last_q
        // ExtendToGoal(const Eigen::VectorXd &q_near, const JPlusRbtParameters &planner_parameters);

        // AlgorithmState
        // GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, Eigen::VectorXd q_a, const JPlusRbtParameters &planner_parameters);

        // AlgorithmState
        // GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, Eigen::VectorXd closest_q, const JPlusRbtParameters &planner_parameters);

        std::optional<std::vector<Eigen::VectorXd>>
        RRTMultiGoal(const VectorXd &q_start, std::vector<VectorXd> &q_goals, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        RbtMultiGoal(const VectorXd &q_start, std::vector<VectorXd> &q_goals, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        BiJPlusRbtMultiGoal(const VectorXd &q_start, std::vector<VectorXd> &q_goals, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        // BiJPlusRbt(const VectorXd &q_start, const std::vector<VectorXd> &q_goal, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);
        BiJPlusRbt(const VectorXd &q_start, VectorXd &q_goal, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>> JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        Bur
        ExtendTowardsCartesian(const VectorXd &q_near, const JPlusRbtParameters &planner_parameters, const double &closest_distance);

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

        static std::vector<Eigen::VectorXd>
        InterpolatePath(std::vector<Eigen::VectorXd> path, Qunit threshold = 1.0);
    };
}

#endif
