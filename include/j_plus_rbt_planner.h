#include "rand_nums.h"
#include "base_planner.h"
#include "rbt_planner.h"
#include "collision_env.h"
#include "robot_base.h"
#include "bur_tree.h"

#ifndef J_PLUS_RBT_PLANNER_H
#define J_PLUS_RBT_PLANNER_H

namespace Burs
{

    struct JPlusRbtParameters : RbtParameters
    {
        double p_close_enough;
        double probability_to_steer_to_target;
        std::shared_ptr<BurTree> target_poses;
        Eigen::Vector3d mean_target;
    };

    class JPlusRbtPlanner : public RbtPlanner
    {

    public:
        std::shared_ptr<RandomNumberGenerator> rng;

    public:
        JPlusRbtPlanner(std::string urdf_file);

        AlgorithmState
        CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const JPlusRbtParameters &planner_parameters, unsigned int &closest_index);

        std::optional<std::vector<Eigen::VectorXd>>
        JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters);

        Bur
        ExtendTowardsCartesian(const VectorXd &q_near, const JPlusRbtParameters &planner_parameters, const double &closest_distance);

        std::vector<Eigen::VectorXd>
        ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id);

        Eigen::Vector3d
        GetMeanTranslation(std::vector<KDL::Frame> &target_poses);

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

        static std::shared_ptr<BurTree>
        ConstructTreeFromTargets(std::vector<KDL::Frame> &target_poses);
    };
}

#endif
