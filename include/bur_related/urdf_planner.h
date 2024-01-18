#include "bur_related/base_planner.h"
#include "env_related/collision_env.h"
#include "robot_related/robot_base.h"

#ifndef URDF_PLANNER_H
#define URDF_PLANNER_H

namespace Burs
{
    class URDFPlanner : public BasePlanner
    {
    public:
        // std::shared_ptr<CollisionEnv> mCollisionEnv;
        // std::shared_ptr<BasePlanner> mBasePlanner;

        URDFPlanner(std::string urdf_file, int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes);

        template <typename T>
        std::vector<T>
        SelectRandomElements(std::vector<T> &vec, size_t N);

        AlgorithmState
        CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const std::vector<KDL::Frame> &target_poses, const double &p_close_enough, unsigned int &closest_index);

        std::optional<std::vector<Eigen::VectorXd>>
        JPlusRbt(const VectorXd &q_start, std::vector<KDL::Frame> &target_poses, const double &probability_to_steer_to_target = 0.1, const double &p_close_enough = 0.15);

        std::vector<Eigen::VectorXd>
        ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id);

        unsigned int
        GetNrOfJoints();

        std::optional<std::vector<Eigen::VectorXd>>
        PlanPath(Eigen::VectorXd start, Eigen::VectorXd goal);

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
