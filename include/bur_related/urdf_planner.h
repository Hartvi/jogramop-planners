#include "bur_related/base_planner.h"
#include "env_related/collision_env.h"
#include "robot_related/robot_base.h"

#ifndef URDF_PLANNER_H
#define URDF_PLANNER_H

namespace Burs
{
    class URDFPlanner
    {
    public:
        std::shared_ptr<CollisionEnv> mCollisionEnv;
        std::shared_ptr<BasePlanner> mBasePlanner;

        URDFPlanner(std::string urdf_file, int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes);

        unsigned int
        GetNrOfJoints();

        std::optional<std::vector<Eigen::VectorXd>>
        PlanPath(Eigen::VectorXd start, Eigen::VectorXd goal);

        int
        AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t);

        void
        SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t);

        std::string
        ToString(const Eigen::VectorXd &q_in);
    };
}

#endif
