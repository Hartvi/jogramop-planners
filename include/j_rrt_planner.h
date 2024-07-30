
#ifndef J_RRT_PLANNER_H
#define J_RRT_PLANNER_H

#include "rand_nums.h"
#include "rbte_planner.h"
#include "robot_base.h"
// #include "bur_tree.h"
#include "planning_result.h"
#include "grasps.h"
#include "rand_nums.h"
#include "j_plus_rbt_parameters.h"

namespace Burs
{

    class JRRTPlanner : public RbtePlanner
    {
    public:
        std::shared_ptr<RandomNumberGenerator> rng;

    public:
        JRRTPlanner(std::string path_to_urdf_file);
        JRRTPlanner();

        std::optional<std::vector<VectorXd>>
        JRRT(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result);

        KDL::Twist
        GetTwist(const KDL::Frame &tgt, const KDL::Frame &src, const double &max_dist, const bool &use_rot) const;

        KDL::Vector
        GetRotVec(const KDL::Frame &tgt, const KDL::Frame &src) const;

        Eigen::Matrix3d
        ProjectApproachDirection(const Eigen::Matrix3d &rotMatGrasp, const Eigen::Matrix3d &rotMatEE) const;

        AlgorithmState
        ExtendToGoalRRT(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const;

        std::optional<std::vector<VectorXd>>
        RotTest(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result);

        std::shared_ptr<BurTree>
        JRRTPreheat(VectorXd q_start, int iters, JPlusRbtParameters &planner_parameters);
    };
}

#endif