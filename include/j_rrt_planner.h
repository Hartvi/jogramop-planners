
#ifndef J_RRT_PLANNER_H
#define J_RRT_PLANNER_H

#include "rand_nums.h"
#include "rbt_planner.h"
#include "robot_base.h"
// #include "bur_tree.h"
#include "planning_result.h"
#include "grasps.h"
#include "rand_nums.h"
#include "j_plus_rbt_parameters.h"

namespace Burs
{

    class JRRTPlanner : public RbtPlanner
    {
    public:
        std::shared_ptr<RandomNumberGenerator> rng;

    public:
        JRRTPlanner(std::string path_to_urdf_file);
        JRRTPlanner();

        std::optional<std::vector<VectorXd>>
        JRRT(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result);

        KDL::Twist
        GetTwist(const KDL::Frame &tgt, const KDL::Frame &src, const double &max_dist) const;

        AlgorithmState
        ExtendToGoalRRT(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const;
    };
}

#endif