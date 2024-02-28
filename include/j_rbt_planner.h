#ifndef J_RBT_PLANNER_H
#define J_RBT_PLANNER_H

#include "rand_nums.h"
#include "j_rrt_planner.h"
#include "robot_base.h"
#include "bur_tree.h"
#include "j_plus_rbt_parameters.h"
#include "planning_result.h"
#include "grasps.h"
#include <vector>

namespace Burs
{

    class JRbtPlanner : public JRRTPlanner
    {

    public:
        JRbtPlanner(std::string urdf_file);

        // std::optional<std::vector<Eigen::VectorXd>>
        // JRbt(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        // std::optional<std::vector<Eigen::VectorXd>>
        // JRbtS(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &planning_result);

        std::optional<std::vector<VectorXd>>
        JRbtBasic(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result);

        AlgorithmState
        ExtendToGoalRbt(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const;

        std::pair<std::vector<double>, std::vector<RS>>
        ExtendToGoalRbtStep(std::shared_ptr<BurTree> tree, const int &idx_near, JPlusRbtParameters &planner_parameters) const;

        std::vector<Grasp>
        GetBestAndRandomGrasps(JPlusRbtParameters &planner_parameters) const;
    };
}

#endif
