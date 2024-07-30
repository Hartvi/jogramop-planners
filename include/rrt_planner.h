
#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <Eigen/Dense>
#include <string>
#include "base_planner.h"
#include "rrt_parameters.h"
#include "planning_result.h"

namespace Burs
{
    using namespace Eigen;

    // enum StepState
    // {
    //     Advanced,
    //     Crashed
    // };

    class RRTPlanner : public BasePlanner
    {
    public:
        RRTPlanner(std::string path_to_urdf_file);
        RRTPlanner();

        virtual ~RRTPlanner() = default;

        int
        RRTStepInQ(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Qunit &epsilon_q, const Meters &p_step, const DistanceEstimateType &det = DistanceEstimateType::None) const;

        std::optional<std::vector<VectorXd>>
        RRTConnectBasic(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        std::optional<std::vector<VectorXd>>
        RRTConnectQStep(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        AlgorithmState
        GreedyExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const;

        AlgorithmState
        ExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters) const;

    protected:
    };

}

#endif
