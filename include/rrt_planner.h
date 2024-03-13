
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

    class RRTPlanner : public BasePlanner
    {
    public:
        RRTPlanner(std::string path_to_urdf_file);
        RRTPlanner();

        virtual ~RRTPlanner() = default;

        int
        RRTStepInQ(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Qunit &epsilon_q, const Meters &p_step, const bool &full_state = false) const;

        // int
        // RRTStepInQ(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Meters &epsilon_q, const Meters &p_step) const;

        int
        RRTStep(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Meters &epsilon_q) const;

        std::optional<std::vector<VectorXd>>
        RRTConnectQStep(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        // AlgorithmState
        // GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, Eigen::VectorXd q_a, const RRTParameters &planner_parameters);

        AlgorithmState
        GreedyExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const;

        AlgorithmState
        GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const;

        std::optional<std::vector<VectorXd>>
        TestSampling(const VectorXd &q_start, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        void
        GenerateRandomSamples(std::shared_ptr<BurTree> t, int num_samples);

    protected:
        // RadiusFuncParallel radius_func;
        // ForwardKinematicsParallel forwardKinematicsParallel;
        // bool checkGround;
    };

}

#endif
