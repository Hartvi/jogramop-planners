
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

        std::optional<std::vector<Eigen::VectorXd>>
        RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result);

        std::vector<Eigen::VectorXd>
        Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest);

        AlgorithmState
        GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, Eigen::VectorXd q_a, const RRTParameters &planner_parameters);

        AlgorithmState
        GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, Eigen::VectorXd closest_q, const RRTParameters &planner_parameters);

    protected:
        RadiusFuncParallel radius_func;
        ForwardKinematicsParallel forwardKinematicsParallel;
        bool checkGround;
    };

}

#endif
