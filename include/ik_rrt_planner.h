
#ifndef IKRRT_PLANNER_H
#define IKRRT_PLANNER_H

#include <Eigen/Dense>
#include <string>
#include "j_rrt_planner.h"
#include "j_plus_rbt_parameters.h"
#include "planning_result.h"

namespace Burs
{
    using namespace Eigen;

    class IKRRTPlanner : public JRRTPlanner
    {
    public:
        IKRRTPlanner(std::string path_to_urdf_file);
        IKRRTPlanner() = default;

        virtual ~IKRRTPlanner() = default;

        std::optional<std::vector<VectorXd>>
        IKRRT(const VectorXd &q_start, const JPlusRbtParameters &plan_parameters, PlanningResult &planning_result);
    };

}

#endif
