
#ifndef RBTE_PLANNER_H
#define RBTE_PLANNER_H
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "base_planner.h"
#include "base_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"
#include "j_plus_rbt_parameters.h"
#include "rbt_planner.h"

namespace Burs
{
    using namespace Eigen;

    class RbtePlanner : public RbtPlanner
    {
    public:
        RbtePlanner(std::string path_to_urdf_file);

        RbtePlanner();

        virtual ~RbtePlanner() = default;

        std::optional<std::vector<Eigen::VectorXd>>
        RbteConnect(const VectorXd &q_start, const VectorXd &q_goal, const JPlusRbtParameters &plan_parameters, PlanningResult &planning_result);

        void
        TestFunctions();

        std::vector<RS>
        CreateExtendedBur(RS &near_state, const JPlusRbtParameters &plan_parameters);
    };

}

#endif