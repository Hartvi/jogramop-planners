
#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "base_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"
#include "min_planner.h"

namespace Burs
{
    using namespace Eigen;

    class BasePlanner : public MinPlanner
    {
    public:
        BasePlanner(std::string path_to_urdf_file);

        BasePlanner();

        virtual ~BasePlanner() = default;

        VectorXd
        GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const;

        // EXAMPLE USAGE OF BASIC FUNCTIONS
        void
        ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal);

        std::vector<Eigen::VectorXd>
        ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id);
    };
}

#endif