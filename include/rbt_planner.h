
#ifndef RBT_PLANNER_H
#define RBT_PLANNER_H
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "base_planner.h"
#include "base_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"
#include "j_plus_rbt_parameters.h"
#include "rrt_planner.h"

namespace Burs
{
    using namespace Eigen;

    class RbtPlanner : public RRTPlanner
    {
    public:
        RbtPlanner(std::string path_to_urdf_file);

        RbtPlanner();

        virtual ~RbtPlanner() = default;

        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        std::pair<AlgorithmState, int>
        BurConnect(std::shared_ptr<BurTree> t, const RS &state, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist);

        void
        InitGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &start_idx) const;

        double
        SetGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &state_idx) const;

        unsigned int
        GetBestGrasp(JPlusRbtParameters &planner_parameters) const;

        std::vector<RS>
        Densify(const RS &src, const RS &tgt, const RbtParameters &plan_params) const;

        std::optional<std::vector<Eigen::VectorXd>>
        TestCollisionVsDistanceTime(const VectorXd &q_start, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        std::pair<double, KDL::Frame>
        BasicDistanceMetric(const KDL::Frame &ee, const KDL::Frame &tgt, const double &angle_ratio) const;

        std::pair<bool, KDL::Rotation>
        GetClosestSymmetricGrasp(const KDL::Rotation &rotMatGrasp, const KDL::Rotation &rotMatEE) const;

    protected:
        bool checkGround;
    };

}

#endif