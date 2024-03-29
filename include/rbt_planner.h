
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

        /// @brief maximum distance of any segment's movement between configuration points q1 and q2
        /// \rho_R(q_1, q_2) = \max_i |f_{p_i}(q_1) - f_{p_i}(q_2)|
        // double
        // RhoR(const VectorXd &q1, const VectorXd &q2) const;

        /// @brief distance to closest obstacle
        /// \psi(t) = d_c - \rho_R(q, q + t(q_e - q))
        // PhiFunc GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const;
        // double GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e, const double &t) const;

        /// @brief parameter update to iterate to edge configuration
        // t_{k+1} = t_k + \frac{\psi(t_k)}{\sum_{i=1}^n r_i(t_k)|q_e_i - q_{k_i}|(1 - t_k)}
        // double
        // GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnectDenseBurs(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        std::pair<AlgorithmState, int>
        BurConnect(std::shared_ptr<BurTree> t, const RS &state, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist);

        int
        AddPointsExceptFirst(std::shared_ptr<BurTree> t, const int &first_el_idx, const std::vector<RS> vec) const;

        std::vector<double>
        AddDenseBur(std::shared_ptr<BurTree> tree, const int &idx_near, const std::vector<RS> &endpoints, JPlusRbtParameters &plan_params) const;

        std::vector<RS>
        Densify(const RS &src, const RS &tgt, const RbtParameters &plan_params) const;

        void
        InitGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &start_idx) const;

        double
        SetGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &state_idx) const;

        unsigned int
        GetBestGrasp(JPlusRbtParameters &planner_parameters) const;

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