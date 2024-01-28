
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
#include "rbt_parameters.h"
#include "rrt_planner.h"

namespace Burs
{
    using namespace Eigen;

    class RbtPlanner : public RRTPlanner
    {
    public:
        RbtPlanner(std::string path_to_urdf_file);
        // int q_dim,
        //        ForwardKinematicsParallel f,
        //        int max_iters,
        //        double d_crit,
        //        double delta_q,
        //        double epsilon_q,
        //        MatrixXd bounds,
        //        RadiusFuncParallel radius_func,
        //        int num_spikes);

        RbtPlanner();

        virtual ~RbtPlanner() = default;

        /// @brief maximum distance of any segment's movement between configuration points q1 and q2
        /// \rho_R(q_1, q_2) = \max_i |f_{p_i}(q_1) - f_{p_i}(q_2)|
        double
        RhoR(const VectorXd &q1, const VectorXd &q2) const;

        /// @brief distance to closest obstacle
        /// \psi(t) = d_c - \rho_R(q, q + t(q_e - q))
        // PhiFunc GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const;
        // double GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e, const double &t) const;

        /// @brief parameter update to iterate to edge configuration
        // t_{k+1} = t_k + \frac{\psi(t_k)}{\sum_{i=1}^n r_i(t_k)|q_e_i - q_{k_i}|(1 - t_k)}
        double
        GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        // void
        // SetEndpoints(MatrixXd &Qe, const VectorXd &q_near, double factor) const;

        // std::optional<std::vector<Eigen::VectorXd>>
        // RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result);

        /// @brief Plan path using two opposing trees
        /// @return Matrix (q_dim, n), where n is the number of steps. OTHERWISE `VectorXd()` if planning fails
        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters);

        // std::vector<Eigen::VectorXd>
        // Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest);

        AlgorithmState
        BurConnect(std::shared_ptr<BurTree> t, VectorXd &q, const RbtParameters &plan_parameters);

        Bur
        GetBur(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest);

        std::vector<MatrixXd>
        GetSteppedEndpoints(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest);

        // AlgorithmState
        // GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, Eigen::VectorXd q_a, const RbtParameters &planner_parameters);

        // AlgorithmState
        // GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, Eigen::VectorXd closest_q, const RbtParameters &planner_parameters);

        std::vector<Eigen::VectorXd>
        Densify(const VectorXd &src, const VectorXd &tgt, const RbtParameters &plan_params);

    protected:
        bool checkGround;

        // int num_spikes;
        // int max_iters;
        // double d_crit;
        // double delta_q;
        // double epsilon_q;
    };

}

#endif