#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "env_related/base_env.h"
#include "bur_related/bur_funcs.h"
#include "bur_related/bur_tree.h"

#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

namespace Burs
{
    using namespace Eigen;

    enum AlgorithmState
    {
        Reached,
        Trapped,
        Failure
    };

    class BasePlanner
    {
    public:
        // BasePlanner() = default;

        BasePlanner(int q_dim,
                    ForwardKinematicsParallel f,
                    int max_iters,
                    double d_crit,
                    double delta_q,
                    double epsilon_q,
                    MatrixXd bounds,
                    RadiusFuncParallel radius_func,
                    int num_spikes);

        BasePlanner();

        ~BasePlanner();

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

        /// @brief Get set of random configurations
        MatrixXd
        GetRandomQ(const int &num_spikes) const;

        void
        GetEndpoints(MatrixXd &Qe, const VectorXd &q_near, double factor) const;

        /// @brief normalize spine to some length
        // q_{e_i} \leftarrow q_{near} + \delta \frac{q_{e_i} - q_{near}}{\| q_{e_i} - q_{near} \|}
        VectorXd
        GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const;

        /// @brief Plan path using two opposing trees
        /// @return Matrix (q_dim, n), where n is the number of steps. OTHERWISE `VectorXd()` if planning fails
        std::optional<std::vector<Eigen::VectorXd>>
        RbtConnect(const VectorXd &q_start, const VectorXd &q_goal);

        void
        SetBurEnv(std::shared_ptr<BaseEnv> bur_env);

        template <typename T>
        std::shared_ptr<T>
        GetBurEnv();

        std::vector<Eigen::VectorXd> Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest);

        AlgorithmState
        BurConnect(std::shared_ptr<BurTree> t, VectorXd &q);

        bool
        IsColliding(const VectorXd &q);

        double
        GetClosestDistance(const VectorXd &q);

        VectorXd
        Nearest(std::shared_ptr<BurTree> t, VectorXd &q);

        int
        NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q);

        Bur
        GetBur(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest);

        std::shared_ptr<BaseEnv> bur_env;

    protected:
        RadiusFuncParallel radius_func;
        ForwardKinematicsParallel forwardKinematicsParallel;
        int num_spikes;
        int q_dim;
        MatrixXd bounds;
        int max_iters;
        double d_crit;
        double delta_q;
        double epsilon_q;
    };

    template <typename T>
    std::shared_ptr<T>
    BasePlanner::GetBurEnv()
    {
        std::shared_ptr<T> ret_env = std::dynamic_pointer_cast<T>(bur_env);
        if (ret_env != nullptr)
        {
            return ret_env;
        }
        else
        {
            throw std::runtime_error("Could not get BurEnv. The actual type is not " + std::string(typeid(T).name()) + ".");
        }
    }
}

#endif